#!/usr/bin/env python3
import base64
import os
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from typing import Dict, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse, JSONResponse, Response, StreamingResponse
from geometry_msgs.msg import Twist
from pydantic import BaseModel, Field
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image


@dataclass
class EncodedFrame:
    seq: int
    stamp_ns: int
    jpeg_bytes: bytes


class CmdVelRequest(BaseModel):
    linear_x: float = Field(default=0.0, description="Forward/backward m/s")
    linear_y: float = Field(default=0.0, description="Left/right m/s")
    angular_z: float = Field(default=0.0, description="Yaw rad/s")


class WebGatewayNode(Node):
    def __init__(self) -> None:
        super().__init__("web_gateway")

        self.bridge = CvBridge()
        self.lock = threading.Lock()

        self.host = self.declare_parameter("host", "0.0.0.0").get_parameter_value().string_value
        self.port = int(self.declare_parameter("port", 8000).get_parameter_value().integer_value)
        self.jpeg_quality = int(
            self.declare_parameter("jpeg_quality", 80).get_parameter_value().integer_value
        )
        self.depth_max_m = float(
            self.declare_parameter("depth_max_m", 12.0).get_parameter_value().double_value
        )
        self.cmd_timeout_sec = float(
            self.declare_parameter("cmd_timeout_sec", 0.5).get_parameter_value().double_value
        )
        self.cmd_max_hz = max(
            1.0,
            float(self.declare_parameter("cmd_max_hz", 6.0).get_parameter_value().double_value),
        )
        self.record_output_dir = os.path.expanduser(
            self.declare_parameter("record_output_dir", "/home/test").get_parameter_value().string_value
        )
        self.record_save_interval_sec = float(
            self.declare_parameter("record_save_interval_sec", 1.0).get_parameter_value().double_value
        )
        self.record_sync_tolerance_sec = float(
            self.declare_parameter("record_sync_tolerance_sec", 0.6).get_parameter_value().double_value
        )
        self.record_require_lidar = bool(
            self.declare_parameter("record_require_lidar", True).get_parameter_value().bool_value
        )
        self.record_imu_save_hz = float(
            self.declare_parameter("record_imu_save_hz", 20.0).get_parameter_value().double_value
        )
        self.record_imu_window_sec = float(
            self.declare_parameter("record_imu_window_sec", 0.1).get_parameter_value().double_value
        )
        self.record_pose_topic = self.declare_parameter(
            "record_pose_topic", "/world/small_house/dynamic_pose/info"
        ).get_parameter_value().string_value
        self.record_run_prefix = self.declare_parameter(
            "record_run_prefix", "run"
        ).get_parameter_value().string_value
        self.dataset_record_output_dir = os.path.expanduser(
            self.declare_parameter(
                "dataset_record_output_dir", "/home/test/dataset"
            ).get_parameter_value().string_value
        )
        self.dataset_record_save_interval_sec = float(
            self.declare_parameter("dataset_record_save_interval_sec", 1.0).get_parameter_value().double_value
        )
        self.dataset_record_sync_tolerance_sec = float(
            self.declare_parameter("dataset_record_sync_tolerance_sec", 0.6).get_parameter_value().double_value
        )
        self.dataset_record_require_lidar = bool(
            self.declare_parameter("dataset_record_require_lidar", True).get_parameter_value().bool_value
        )
        self.dataset_record_pose_topic = self.declare_parameter(
            "dataset_record_pose_topic", "/world/small_house/dynamic_pose/info"
        ).get_parameter_value().string_value
        self.cmd_period_sec = 1.0 / self.cmd_max_hz

        self.frames: Dict[str, Optional[EncodedFrame]] = {
            "front": None,
            "rear": None,
            "depth": None,
        }
        self.frame_seq = 0

        self.cmd_pub = self.create_publisher(Twist, "/model/smart_agent/cmd_vel", 10)
        self.last_cmd_time = time.monotonic()
        self.last_cmd_nonzero = False
        self._record_lock = threading.Lock()
        self._recorder_proc: Optional[subprocess.Popen] = None
        self._dataset_record_lock = threading.Lock()
        self._dataset_recorder_proc: Optional[subprocess.Popen] = None
        self.create_timer(0.1, self._watchdog_cb)

        self.create_subscription(Image, "/agent/front_camera/image_raw", self._front_cb, 10)
        self.create_subscription(Image, "/agent/rear_camera/image_raw", self._rear_cb, 10)
        self.create_subscription(Image, "/agent/depth_camera/image_raw", self._depth_cb, 10)

        self.get_logger().info(
            "Web gateway ready. "
            f"host={self.host}, port={self.port}, jpeg_quality={self.jpeg_quality}, "
            f"cmd_max_hz={self.cmd_max_hz:.1f}, record_output_dir={self.record_output_dir}"
        )

    def _stamp_to_ns(self, msg) -> int:
        sec = int(msg.header.stamp.sec)
        nsec = int(msg.header.stamp.nanosec)
        if sec == 0 and nsec == 0:
            return self.get_clock().now().nanoseconds
        return sec * 1_000_000_000 + nsec

    def _encode_bgr_jpeg(self, bgr_image: np.ndarray) -> bytes:
        ok, encoded = cv2.imencode(
            ".jpg",
            bgr_image,
            [int(cv2.IMWRITE_JPEG_QUALITY), int(np.clip(self.jpeg_quality, 10, 100))],
        )
        if not ok:
            raise RuntimeError("jpeg encode failed")
        return encoded.tobytes()

    def _store_frame(self, key: str, stamp_ns: int, jpeg_bytes: bytes) -> None:
        with self.lock:
            self.frame_seq += 1
            self.frames[key] = EncodedFrame(
                seq=self.frame_seq,
                stamp_ns=stamp_ns,
                jpeg_bytes=jpeg_bytes,
            )

    def _front_cb(self, msg: Image) -> None:
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self._store_frame("front", self._stamp_to_ns(msg), self._encode_bgr_jpeg(bgr))

    def _rear_cb(self, msg: Image) -> None:
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self._store_frame("rear", self._stamp_to_ns(msg), self._encode_bgr_jpeg(bgr))

    def _depth_cb(self, msg: Image) -> None:
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if depth.dtype == np.float32:
            depth_m = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
            depth_u8 = np.clip((depth_m / max(self.depth_max_m, 0.1)) * 255.0, 0, 255).astype(np.uint8)
        elif depth.dtype == np.uint16:
            depth_u8 = np.clip((depth.astype(np.float32) / 1000.0 / max(self.depth_max_m, 0.1)) * 255.0, 0, 255).astype(
                np.uint8
            )
        else:
            depth_u8 = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        color = cv2.applyColorMap(depth_u8, cv2.COLORMAP_TURBO)
        self._store_frame("depth", self._stamp_to_ns(msg), self._encode_bgr_jpeg(color))

    def get_frame(self, key: str) -> Optional[EncodedFrame]:
        with self.lock:
            return self.frames.get(key)

    def _frame_seq(self, key: str) -> int:
        frame = self.get_frame(key)
        return -1 if frame is None else frame.seq

    def wait_for_front_newer_than(self, prev_seq: int, timeout_sec: float = 0.35) -> None:
        deadline = time.monotonic() + max(0.0, timeout_sec)
        while time.monotonic() < deadline:
            if self._frame_seq("front") > prev_seq:
                return
            time.sleep(0.005)

    def get_frame_bundle(self) -> Dict[str, object]:
        bundle_frames: Dict[str, Optional[str]] = {}
        bundle_seq: Dict[str, int] = {}
        for camera in ("front", "rear", "depth"):
            frame = self.get_frame(camera)
            if frame is None:
                bundle_frames[camera] = None
                bundle_seq[camera] = -1
                continue
            encoded = base64.b64encode(frame.jpeg_bytes).decode("ascii")
            bundle_frames[camera] = f"data:image/jpeg;base64,{encoded}"
            bundle_seq[camera] = frame.seq
        return {"frames": bundle_frames, "seq": bundle_seq}

    def _publish_twist(self, cmd: tuple[float, float, float]) -> None:
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = cmd
        self.cmd_pub.publish(msg)

    def publish_cmd_vel(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        cmd = (float(linear_x), float(linear_y), float(angular_z))
        self._publish_twist(cmd)
        self.last_cmd_time = time.monotonic()
        self.last_cmd_nonzero = any(abs(v) > 1e-6 for v in cmd)

    def _watchdog_cb(self) -> None:
        if not self.last_cmd_nonzero:
            return
        if time.monotonic() - self.last_cmd_time < self.cmd_timeout_sec:
            return
        self.publish_cmd_vel(0.0, 0.0, 0.0)
        self.last_cmd_nonzero = False

    def _recorder_running_unlocked(self) -> bool:
        return self._recorder_proc is not None and self._recorder_proc.poll() is None

    def recording_status(self) -> Dict[str, object]:
        with self._record_lock:
            running = self._recorder_running_unlocked()
        return {
            "running": running,
            "output_dir": self.record_output_dir,
            "save_interval_sec": self.record_save_interval_sec,
            "sync_tolerance_sec": self.record_sync_tolerance_sec,
            "require_lidar": self.record_require_lidar,
            "imu_save_hz": self.record_imu_save_hz,
            "imu_window_sec": self.record_imu_window_sec,
            "pose_topic": self.record_pose_topic,
            "run_prefix": self.record_run_prefix,
        }

    def start_recording(self) -> Dict[str, object]:
        with self._record_lock:
            if self._recorder_running_unlocked():
                return {"ok": True, "running": True, "message": "Recorder already running."}

            cmd = [
                sys.executable,
                "-m",
                "smart_agent_gazebo.data_recorder",
                "--ros-args",
                "-p",
                "use_sim_time:=true",
                "-p",
                f"output_dir:={self.record_output_dir}",
                "-p",
                f"save_interval_sec:={self.record_save_interval_sec}",
                "-p",
                f"sync_tolerance_sec:={self.record_sync_tolerance_sec}",
                "-p",
                f"require_lidar:={str(self.record_require_lidar).lower()}",
                "-p",
                f"imu_save_hz:={self.record_imu_save_hz}",
                "-p",
                f"imu_window_sec:={self.record_imu_window_sec}",
                "-p",
                f"pose_topic:={self.record_pose_topic}",
                "-p",
                f"run_prefix:={self.record_run_prefix}",
            ]
            try:
                self._recorder_proc = subprocess.Popen(cmd)
            except Exception as exc:
                self.get_logger().error(f"Failed to start recorder: {exc}")
                self._recorder_proc = None
                return {"ok": False, "running": False, "message": str(exc)}

        self.get_logger().info("Data recorder started from web console.")
        return {"ok": True, "running": True, "message": "Recorder started."}

    def stop_recording(self) -> Dict[str, object]:
        with self._record_lock:
            if not self._recorder_running_unlocked():
                self._recorder_proc = None
                return {"ok": True, "running": False, "message": "Recorder not running."}

            proc = self._recorder_proc
            assert proc is not None
            proc.terminate()
            try:
                proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=1.0)
            self._recorder_proc = None

        self.get_logger().info("Data recorder stopped from web console.")
        return {"ok": True, "running": False, "message": "Recorder stopped."}

    def _dataset_recorder_running_unlocked(self) -> bool:
        return self._dataset_recorder_proc is not None and self._dataset_recorder_proc.poll() is None

    def dataset_recording_status(self) -> Dict[str, object]:
        with self._dataset_record_lock:
            running = self._dataset_recorder_running_unlocked()
        return {
            "running": running,
            "output_dir": self.dataset_record_output_dir,
            "save_interval_sec": self.dataset_record_save_interval_sec,
            "sync_tolerance_sec": self.dataset_record_sync_tolerance_sec,
            "require_lidar": self.dataset_record_require_lidar,
            "pose_topic": self.dataset_record_pose_topic,
        }

    def start_dataset_recording(self) -> Dict[str, object]:
        with self._dataset_record_lock:
            if self._dataset_recorder_running_unlocked():
                return {
                    "ok": True,
                    "running": True,
                    "message": "Dataset recorder already running.",
                }

            cmd = [
                sys.executable,
                "-m",
                "smart_agent_gazebo.dataset_recorder",
                "--ros-args",
                "-p",
                "use_sim_time:=true",
                "-p",
                f"output_dir:={self.dataset_record_output_dir}",
                "-p",
                f"save_interval_sec:={self.dataset_record_save_interval_sec}",
                "-p",
                f"sync_tolerance_sec:={self.dataset_record_sync_tolerance_sec}",
                "-p",
                f"require_lidar:={str(self.dataset_record_require_lidar).lower()}",
                "-p",
                f"pose_topic:={self.dataset_record_pose_topic}",
            ]

            try:
                self._dataset_recorder_proc = subprocess.Popen(cmd)
            except Exception as exc:
                self.get_logger().error(f"Failed to start dataset recorder: {exc}")
                self._dataset_recorder_proc = None
                return {"ok": False, "running": False, "message": str(exc)}

        self.get_logger().info("Dataset recorder started from web console.")
        return {"ok": True, "running": True, "message": "Dataset recorder started."}

    def stop_dataset_recording(self) -> Dict[str, object]:
        with self._dataset_record_lock:
            if not self._dataset_recorder_running_unlocked():
                self._dataset_recorder_proc = None
                return {
                    "ok": True,
                    "running": False,
                    "message": "Dataset recorder not running.",
                }

            proc = self._dataset_recorder_proc
            assert proc is not None
            proc.terminate()
            try:
                proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=1.0)
            self._dataset_recorder_proc = None

        self.get_logger().info("Dataset recorder stopped from web console.")
        return {"ok": True, "running": False, "message": "Dataset recorder stopped."}


def create_app(node: WebGatewayNode) -> FastAPI:
    app = FastAPI(title="Smart Agent Web Gateway", version="1.0.0")

    @app.get("/", response_class=HTMLResponse)
    def index() -> str:
        return """
<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <title>Smart Agent Web Console</title>
  <style>
    body { font-family: sans-serif; margin: 12px; background: #111; color: #ddd; }
    .row { display: flex; gap: 12px; flex-wrap: wrap; }
    .card { background: #1d1d1d; padding: 8px; border-radius: 8px; }
    img { width: 480px; max-width: 45vw; border-radius: 6px; background: #000; }
    pre { max-height: 220px; overflow: auto; background: #000; padding: 8px; border-radius: 6px; }
    .hint { color: #9bd; }
    .ctrl { display: grid; grid-template-columns: repeat(3, 70px); gap: 6px; margin-top: 8px; }
    button { height: 40px; border: 0; border-radius: 6px; background: #2a2a2a; color: #eee; cursor: pointer; }
    button:active { background: #3f3f3f; }
    .wide { grid-column: span 3; }
  </style>
</head>
<body>
  <h2>Smart Agent Headless Console</h2>
  <div class="hint">W/S 前后, A/D 横移, Q/E 转向, 空格急停。每次指令会返回并刷新一帧图像。</div>
  <div class="row">
    <div class="card"><div>Front RGB</div><img id="frontImg" src="" /></div>
    <div class="card"><div>Rear RGB</div><img id="rearImg" src="" /></div>
    <div class="card"><div>Depth (Pseudo Color)</div><img id="depthImg" src="" /></div>
  </div>
  <div class="card" style="margin-top:12px;">
    <div>Control</div>
    <div class="ctrl">
      <button data-k="q">Q</button><button data-k="w">W</button><button data-k="e">E</button>
      <button data-k="a">A</button><button data-k="s">S</button><button data-k="d">D</button>
      <button id="stop" class="wide">STOP (Space)</button>
    </div>
    <pre id="cmd">cmd: {"linear_x":0,"linear_y":0,"angular_z":0}</pre>
  </div>
  <div class="card" style="margin-top:12px;">
    <div>Recording</div>
    <div class="ctrl" style="grid-template-columns: repeat(1, 220px);">
      <button id="recToggle">Start Recording</button>
    </div>
    <pre id="recStatus">recording: unknown</pre>
  </div>
  <div class="card" style="margin-top:12px;">
    <div>Dataset Recording</div>
    <div class="ctrl" style="grid-template-columns: repeat(1, 220px);">
      <button id="datasetRecToggle">Start Dataset Recording</button>
    </div>
    <pre id="datasetRecStatus">dataset recording: unknown</pre>
  </div>
  <script>
    const speed = {x: 2.0, y: 2.0, z: 2.6};
    const keepaliveMs = 220;
    const keys = new Set();
    let inFlight = false;
    let pendingCmd = null;
    let lastSentSig = '';
    let lastSentAt = 0;
    let recording = false;
    let datasetRecording = false;

    function computeCmd() {
      let linear_x = 0.0, linear_y = 0.0, angular_z = 0.0;
      if (keys.has('w')) linear_x += speed.x;
      if (keys.has('s')) linear_x -= speed.x;
      if (keys.has('a')) linear_y += speed.y;
      if (keys.has('d')) linear_y -= speed.y;
      if (keys.has('q')) angular_z += speed.z;
      if (keys.has('e')) angular_z -= speed.z;
      return {linear_x, linear_y, angular_z};
    }
    function cmdSignature(cmd) {
      return `${cmd.linear_x.toFixed(3)},${cmd.linear_y.toFixed(3)},${cmd.angular_z.toFixed(3)}`;
    }
    function isZeroCmd(cmd) {
      return Math.abs(cmd.linear_x) < 1e-6 && Math.abs(cmd.linear_y) < 1e-6 && Math.abs(cmd.angular_z) < 1e-6;
    }
    function renderCmd(cmd) {
      document.getElementById('cmd').textContent = 'cmd: ' + JSON.stringify(cmd);
    }
    function applyFrames(payload) {
      if (!payload || !payload.frames) return;
      const frames = payload.frames;
      if (frames.front) document.getElementById('frontImg').src = frames.front;
      if (frames.rear) document.getElementById('rearImg').src = frames.rear;
      if (frames.depth) document.getElementById('depthImg').src = frames.depth;
    }
    function updateRecordingUI(message = '') {
      const btn = document.getElementById('recToggle');
      const status = document.getElementById('recStatus');
      btn.textContent = recording ? 'Stop Recording' : 'Start Recording';
      status.textContent = `recording: ${recording ? 'ON' : 'OFF'}` + (message ? ` | ${message}` : '');
    }
    async function syncRecordingStatus() {
      try {
        const resp = await fetch('/recording/status');
        if (!resp.ok) return;
        const data = await resp.json();
        recording = !!data.running;
        updateRecordingUI();
      } catch (_) {}
    }
    async function toggleRecording() {
      const endpoint = recording ? '/recording/stop' : '/recording/start';
      try {
        const resp = await fetch(endpoint, {method: 'POST'});
        if (!resp.ok) {
          updateRecordingUI('request failed');
          return;
        }
        const data = await resp.json();
        recording = !!data.running;
        updateRecordingUI(data.message || '');
      } catch (_) {
        updateRecordingUI('request error');
      }
    }
    function updateDatasetRecordingUI(message = '') {
      const btn = document.getElementById('datasetRecToggle');
      const status = document.getElementById('datasetRecStatus');
      btn.textContent = datasetRecording ? 'Stop Dataset Recording' : 'Start Dataset Recording';
      status.textContent = `dataset recording: ${datasetRecording ? 'ON' : 'OFF'}` + (message ? ` | ${message}` : '');
    }
    async function syncDatasetRecordingStatus() {
      try {
        const resp = await fetch('/dataset_recording/status');
        if (!resp.ok) return;
        const data = await resp.json();
        datasetRecording = !!data.running;
        updateDatasetRecordingUI();
      } catch (_) {}
    }
    async function toggleDatasetRecording() {
      const endpoint = datasetRecording ? '/dataset_recording/stop' : '/dataset_recording/start';
      try {
        const resp = await fetch(endpoint, {method: 'POST'});
        if (!resp.ok) {
          updateDatasetRecordingUI('request failed');
          return;
        }
        const data = await resp.json();
        datasetRecording = !!data.running;
        updateDatasetRecordingUI(data.message || '');
      } catch (_) {
        updateDatasetRecordingUI('request error');
      }
    }
    async function postCmd(cmd) {
      try {
        const resp = await fetch('/cmd_vel', {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify(cmd)
        });
        if (!resp.ok) return;
        const data = await resp.json();
        applyFrames(data);
      } catch (_) {}
    }
    async function loadLatestFrames() {
      try {
        const resp = await fetch('/frames/latest');
        if (!resp.ok) return;
        const data = await resp.json();
        applyFrames(data);
      } catch (_) {}
    }
    function sendWithDrain(cmd) {
      inFlight = true;
      renderCmd(cmd);
      const sig = cmdSignature(cmd);
      lastSentSig = sig;
      lastSentAt = Date.now();
      void postCmd(cmd).finally(() => {
        inFlight = false;
        if (!pendingCmd) return;
        const next = pendingCmd;
        pendingCmd = null;
        sendWithDrain(next);
      });
    }
    function trySend(force = false) {
      const cmd = computeCmd();
      const now = Date.now();
      const sig = cmdSignature(cmd);
      const changed = sig !== lastSentSig;
      const dueKeepalive = !isZeroCmd(cmd) && (now - lastSentAt) >= keepaliveMs;
      if (!force && !changed && !dueKeepalive) return;

      if (inFlight) {
        pendingCmd = cmd;
        return;
      }

      sendWithDrain(cmd);
    }
    function keyDown(k) {
      if (keys.has(k)) return;
      keys.add(k);
      trySend(true);
    }
    function keyUp(k) {
      if (!keys.has(k)) return;
      keys.delete(k);
      trySend(true);
    }
    window.addEventListener('keydown', (e) => {
      const k = e.key.toLowerCase();
      if (['w','a','s','d','q','e',' '].includes(k)) e.preventDefault();
      if (['w','a','s','d','q','e'].includes(k)) keyDown(k);
      if (k === ' ') {
        keys.clear();
        trySend(true);
      }
    });
    window.addEventListener('keyup', (e) => {
      const k = e.key.toLowerCase();
      if (['w','a','s','d','q','e',' '].includes(k)) e.preventDefault();
      if (['w','a','s','d','q','e'].includes(k)) keyUp(k);
    });
    window.addEventListener('blur', () => {
      keys.clear();
      trySend(true);
    });
    document.querySelectorAll('button[data-k]').forEach((btn) => {
      const k = btn.getAttribute('data-k');
      btn.addEventListener('mousedown', () => keyDown(k));
      btn.addEventListener('mouseup', () => keyUp(k));
      btn.addEventListener('mouseleave', () => keyUp(k));
      btn.addEventListener('touchstart', (e) => { e.preventDefault(); keyDown(k); }, {passive:false});
      btn.addEventListener('touchend', (e) => { e.preventDefault(); keyUp(k); }, {passive:false});
    });
    document.getElementById('stop').addEventListener('click', () => {
      keys.clear();
      trySend(true);
    });
    document.getElementById('recToggle').addEventListener('click', () => {
      void toggleRecording();
    });
    document.getElementById('datasetRecToggle').addEventListener('click', () => {
      void toggleDatasetRecording();
    });
    void loadLatestFrames();
    void syncRecordingStatus();
    void syncDatasetRecordingStatus();
    setInterval(() => trySend(false), 120);
  </script>
</body>
</html>
"""

    @app.get("/health")
    def health() -> JSONResponse:
        return JSONResponse(
            {
                "ok": True,
                "node": "web_gateway",
                "recording": node.recording_status(),
                "dataset_recording": node.dataset_recording_status(),
            }
        )

    @app.get("/frames/latest")
    def frames_latest() -> JSONResponse:
        return JSONResponse({"ok": True, **node.get_frame_bundle()})

    @app.get("/snapshot/{camera}.jpg")
    def snapshot(camera: str) -> Response:
        if camera not in ("front", "rear", "depth"):
            raise HTTPException(status_code=404, detail="camera must be front|rear|depth")
        frame = node.get_frame(camera)
        if frame is None:
            raise HTTPException(status_code=503, detail=f"{camera} frame not available yet")
        return Response(content=frame.jpeg_bytes, media_type="image/jpeg")

    @app.get("/stream/{camera}")
    def stream(camera: str):
        if camera not in ("front", "rear", "depth"):
            raise HTTPException(status_code=404, detail="camera must be front|rear|depth")

        def generator():
            last_seq = -1
            while True:
                frame = node.get_frame(camera)
                if frame is not None and frame.seq != last_seq:
                    payload = (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n"
                        + f"Content-Length: {len(frame.jpeg_bytes)}\r\n\r\n".encode("utf-8")
                        + frame.jpeg_bytes
                        + b"\r\n"
                    )
                    last_seq = frame.seq
                    yield payload
                else:
                    time.sleep(0.03)

        return StreamingResponse(generator(), media_type="multipart/x-mixed-replace; boundary=frame")

    @app.post("/cmd_vel")
    def cmd_vel(cmd: CmdVelRequest) -> JSONResponse:
        prev_front_seq = node._frame_seq("front")
        node.publish_cmd_vel(cmd.linear_x, cmd.linear_y, cmd.angular_z)
        time.sleep(0.02)
        node.wait_for_front_newer_than(prev_front_seq, timeout_sec=0.35)
        return JSONResponse({"ok": True, **node.get_frame_bundle()})

    @app.post("/stop")
    def stop() -> JSONResponse:
        node.publish_cmd_vel(0.0, 0.0, 0.0)
        return JSONResponse({"ok": True, **node.get_frame_bundle()})

    @app.get("/recording/status")
    def recording_status() -> JSONResponse:
        return JSONResponse({"ok": True, **node.recording_status()})

    @app.post("/recording/start")
    def recording_start() -> JSONResponse:
        result = node.start_recording()
        return JSONResponse(result)

    @app.post("/recording/stop")
    def recording_stop() -> JSONResponse:
        result = node.stop_recording()
        return JSONResponse(result)

    @app.get("/dataset_recording/status")
    def dataset_recording_status() -> JSONResponse:
        return JSONResponse({"ok": True, **node.dataset_recording_status()})

    @app.post("/dataset_recording/start")
    def dataset_recording_start() -> JSONResponse:
        result = node.start_dataset_recording()
        return JSONResponse(result)

    @app.post("/dataset_recording/stop")
    def dataset_recording_stop() -> JSONResponse:
        result = node.stop_dataset_recording()
        return JSONResponse(result)

    return app


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WebGatewayNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    app = create_app(node)
    try:
        import uvicorn

        uvicorn.run(app, host=node.host, port=node.port, log_level="info")
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_recording()
        node.stop_dataset_recording()
        executor.shutdown()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
