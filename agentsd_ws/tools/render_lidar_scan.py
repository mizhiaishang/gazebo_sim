#!/usr/bin/env python3
"""
Render a lidar scan.bin produced by DataRecorder into an image.

Expected binary layout:
float32 Nx3 = [angle(rad), range(m), intensity]
"""

from __future__ import annotations

import argparse
import math
import struct
from pathlib import Path


def load_scan_points(scan_path: Path):
    data = scan_path.read_bytes()
    if len(data) % 12 != 0:
        raise ValueError(f"Unexpected file size {len(data)} bytes (not divisible by 12).")
    n = len(data) // 12
    points = []
    offset = 0
    for _ in range(n):
        ang, rng, intensity = struct.unpack_from("<fff", data, offset)
        offset += 12
        if math.isfinite(ang) and math.isfinite(rng) and rng > 0.0:
            x = rng * math.cos(ang)
            y = rng * math.sin(ang)
            points.append((x, y, intensity))
    return points


def draw_ppm(points, out_path: Path, size: int = 1000):
    w = h = size
    img = bytearray([255] * (w * h * 3))

    if not points:
        _save_ppm(img, w, h, out_path)
        return

    max_abs = max(max(abs(x), abs(y)) for x, y, _ in points)
    max_abs = max(max_abs, 1.0)
    scale = 0.48

    # draw axes
    cx, cy = w // 2, h // 2
    for x in range(w):
        idx = (cy * w + x) * 3
        img[idx : idx + 3] = b"\xE0\xE0\xE0"
    for y in range(h):
        idx = (y * w + cx) * 3
        img[idx : idx + 3] = b"\xE0\xE0\xE0"

    # draw points
    for x, y, intensity in points:
        px = int((x / max_abs * scale + 0.5) * w)
        py = int(((-y) / max_abs * scale + 0.5) * h)
        if px < 0 or px >= w or py < 0 or py >= h:
            continue
        # simple color map by intensity
        if math.isfinite(intensity):
            t = max(0.0, min(1.0, intensity / 2000.0))
        else:
            t = 0.0
        r = int(255 * t)
        g = int(80 + 120 * (1.0 - t))
        b = int(255 * (1.0 - t))
        idx = (py * w + px) * 3
        img[idx : idx + 3] = bytes((r, g, b))

    _save_ppm(img, w, h, out_path)


def _save_ppm(raw_rgb: bytearray, w: int, h: int, out_path: Path):
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("wb") as f:
        f.write(f"P6\n{w} {h}\n255\n".encode("ascii"))
        f.write(raw_rgb)


def try_draw_png(points, out_path: Path):
    try:
        import matplotlib.pyplot as plt
    except Exception:
        return False

    if points:
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        cs = [p[2] if math.isfinite(p[2]) else 0.0 for p in points]
    else:
        xs, ys, cs = [], [], []

    fig = plt.figure(figsize=(8, 8), dpi=160)
    ax = fig.add_subplot(111)
    ax.set_aspect("equal", adjustable="box")
    ax.scatter(xs, ys, c=cs, s=2, cmap="turbo")
    ax.grid(True, alpha=0.25)
    ax.axhline(0.0, color="gray", linewidth=0.8)
    ax.axvline(0.0, color="gray", linewidth=0.8)
    ax.set_title("Lidar point cloud (top-down)")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, bbox_inches="tight")
    plt.close(fig)
    return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("scan_bin", type=Path, help="Path to lidar/scan.bin")
    parser.add_argument(
        "--out",
        type=Path,
        default=Path("scan_pointcloud.png"),
        help="Output image path (PNG preferred)",
    )
    args = parser.parse_args()

    points = load_scan_points(args.scan_bin)
    png_ok = args.out.suffix.lower() == ".png" and try_draw_png(points, args.out)

    if png_ok:
        print(f"Saved PNG: {args.out}")
    else:
        ppm_path = args.out.with_suffix(".ppm")
        draw_ppm(points, ppm_path)
        print(f"matplotlib unavailable or PNG not requested; saved PPM: {ppm_path}")
    print(f"Valid points: {len(points)}")


if __name__ == "__main__":
    main()

