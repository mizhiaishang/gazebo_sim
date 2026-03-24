#!/usr/bin/env python3
"""
Download missing non-human AWS hospital world dependencies from Fuel.

Source world:
  _deps/aws-robomaker-hospital-world/worlds/hospital.world

Destination:
  agent_ws/src/smart_agent_gazebo/models
"""

from pathlib import Path
import io
import json
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
import zipfile


FUEL_BASE = "https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models"

HUMAN_MODELS = {
    "ElderLadyPatient",
    "ElderMalePatient",
    "FemaleVisitor",
    "FemaleVisitorSit",
    "MalePatientBed",
    "MaleVisitorOnPhone",
    "MaleVisitorSit",
    "PatientFSit",
    "PatientWheelChair",
    "Scrubs",
    "TrolleyBedPatient",
    "VisitorKidSit",
}

BUILTINS = {"ground_plane", "sun", "smart_agent"}


def parse_model_uris(world_path: Path) -> set[str]:
    uris: set[str] = set()
    prefix = "<uri>model://"
    suffix = "</uri>"
    for line in world_path.read_text(encoding="utf-8").splitlines():
        text = line.strip()
        if text.startswith(prefix) and text.endswith(suffix):
            uris.add(text[len(prefix) : -len(suffix)].strip())
    return uris


def fuel_model_version(model_name: str) -> str:
    meta_url = f"{FUEL_BASE}/{urllib.parse.quote(model_name)}"
    with urllib.request.urlopen(meta_url, timeout=30) as response:
        payload = json.load(response)
    return str(payload.get("version", 1))


def download_model_zip(model_name: str, version: str) -> bytes:
    zip_url = (
        f"{FUEL_BASE}/{urllib.parse.quote(model_name)}"
        f"/{urllib.parse.quote(version)}/{urllib.parse.quote(model_name)}.zip"
    )
    with urllib.request.urlopen(zip_url, timeout=120) as response:
        return response.read()


def main() -> int:
    root = Path(__file__).resolve().parents[4]
    world_path = root / "_deps" / "aws-robomaker-hospital-world" / "worlds" / "hospital.world"
    models_dir = root / "agent_ws" / "src" / "smart_agent_gazebo" / "models"

    if not world_path.exists():
        print(f"[ERROR] world not found: {world_path}")
        return 2
    if not models_dir.exists():
        print(f"[ERROR] models dir not found: {models_dir}")
        return 2

    all_model_uris = parse_model_uris(world_path)
    target_models = sorted(
        name
        for name in all_model_uris
        if name not in HUMAN_MODELS and name not in BUILTINS
    )

    print(f"[INFO] total model:// uris in hospital.world = {len(all_model_uris)}")
    print(f"[INFO] target non-human models = {len(target_models)}")

    existing = {p.name for p in models_dir.iterdir() if p.is_dir()}
    to_download = [m for m in target_models if m not in existing]

    print(f"[INFO] already available locally = {len(target_models) - len(to_download)}")
    print(f"[INFO] need to download = {len(to_download)}")

    ok: list[str] = []
    failed: list[str] = []

    for index, model_name in enumerate(to_download, start=1):
        print(f"[{index}/{len(to_download)}] downloading {model_name} ...")
        target_dir = models_dir / model_name
        target_dir.mkdir(parents=True, exist_ok=True)
        try:
            version = fuel_model_version(model_name)
            zip_bytes = download_model_zip(model_name, version)
            with zipfile.ZipFile(io.BytesIO(zip_bytes)) as archive:
                archive.extractall(target_dir)
            ok.append(model_name)
            print(f"  [OK] {model_name} (version={version}, bytes={len(zip_bytes)})")
            time.sleep(0.5)
        except (urllib.error.URLError, urllib.error.HTTPError, TimeoutError, OSError, zipfile.BadZipFile) as exc:
            failed.append(model_name)
            print(f"  [FAIL] {model_name}: {exc}")

    print("")
    print(f"[SUMMARY] downloaded={len(ok)} failed={len(failed)}")
    if failed:
        print("[SUMMARY] failed models:")
        for name in failed:
            print(f"  - {name}")

    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
