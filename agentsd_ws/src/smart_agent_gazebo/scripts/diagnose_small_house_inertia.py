#!/usr/bin/env python3
from __future__ import annotations

import re
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
WORLD = ROOT / "worlds" / "small_house.world"
MODELS = ROOT / "models"


URI_RE = re.compile(r"<uri>\s*model://([^<\s]+)\s*</uri>")


def strip_ns(tag: str) -> str:
    return tag.split("}", 1)[-1]


def inertia_invalid(inertial: ET.Element) -> str | None:
    mass_el = inertial.find("mass")
    inertia_el = inertial.find("inertia")
    if mass_el is None:
        return "missing mass"
    if inertia_el is None:
        return "missing inertia matrix"

    def f(name: str) -> float:
        el = inertia_el.find(name)
        if el is None or el.text is None:
            raise ValueError(f"missing {name}")
        return float(el.text.strip())

    try:
        m = float((mass_el.text or "").strip())
        ixx = f("ixx")
        iyy = f("iyy")
        izz = f("izz")
        ixy = f("ixy")
        ixz = f("ixz")
        iyz = f("iyz")
    except Exception as e:
        return f"parse error: {e}"

    if m <= 0:
        return f"mass <= 0 ({m})"
    if ixx <= 0 or iyy <= 0 or izz <= 0:
        return f"principal inertia non-positive ({ixx}, {iyy}, {izz})"

    # Positive-definite checks for symmetric inertia matrix.
    minor2 = ixx * iyy - ixy * ixy
    det3 = (
        ixx * (iyy * izz - iyz * iyz)
        - ixy * (ixy * izz - iyz * ixz)
        + ixz * (ixy * iyz - iyy * ixz)
    )
    if minor2 <= 0:
        return f"not positive-definite (minor2={minor2})"
    if det3 <= 0:
        return f"not positive-definite (det={det3})"
    return None


def main() -> None:
    world_text = WORLD.read_text(encoding="utf-8", errors="ignore")
    model_names = sorted(set(URI_RE.findall(world_text)))
    print(f"World models referenced: {len(model_names)}")

    bad = []
    missing_sdf = []

    for model in model_names:
        sdf_path = MODELS / model / "model.sdf"
        if not sdf_path.exists():
            missing_sdf.append(str(sdf_path))
            continue

        try:
            tree = ET.parse(sdf_path)
        except Exception as e:
            bad.append((model, "xml parse failed", str(e)))
            continue

        root = tree.getroot()
        for link in root.iter():
            if strip_ns(link.tag) != "link":
                continue
            link_name = link.get("name", "<unnamed>")
            inertial = None
            for child in link:
                if strip_ns(child.tag) == "inertial":
                    inertial = child
                    break
            if inertial is None:
                continue
            reason = inertia_invalid(inertial)
            if reason:
                bad.append((model, link_name, reason))

    if missing_sdf:
        print("\nMissing model.sdf:")
        for p in missing_sdf:
            print(f"  {p}")

    if not bad:
        print("\nNo invalid inertia blocks found.")
        return

    print(f"\nInvalid inertia entries: {len(bad)}")
    for model, link_name, reason in bad:
        print(f"  {model} :: link={link_name} :: {reason}")


if __name__ == "__main__":
    main()
