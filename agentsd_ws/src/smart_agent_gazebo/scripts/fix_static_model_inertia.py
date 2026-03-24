#!/usr/bin/env python3
"""
Patch static model SDF files so that every <inertial> block with <mass>
also contains a valid <inertia> matrix.

This is required by Gazebo Sim (Ignition/GZ). Some legacy AWS model files
ship with only <mass>, which Gazebo Sim rejects as "invalid inertia".
"""

from __future__ import annotations

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1] / "models"


INERTIA_BLOCK = """{indent}<inertia>
{indent}  <ixx>1.0</ixx>
{indent}  <iyy>1.0</iyy>
{indent}  <izz>1.0</izz>
{indent}  <ixy>0.0</ixy>
{indent}  <ixz>0.0</ixz>
{indent}  <iyz>0.0</iyz>
{indent}</inertia>"""


STATIC_RE = re.compile(r"<static>\s*(?:1|true)\s*</static>", re.IGNORECASE)
INERTIAL_RE = re.compile(r"<inertial>[\s\S]*?</inertial>", re.IGNORECASE)
MASS_LINE_RE = re.compile(r"^([ \t]*)<mass>\s*[^<]+</mass>\s*$", re.IGNORECASE | re.MULTILINE)
PRINCIPAL_RE = {
    "ixx": re.compile(r"<ixx>\s*([^<]+)\s*</ixx>", re.IGNORECASE),
    "iyy": re.compile(r"<iyy>\s*([^<]+)\s*</iyy>", re.IGNORECASE),
    "izz": re.compile(r"<izz>\s*([^<]+)\s*</izz>", re.IGNORECASE),
}


def patch_inertial_block(block: str) -> tuple[str, bool]:
    if re.search(r"<inertia>\s*<", block, re.IGNORECASE):
        changed = False
        patched = block
        for tag, pattern in PRINCIPAL_RE.items():
            m = pattern.search(patched)
            if not m:
                continue
            try:
                value = float(m.group(1).strip())
            except ValueError:
                continue
            # Gazebo rejects zero / negative principal moments.
            if value <= 0.0:
                patched = pattern.sub(f"<{tag}>1.0</{tag}>", patched, count=1)
                changed = True
        return patched, changed

    mass_match = MASS_LINE_RE.search(block)
    if not mass_match:
        return block, False

    indent = mass_match.group(1)
    inertia_xml = INERTIA_BLOCK.format(indent=indent)
    insertion = mass_match.group(0) + "\n" + inertia_xml
    patched = block.replace(mass_match.group(0), insertion, 1)
    return patched, True


def patch_file(path: Path) -> bool:
    text = path.read_text(encoding="utf-8", errors="ignore")

    # Only patch static models from the legacy assets.
    if not STATIC_RE.search(text):
        return False
    if "<inertial>" not in text:
        return False

    changed = False

    def repl(match: re.Match[str]) -> str:
        nonlocal changed
        block = match.group(0)
        patched, was_changed = patch_inertial_block(block)
        if was_changed:
            changed = True
        return patched

    patched_text = INERTIAL_RE.sub(repl, text)
    if not changed:
        return False

    path.write_text(patched_text, encoding="utf-8")
    return True


def main() -> None:
    updated = []
    for sdf in ROOT.rglob("model.sdf"):
        if patch_file(sdf):
            updated.append(sdf)

    print(f"Patched {len(updated)} model files.")
    for p in updated:
        print(p)


if __name__ == "__main__":
    main()
