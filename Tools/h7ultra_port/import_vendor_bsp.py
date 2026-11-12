#!/usr/bin/env python3
"""Import the *pinned* EndlessFlight vendor BSP into THIS Ryan PX4 fork.

Dry-run by default; never flashes a board or modifies git refs.
The original 41KiB vendor bootloader binary is copied only into the local
source checkout, NOT programmed to a controller by this script.
"""
from __future__ import annotations

import argparse
import hashlib
import io
import json
from pathlib import Path, PurePosixPath
import subprocess
import sys
import zipfile

RYAN_BASE = "3fef8242687cf733b590ff21c5b36779943299d1"
BSP_SHA256 = "b20f875ea87e09134e4cf9727273684ded0c45cff5c6543c022cb449bf28cc27"
VENDOR_INNER = "fc-h7-ultra-master/PX4/PX4_1.16.0/h743ultra.zip"
BOARD = Path("boards/endlessflight/h743ultra")
APPROVED_EXISTING = {
    Path("default.px4board"),
    Path("firmware.prototype"),
    Path("init/rc.board_defaults"),
    Path("init/rc.board_sensors"),
}

def commands(blob: bytes) -> list[str]:
    """Inspect startup script actions while ignoring formatting/comments."""
    return [line.strip() for line in blob.decode().splitlines()
            if line.strip() and not line.lstrip().startswith("#")]

def unpack_vendor(path: Path) -> dict[Path, bytes]:
    with zipfile.ZipFile(path) as outer:
        if VENDOR_INNER in outer.namelist():
            data = outer.read(VENDOR_INNER)
        elif "h743ultra/default.px4board" in outer.namelist():
            data = path.read_bytes()
        else:
            raise ValueError("Not the pinned EndlessFlight FC-H7-Ultra vendor ZIP")
    digest = hashlib.sha256(data).hexdigest()
    if digest != BSP_SHA256:
        raise ValueError(f"BSP SHA256 mismatch: {digest} != {BSP_SHA256}")
    files: dict[Path, bytes] = {}
    with zipfile.ZipFile(io.BytesIO(data)) as z:
        for item in z.infolist():
            if item.is_dir():
                continue
            p = PurePosixPath(item.filename)
            if len(p.parts) < 2 or p.parts[0] != "h743ultra" or ".." in p.parts:
                raise ValueError("Unexpected archive path: " + item.filename)
            relative = Path(*p.parts[1:])
            if relative in files or item.file_size > 2_000_000:
                raise ValueError("Duplicate or oversized BSP entry: " + item.filename)
            files[relative] = z.read(item)
    if len(files) != 24:
        raise ValueError(f"Expected 24 vendor BSP files; got {len(files)}")
    identity = json.loads(files[Path("firmware.prototype")])
    if identity.get("board_id") != 1179 or identity.get("description") != "Firmware for the EndlessFlight_H7ultra board":
        raise ValueError("Vendor firmware identity mismatch")
    config = files[Path("default.px4board")].decode()
    if 'CONFIG_BOARD_SERIAL_TEL1="/dev/ttyS0"' not in config or 'CONFIG_DRIVERS_MAGNETOMETER_QMC5883P=y' not in config:
        raise ValueError("Vendor UART1 / magnetometer config changed")
    return files

def verify_existing(path: Path, local: bytes, vendor: bytes) -> None:
    if local == vendor:
        return
    if path not in APPROVED_EXISTING:
        raise ValueError("Refusing to overwrite nonmatching existing vendor BSP: " + str(path))
    if path == Path("firmware.prototype"):
        if json.loads(local) != json.loads(vendor):
            raise ValueError("Existing firmware prototype differs semantically")
    elif path in (Path("init/rc.board_defaults"), Path("init/rc.board_sensors")):
        if commands(local) != commands(vendor):
            raise ValueError("Existing startup commands differ: " + str(path))
    else:
        # Only known intentional change: unsupported CONFIG_COMMON_INS symbol
        # removed from Ryan 1.16.2, other *effective* configs preserved.
        def active_config(blob: bytes) -> set[str]:
            return {line.strip() for line in blob.decode().splitlines()
                    if line.strip().startswith("CONFIG_") and
                    not line.strip().startswith("CONFIG_COMMON_INS=")}
        if active_config(local) != active_config(vendor):
            raise ValueError("Existing default.px4board changes active Kconfig symbols")
    print("PRESERVE existing reviewed copy:", path)

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--vendor-zip", type=Path, required=True,
                    help="User-supplied fc-h7-ultra-master.zip or nested h743ultra.zip")
    ap.add_argument("--px4-dir", type=Path, default=Path("."),
                    help="Ryan PX4 fork checkout (default: cwd)")
    ap.add_argument("--apply", action="store_true",
                    help="Install missing source files after all validation; default: dry run")
    a = ap.parse_args()
    root = a.px4_dir.expanduser().resolve()
    vendor = a.vendor_zip.expanduser().resolve()
    try:
        if not (root / ".git").exists():
            raise ValueError("PX4 source checkout .git missing")
        if subprocess.run(["git", "-C", str(root), "merge-base", "--is-ancestor",
                           RYAN_BASE, "HEAD"], check=False).returncode:
            raise ValueError("Expected descendant of Ryan PX4 " + RYAN_BASE)
        data = unpack_vendor(vendor)
        changes = {}
        for rel, payload in sorted(data.items()):
            dst = root / BOARD / rel
            if dst.exists():
                verify_existing(rel, dst.read_bytes(), payload)
            else:
                changes[dst] = payload
                print("ADD", dst.relative_to(root), f"({len(payload)} B)")
    except (OSError, zipfile.BadZipFile, ValueError) as exc:
        ap.error(str(exc))
    print(f"Verified {len(data)} vendor files, {len(changes)} new files.")
    if not a.apply:
        print("DRY RUN only. Use --apply to write the missing files locally.")
        return 0
    for dest, payload in changes.items():
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_bytes(payload)
    print("BSP source staged locally. NO commit/push/flash/bootloader programming.")
    return 0

if __name__ == "__main__":
    sys.exit(main())
