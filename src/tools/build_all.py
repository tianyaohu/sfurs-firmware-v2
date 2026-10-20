#!/usr/bin/env python3
"""Build every firmware artifact into build/ (git-ignored). No hardware access.

  build/esc/<W>/motor_<W>_drive_offset.{elf,bin}   ESC app at 0x08004000
  build/esc/<W>/fwu_payload.c                      ESC app as Nucleo payload
  build/updater/<W>/sfurs_firmware_v2_CM7.elf      Nucleo ESC-updater image
  build/drive/sfurs_firmware_v2_CM7.elf            Nucleo drive image

Usage: scripts/build_all.py [--wheels NE SE SW NW] [--shift NE=-60 ...] [--drive-only]
--shift bakes a Hall phase shift measured with `esc_tool.py align`.
"""
import argparse
import hashlib
import os
import pathlib
import shutil
import subprocess
import sys
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[1]
OUT = ROOT / "build"
WHEELS = ("NE", "SE", "SW", "NW")
FALLBACK_TOOLCHAIN = "/tmp/opencode/cm7-toolchain/root/usr/bin"


def run(*cmd):
    subprocess.run([str(c) for c in cmd], check=True, cwd=ROOT,
                   stdout=subprocess.DEVNULL if "--quiet" in sys.argv else None)


def cmake(build_dir, *defines):
    run("cmake", "-S", ROOT / "CM7", "-B", build_dir, "-G", "Ninja",
        f"-DCMAKE_TOOLCHAIN_FILE={ROOT / 'gcc-arm-none-eabi.cmake'}",
        "-DCMAKE_BUILD_TYPE=Debug", *defines)
    run("cmake", "--build", build_dir)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--wheels", nargs="+", choices=WHEELS, default=list(WHEELS))
    parser.add_argument("--shift", nargs="*", default=[], metavar="W=DEG")
    parser.add_argument("--drive-only", action="store_true", help="only the Nucleo drive image")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()
    shifts = dict(item.split("=") for item in args.shift)
    if not shutil.which("arm-none-eabi-gcc") and os.path.isdir(FALLBACK_TOOLCHAIN):
        os.environ["PATH"] = FALLBACK_TOOLCHAIN + os.pathsep + os.environ["PATH"]
    artifacts = []
    if not args.drive_only:
        for w in args.wheels:
            esc = OUT / "esc" / w
            extra = ["--hall-shift", shifts[w]] if w in shifts else []
            run(sys.executable, ROOT / "HW/B-G431B-ESC1/motor_config/build_cli.py",
                "--motor", w, "--app-offset", "--output", esc, *extra)
            binary = esc / f"motor_{w}_drive_offset.bin"
            run(sys.executable, ROOT / "scripts/fwu_image.py", "--bin", binary,
                "--output", esc / "fwu_payload.c")
            cmake(OUT / "updater" / w, f"-DFWU_IMAGE_FILE={esc / 'fwu_payload.c'}")
            artifacts += [binary, OUT / "updater" / w / "sfurs_firmware_v2_CM7.elf"]
    cmake(OUT / "drive", "-UFWU_IMAGE_FILE", "-DFWU_IMAGE_FILE=")
    artifacts.append(OUT / "drive/sfurs_firmware_v2_CM7.elf")
    # Hash what gets flashed: an ELF's loadable bytes (its debug sections are
    # not reproducible between builds), a .bin as is.
    for path in artifacts:
        if path.suffix == ".elf":
            with tempfile.TemporaryDirectory() as tmp:
                image = pathlib.Path(tmp) / "image.bin"
                run("arm-none-eabi-objcopy", "-O", "binary", path, image)
                data = image.read_bytes()
        else:
            data = path.read_bytes()
        print(hashlib.sha256(data).hexdigest()[:16], path.relative_to(ROOT))


if __name__ == "__main__":
    main()
