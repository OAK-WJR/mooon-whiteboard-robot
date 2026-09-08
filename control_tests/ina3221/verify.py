#!/usr/bin/env python3
"""Offline build of the real standalone driver; explicit public files only."""
from pathlib import Path
import argparse
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]


def run(args: list[str]) -> None:
    subprocess.run(args, check=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cxx", default="c++")
    parser.add_argument("--target-cxx", help="Optional installed ESP32-S3 C++ compiler (compile only)")
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="ina3221-public-") as directory:
        stage = Path(directory)
        # Never traverse/copy the repository include directory or private config.
        for source in ("include/ina3221_monitor.h", "src/ina3221_monitor.cpp",
                       "control_tests/ina3221/test_monitor.cpp"):
            shutil.copyfile(ROOT / source, stage / Path(source).name)
        (stage / "config.h").write_text('#error "private config must never be included"\n')
        flags = ["-std=c++17", "-Wall", "-Wextra", "-Werror", "-pedantic", "-I", str(stage)]
        executable = str(stage / "test_monitor")
        run([args.cxx, *flags, "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
             str(stage / "ina3221_monitor.cpp"), str(stage / "test_monitor.cpp"), "-o", executable])
        run([executable])
        if args.target_cxx:
            run([args.target_cxx, *flags, "-fno-exceptions", "-fno-rtti", "-c",
                 str(stage / "ina3221_monitor.cpp"), "-o", str(stage / "ina3221_monitor.o")])
            print("PASS ESP32-S3 standalone driver object compile (no runtime/hardware claim)")


if __name__ == "__main__":
    main()
