#!/usr/bin/env python3
"""Offline real-worker tests and optional public-only Pico/R3 project builds."""
import argparse
import importlib.util
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]


def run(command, stage):
    result = subprocess.run([str(value) for value in command], cwd=stage,
                            text=True, capture_output=True, timeout=900)
    if result.returncode:
        print((result.stdout + result.stderr)[-6000:], flush=True)
        raise RuntimeError("Offline command failed: " + str(command[0]))
    return result.stdout + result.stderr


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--build", action="store_true", help="Also compile public-only Pico/R3 projects and check the adapter objects")
    parser.add_argument("--tsan", action="store_true", help="Run a separate ThreadSanitizer worker build")
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="pico-i2c-public-") as folder:
        stage = Path(folder)
        sources = [ROOT / path for path in ("include/pico_i2c_transport.h", "include/bq76905_monitor.h",
                   "include/ina3221_monitor.h", "src/pico_i2c_transport.cpp")]
        sources += list((ROOT / "control_tests/pico_i2c").rglob("*.h"))
        sources += list((ROOT / "control_tests/pico_i2c").rglob("*.cpp"))
        for source in sources:
            target = stage / source.relative_to(ROOT)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, target)
        (stage / "include/config.h").write_text('#error "PRIVATE_CONFIG_MUST_NOT_BE_USED"\n')
        flags = ["c++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-pedantic", "-pthread", "-g",
                 "-Icontrol_tests/pico_i2c/mock", "-Iinclude", "-fno-omit-frame-pointer"]
        source_args = ["src/pico_i2c_transport.cpp", "control_tests/pico_i2c/test_transport.cpp"]
        print("Compiling actual transport and FreeRTOS worker with native boundaries", flush=True)
        print(run(flags + ["-fsanitize=address,undefined", "-fno-sanitize-recover=all"] + source_args + ["-o", "test"], stage), end="")
        print(run([stage / "test"], stage), end="", flush=True)
        if args.tsan:
            print(run(flags + ["-fsanitize=thread"] + source_args + ["-o", "test-tsan"], stage), end="")
            print(run([stage / "test-tsan"], stage), end="", flush=True)
        if args.build:
            # Existing helper excludes the real private config by path BEFORE reads.
            spec = importlib.util.spec_from_file_location("pico_public_copy", ROOT / "control_tests/verify_pico_profile.py")
            helper = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(helper)
            helper.public_copy(stage)
            for environment in ("top_board_v2_pico", "top_board_v2_r3"):
                print(environment + " public-only build START", flush=True)
                output = run([ROOT / ".venv/bin/pio", "run", "-e", environment], stage)
                build = stage / ".pio/build" / environment / "src"
                assert (build / "pico_i2c_transport.cpp.o").is_file(), "Adapter object was not compiled"
                dependencies = list(build.glob("*.d"))
                assert dependencies
                for dependency in dependencies:
                    assert "include/config.h" not in dependency.read_text(), "Private-config dependency"
                for line in output.splitlines():
                    if "RAM:" in line or "Flash:" in line:
                        print(line, flush=True)
                print(environment + " object/project compile and sentinel dependencies PASS", flush=True)
    print("No device I/O. These worker tests do not qualify the separate hardware backend or runtime.", flush=True)


if __name__ == "__main__":
    main()
