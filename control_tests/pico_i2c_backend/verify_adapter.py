#!/usr/bin/env python3
"""ASan/UBSan tests of the real C++ backend through a fake native C boundary.

No C driver, shared worker, SDK build/link, hardware or private config access.
Only the explicit public source allowlist below is copied into a temporary tree.
"""
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
PUBLIC_SOURCES = (
    "include/pico_i2c_esp_backend.h",
    "include/pico_i2c_native.h",
    "include/pico_i2c_transport.h",
    "include/bq76905_monitor.h",
    "include/ina3221_monitor.h",
    "src/pico_i2c_esp_backend.cpp",
    "control_tests/pico_i2c_backend/test_adapter.cpp",
    "control_tests/pico_i2c_backend/mock/freertos/FreeRTOS.h",
    "control_tests/pico_i2c_backend/mock/freertos/task.h",
)


def run(command, stage):
    result = subprocess.run(command, cwd=stage, text=True, capture_output=True, timeout=120)
    print(result.stdout, end="", flush=True)
    if result.stderr:
        print(result.stderr, end="", flush=True)
    if result.returncode:
        raise SystemExit(result.returncode)


def main():
    with tempfile.TemporaryDirectory(prefix="pico-backend-adapter-public-") as folder:
        stage = Path(folder)
        for relative in PUBLIC_SOURCES:
            target = stage / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(ROOT / relative, target)
        # New poison file only; the real include/config.h is never accessed.
        (stage / "include/config.h").write_text('#error "PRIVATE_CONFIG_MUST_NOT_BE_USED"\n')
        flags = ["c++", "-std=c++17", "-O1", "-g", "-Wall", "-Wextra", "-Werror", "-pedantic",
                 "-fsanitize=address,undefined", "-fno-sanitize-recover=all", "-fno-omit-frame-pointer",
                 "-DWBB_I2C_BACKEND_NATIVE_TEST", "-Icontrol_tests/pico_i2c_backend/mock", "-Iinclude"]
        print("Compiling real EspIdfBackend with fake native C API (ASan/UBSan)", flush=True)
        objects = []
        for index, source in enumerate(("src/pico_i2c_esp_backend.cpp",
                                        "control_tests/pico_i2c_backend/test_adapter.cpp")):
            obj, dep = f"adapter-{index}.o", f"adapter-{index}.d"
            run(flags + ["-MMD", "-MF", dep, "-c", source, "-o", obj], stage)
            assert "include/config.h" not in (stage / dep).read_text(), "Private-config dependency"
            objects.append(obj)
        run(flags + objects + ["-o", "test-adapter"], stage)
        run([str(stage / "test-adapter")], stage)
        print("PASS public allowlist and poison-config dependency checks; no device or SDK C-driver test", flush=True)


if __name__ == "__main__":
    main()
