#!/usr/bin/env python3
"""Reproducible local checks. Does not upload firmware or operate any motors."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from datetime import datetime, timezone

ROOT=Path(__file__).resolve().parents[1]
V=ROOT/'hardware/top_board_v2/validation/reverification'
V.mkdir(parents=True,exist_ok=True)
rows=[]

def source_hashes():
    paths=set()
    for folder in ('src','include','control_tests','usb_diagnostics/src','usb_diagnostics/include','usb_diagnostics/tests'):
        paths.update(p for p in (ROOT/folder).rglob('*') if p.suffix in ('.cpp','.h','.py'))
    paths.update(ROOT/p for p in ('platformio.ini','usb_diagnostics/platformio.ini','test/test_all.cpp',
        '.scripts/extra_compiledb.py','scripts/pcb_control.py','scripts/pcb_diagnostics.py',
        'scripts/robot_control.py','scripts/requirements-control.txt'))
    # Bind the native JSON parser and SDK assumptions to the local versions read.
    paths.update(ROOT/p for p in ('.pio/libdeps/top_board_v2/ArduinoJson/library.json',
        '.pio/libdeps/top_board_v2/ArduinoJson/src/ArduinoJson/Json/JsonDeserializer.hpp',
        '.platformio/packages/framework-arduinoespressif32/package.json',
        '.platformio/packages/framework-arduinoespressif32/libraries/BLE/src/BLECharacteristic.cpp',
        '.platformio/packages/framework-arduinoespressif32/libraries/BLE/src/BLEServer.cpp',
        '.platformio/packages/framework-arduinoespressif32/cores/esp32/esp32-hal-misc.c',
        '.platformio/packages/framework-arduinoespressif32-libs/esp32c6/include/freertos/FreeRTOS-Kernel/include/freertos/queue.h',
        '.platformio/packages/framework-arduinoespressif32-libs/esp32c6/include/freertos/FreeRTOS-Kernel/include/freertos/task.h',
        '.platformio/packages/framework-arduinoespressif32-libs/esp32c6/include/freertos/FreeRTOS-Kernel/portable/riscv/include/freertos/portmacro.h',
        '.platformio/packages/framework-arduinoespressif32-libs/esp32/include/freertos/FreeRTOS-Kernel/portable/xtensa/include/freertos/portmacro.h'))
    return {str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(paths) if p.is_file()}

before_hashes=source_hashes()


def run(name, args, cwd=ROOT):
    with (V/(name+'.log')).open('w') as log:
        try:
            code=subprocess.run([str(a) for a in args],cwd=cwd,stdout=log,stderr=subprocess.STDOUT,timeout=900).returncode
        except subprocess.TimeoutExpired:
            log.write('\nVerification step timed out after 900 seconds.\n');code=124
        except OSError as error:
            log.write(f'\nCould not execute verification step: {error}\n');code=127
    rows.append(dict(name=name,exit_code=code,command=[str(a) for a in args],cwd=str(cwd)))
    print(name, 'PASS' if code==0 else 'FAIL',flush=True)
    return code==0


with tempfile.TemporaryDirectory(prefix='wbb-verification-') as temp:
    compile_flags=['c++','-std=c++17','-fsanitize=address,undefined','-fno-sanitize-recover=all']
    runtime_flags=['-Icontrol_tests/mock','-Iinclude','-I.pio/libdeps/top_board_v2/ArduinoJson/src']
    board_flags=['-DCONFIG_IDF_TARGET_ESP32C6=1','-DWBB_TOP_BOARD_V2=1']
    for name,source,includes in [
        ('native_simulation_direct','test/test_all.cpp',['-DNATIVE_TEST','-Iinclude','-Isrc']),
        ('control_session','control_tests/test_session.cpp',['-Iinclude']),
        ('main_motor_faults','control_tests/test_motor_driver.cpp',runtime_flags+board_flags),
        ('main_robot_driver','control_tests/test_robot_driver.cpp',runtime_flags+board_flags),
        ('main_odometry','control_tests/test_odometry.cpp',runtime_flags+board_flags),
        ('main_stall_timing','control_tests/test_stall_timing.cpp',runtime_flags+board_flags),
        ('main_runtime_timing','control_tests/test_runtime_timing.cpp',runtime_flags),
        ('main_runtime_commands','control_tests/test_runtime_commands.cpp',runtime_flags),
        ('main_transport','control_tests/test_transport.cpp',runtime_flags),
        ('main_transport_reboot','control_tests/test_transport_reboot.cpp',runtime_flags),
        ('diagnostic_safety','usb_diagnostics/tests/test_safety.cpp',['-Iusb_diagnostics/include']),
        ('firmware_adapter','usb_diagnostics/tests/test_firmware.cpp',['-Iusb_diagnostics/tests/mock','-Iusb_diagnostics/include']),
        ('firmware_top_v2','usb_diagnostics/tests/test_firmware_top_v2.cpp',['-Iusb_diagnostics/tests/mock','-Iusb_diagnostics/include'])]:
        binary=Path(temp)/name
        if run(name+'_compile',compile_flags+includes+[source,'-o',binary]):
            run(name,[binary])
            if name=='main_odometry':
                for mode in ('overflow','directions','invalid-dt'):
                    run(name+'_'+mode,[binary,mode])
            if name=='main_stall_timing':
                for mode in ('wrap','zero-deadline'):
                    run(name+'_'+mode,[binary,mode])
            if name=='main_transport_reboot':
                for mode in ('full','flood','tx-failure','wrap','owner-failure'):
                    run(name+'_'+mode,[binary,mode])
    run('control_host_tests',[sys.executable,'-m','unittest','discover','-s','control_tests','-p','test_*.py','-v'])
    run('diagnostic_host_tests',[sys.executable,'-m','unittest','discover','-s','usb_diagnostics/tests','-p','test_*.py','-v'])
    run('main_firmware_build',[ROOT/'.venv/bin/pio','run','-e','top_board_v2','-e','c6','-e','esp32','-e','motor_test'])
    run('diagnostic_firmware_build',[ROOT/'.venv/bin/pio','run','-e','adapter','-e','top_board_v2'],ROOT/'usb_diagnostics')

hashes=source_hashes()
unchanged=before_hashes==hashes
report=dict(time_utc=datetime.now(timezone.utc).isoformat(),all_passed=unchanged and all(r['exit_code']==0 for r in rows),
            checks=rows,source_hashes=hashes,hardware_attached=False,firmware_flashed=False,
            sources_unchanged_during_tests=unchanged,
            native_platform=sys.platform,native_command_hmac='macOS CommonCrypto SHA256',
            native_scope='Actual command/loop, controller, driver and transport source. GPIO, interrupts, clocks, NVS, RTOS queues, Wi-Fi and BLE APIs are test doubles; SDK callback scheduling and RF are not emulated.',
            radio_tested=False,physical_motor_tested=False)
(V/'software_verification.json').write_text(json.dumps(report,indent=2)+'\n')
raise SystemExit(0 if report['all_passed'] else 1)
