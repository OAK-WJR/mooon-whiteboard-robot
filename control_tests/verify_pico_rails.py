#!/usr/bin/env python3
"""Actual runtime/monitor telemetry with mock hardware; never opens a device."""
import argparse
import configparser
from pathlib import Path
import re
import shlex
import tempfile
from verify_pico_profile import ROOT, public_copy, run

MODES = ('ready', 'generation', 'expired', 'wrap', 'absent', 'start_failure', 'setup_timeout',
         'nack', 'short', 'pending', 'por', 'invalid_voltage', 'gpio_startup_fault',
         'watchdog_startup_fault', 'gpio_runtime_fault', 'watchdog_fault', 'reboot', 'cleanup_fault')

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build',action='store_true');args=parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='wbb-rail-runtime-') as folder:
        stage=Path(folder);public_copy(stage)
        config=configparser.ConfigParser(interpolation=None);config.read(stage/'platformio.ini')
        flags=shlex.split(config['env:top_board_v2_pico']['build_flags'])
        json_include=next((ROOT/'.pio/libdeps').glob('*/ArduinoJson/src'))
        command=['c++','-std=c++17','-fsanitize=address,undefined','-fno-sanitize-recover=all',
                 '-Icontrol_tests/pico_mock','-Icontrol_tests/mock','-Iinclude','-Isrc','-I'+str(json_include),
                 '-DCONFIG_IDF_TARGET_ESP32S3=1',*flags,'-MMD','-MF','rails.d',
                 'control_tests/test_pico_rails.cpp','-o','rails']
        run('rail_runtime_compile',command,stage)
        assert 'include/config.h' not in (stage/'rails.d').read_text()
        for mode in MODES:run(mode,[stage/'rails',mode],stage)
        if args.build:
            # No force-link option: the real application must retain this path.
            for env in ('top_board_v2_pico','top_board_v2_r3'):
                output=run(env,[ROOT/'.venv/bin/pio','run','-e',env],stage)
                print('\n'.join(l for l in output.splitlines() if 'RAM:' in l or 'Flash:' in l),flush=True)
                for dependency in (stage/'.pio/build'/env/'src').glob('*.d'):
                    assert 'include/config.h' not in dependency.read_text()
            nm=next((ROOT/'.platformio/packages').glob('toolchain-xtensa-esp-elf/bin/xtensa-esp32s3-elf-nm'))
            symbols=run('runtime_link',[nm,'--defined-only',stage/'.pio/build/top_board_v2_pico/firmware.elf'],stage)
            for name in ('wbb_i2c_open','wbb_i2c_transfer','wbb_i2c_close','wbb_idf554_i2c_master_transmit_receive'):
                assert re.search(r'\b'+name+r'$',symbols,re.M),'Runtime did not retain '+name
            for name in ('i2c_new_master_bus','i2c_master_transmit_receive','i2c_master_cmd_begin'):
                assert not re.search(r'\b'+name+r'$',symbols,re.M),'Unexpected stock bus path '+name
    print(f'{len(MODES)} runtime rail cases PASS. Offline only; no silicon/radio/power qualification.',flush=True)

if __name__=='__main__':main()
