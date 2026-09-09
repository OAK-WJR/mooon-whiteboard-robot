#!/usr/bin/env python3
"""Cell/rail/runtime integration tests with SDK and device boundaries mocked."""
import argparse
import configparser
from pathlib import Path
import re
import shlex
import tempfile
from verify_pico_profile import ROOT, public_copy, run

MODES=('normal','wrap','long_frames','absent','wake_abort','wake_assert_fault','wake_release_fault',
       'wake_stall','stale','por','configuration_drift','bus_error','lower_cell_invalid','reboot_in_transfer')

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--build',action='store_true');args=parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='wbb-cell-runtime-') as folder:
        stage=Path(folder);public_copy(stage)
        config=configparser.ConfigParser(interpolation=None);config.read(stage/'platformio.ini')
        flags=shlex.split(config['env:top_board_v2_pico']['build_flags'])
        inc=next((ROOT/'.pio/libdeps').glob('*/ArduinoJson/src'))
        cmd=['c++','-std=c++17','-fsanitize=address,undefined','-fno-sanitize-recover=all',
             '-Icontrol_tests/pico_mock','-Icontrol_tests/mock','-Iinclude','-Isrc','-I'+str(inc),
             '-DCONFIG_IDF_TARGET_ESP32S3=1',*flags,'-MMD','-MF','cells.d',
             'control_tests/test_pico_cells.cpp','-o','cells']
        run('cell_runtime_compile',cmd,stage)
        assert 'include/config.h' not in (stage/'cells.d').read_text()
        for mode in MODES:run(mode,[stage/'cells',mode],stage)
        if args.build:
            for env in ('top_board_v2_pico','top_board_v2_r3'):
                output=run(env,[ROOT/'.venv/bin/pio','run','-e',env],stage)
                print('\n'.join(l for l in output.splitlines() if 'RAM:' in l or 'Flash:' in l),flush=True)
                for dependency in (stage/'.pio/build'/env/'src').glob('*.d'):
                    assert 'include/config.h' not in dependency.read_text()
            nm=next((ROOT/'.platformio/packages').glob('toolchain-xtensa-esp-elf/bin/xtensa-esp32s3-elf-nm'))
            symbols=run('cell_runtime_link',[nm,'-C','--defined-only',stage/'.pio/build/top_board_v2_pico/firmware.elf'],stage)
            for name in ('wbb_i2c_transfer','wbb_idf554_i2c_master_transmit_receive','bq76905::Monitor::tick()',
                         'ina3221::Monitor::tick()','pico_rails::Service::tickWake()'):
                assert name in symbols,'Missing actual runtime symbol '+name
            for name in ('i2c_new_master_bus','i2c_master_transmit_receive','i2c_master_cmd_begin'):
                assert not re.search(r'\b'+name+r'$',symbols,re.M),'Unexpected stock transfer '+name
    print(f'{len(MODES)} cell runtime scenarios PASS. No real BQ, USB, charging or motor operation.',flush=True)

if __name__=='__main__':main()
