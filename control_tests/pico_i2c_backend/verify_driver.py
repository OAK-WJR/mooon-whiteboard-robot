#!/usr/bin/env python3
"""Exercise production C wait/dispatch functions, plus public-only link audit."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[2]
VENDOR=ROOT/'third_party/esp_idf_i2c_no_clear'


def function(source,name):
    match=re.search(r'^static [\w *]+ '+name+r'\([^;]*?\)\s*\{',source,re.M)
    assert match,name
    start=source.index('{',match.start());end=start+1;depth=1
    while depth:
        depth+=(source[end]=='{')-(source[end]=='}');end+=1
    return source[match.start():end]


def run(command,cwd):
    result=subprocess.run([str(v) for v in command],cwd=cwd,capture_output=True,text=True,timeout=900)
    if result.returncode:print((result.stdout+result.stderr)[-6000:]);raise RuntimeError(command[0])
    return result.stdout+result.stderr


def common_bus_layout(dwarf):
    headers=list(re.finditer(r'^ <(\d+)><[0-9a-f]+>: Abbrev Number:.*?(?:\((DW_TAG_\w+)\))?$',dwarf,re.M))
    rows=[]
    for i,h in enumerate(headers):
        attrs=dwarf[h.end():headers[i+1].start() if i+1<len(headers) else len(dwarf)]
        tag=re.search(r'\((DW_TAG_\w+)\)',h.group(0))
        rows.append((int(h[1]),tag[1] if tag else None,attrs))
    def attr(text,name):
        line=next(l for l in text.splitlines() if 'DW_AT_'+name in l)
        return line.rsplit(':',1)[-1].strip()
    found=[]
    for i,(depth,tag,attrs) in enumerate(rows):
        if tag!='DW_TAG_structure_type' or 'DW_AT_byte_size' not in attrs or 'DW_AT_name' not in attrs:continue
        if attr(attrs,'name')!='i2c_bus_t':continue
        layout={'size':int(attr(attrs,'byte_size')),'members':{}}
        for dep,t,a in rows[i+1:]:
            if dep<=depth:break
            if dep==depth+1 and t=='DW_TAG_member':layout['members'][attr(a,'name')]=int(attr(a,'data_member_location'))
        found.append(layout)
    assert found and all(v==found[0] for v in found),'Missing/inconsistent I2C shared-base DWARF'
    return found[0]


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--build',action='store_true');args=parser.parse_args()
    source=(VENDOR/'i2c_master.c').read_text();manifest=json.loads((VENDOR/'source_manifest.json').read_text())
    for name,record in manifest['sources'].items():assert hashlib.sha256((VENDOR/'upstream'/name).read_bytes()).hexdigest()==record['sha256']
    for name,digest in manifest['adapted_sha256'].items():assert hashlib.sha256((VENDOR/name).read_bytes()).hexdigest()==digest
    assert 'i2c_ll_master_clr_bus(' not in source and 'gpio_set_level(' not in source
    assert not re.search(r's_i2c_hw_fsm_reset\([^;]+, true\)',source)
    assert 'return ESP_ERR_NOT_SUPPORTED' in function(source,'s_i2c_master_clear_bus')
    # The official framing builder remains intact: repeated START and final NACK.
    assert '.hw_cmd = I2C_TRANS_READ_COMMAND(I2C_NACK_VAL)' in source
    with tempfile.TemporaryDirectory(prefix='wbb-driver-functions-') as temp:
        stage=Path(temp)
        names=['wbb_expired','wbb_ticks_left','wbb_take','wbb_event','wbb_quiesce','s_i2c_send_commands']
        (stage/'actual_driver_functions.inc').write_text('\n\n'.join(function(source,n) for n in names)+'\n')
        shutil.copyfile(ROOT/'control_tests/pico_i2c_backend/test_driver_waits.c',stage/'test.c')
        run(['cc','-std=c11','-Wall','-Wextra','-Werror','-pedantic','-fsanitize=address,undefined','-fno-sanitize-recover=all','test.c','-o','test'],stage)
        print(run([stage/'test'],stage),end='',flush=True)
        if args.build:
            spec=importlib.util.spec_from_file_location('public_copy',ROOT/'control_tests/verify_pico_profile.py')
            helper=importlib.util.module_from_spec(spec);spec.loader.exec_module(helper);helper.public_copy(stage)
            ini=stage/'platformio.ini';ini.write_text(ini.read_text().replace('    -DWBB_TOP_BOARD_PICO=1',
                '    -DWBB_TOP_BOARD_PICO=1\n    -Wl,-u,wbb_i2c_open,-u,wbb_i2c_transfer,-u,wbb_i2c_close'))
            for env in ['top_board_v2_pico','top_board_v2_r3']:
                print(env+' public build START',flush=True)
                output=run([ROOT/'.venv/bin/pio','run','-e',env],stage)
                print('\n'.join(line for line in output.splitlines() if 'RAM:' in line or 'Flash:' in line),flush=True)
                for dependency in (stage/'.pio/build'/env/'src').rglob('*.d'):
                    assert 'include/config.h' not in dependency.read_text()
            nm=next((ROOT/'.platformio/packages').glob('toolchain-xtensa-esp-elf/bin/xtensa-esp32s3-elf-nm'))
            elf=stage/'.pio/build/top_board_v2_pico/firmware.elf'
            symbols=run([nm,'--defined-only',elf],stage)
            for name in ['wbb_i2c_open','wbb_i2c_transfer','wbb_i2c_close','wbb_idf554_i2c_new_master_bus',
                         'wbb_idf554_i2c_master_transmit_receive']:
                assert re.search(r'\b'+name+r'$',symbols,re.M),name
            for name in ['i2c_new_master_bus','i2c_master_transmit_receive','i2c_master_cmd_begin']:
                assert not re.search(r'\b'+name+r'$',symbols,re.M),'Unexpected stock transfer path: '+name
            objdump=nm.with_name('xtensa-esp32s3-elf-objdump')
            sdk=ROOT/'.platformio/packages/framework-arduinoespressif32-libs/esp32s3'
            assert 'esp-idf: v5.5.4 735507283d' in (sdk/'versions.txt').read_text()
            stock_layout=common_bus_layout(run([objdump,'--dwarf=info',sdk/'lib/libesp_driver_i2c.a'],stage))
            local_layout=common_bus_layout(run([objdump,'--dwarf=info',stage/'.pio/build/top_board_v2_pico/src/pico_i2c_native.c.o'],stage))
            assert stock_layout==local_layout
            print('Shared i2c_bus_t ABI equals installed SDK: '+json.dumps(local_layout),flush=True)
            print('Forced Pico hardware symbols link to namespaced driver; no stock master transfer symbols PASS',flush=True)
    print('Offline only: no real bus/USB access, no full silicon/timing qualification.',flush=True)


if __name__=='__main__':main()
