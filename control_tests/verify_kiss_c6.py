#!/usr/bin/env python3
"""Verify external-charge C6 firmware/host against the actual logic netlist.

Uses public-only staging with a failing private-config sentinel, no device IO.
--build compiles C6/CH343G and retained C6/R3; no upload targets are invoked.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from verify_pico_profile import ROOT, public_copy

LOGIC=ROOT/'hardware/top_board_v2/jlc_budget_6v/controller_core/logic'
OUT=LOGIC/'firmware_verification'


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--build',action='store_true')
    args=parser.parse_args();OUT.mkdir(exist_ok=True)
    # Current KiCad export, tied to the already completed native schematic check.
    report=json.loads((LOGIC/'verification/report.json').read_text())
    for group in ('source_sha256','verification_code_sha256'):
        for name,sha in report[group].items():
            assert hashlib.sha256(Path(name).read_bytes()).hexdigest()==sha,name
    rows=[]
    watched=[p for folder in ('src','include','control_tests') for p in (ROOT/folder).rglob('*')
             if p!=ROOT/'include/config.h' and p.suffix in ('.h','.cpp','.c')]
    watched += [ROOT/n for n in ('scripts/pcb_control.py','scripts/robot_control.py',
        'scripts/requirements-control.txt','platformio.ini','control_tests/test_ch343g_host.py',
        'control_tests/verify_kiss_c6.py')]
    watched += sorted((ROOT/'control_tests').glob('test_*host.py'))
    def hashes():return {str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in watched}
    before=hashes()
    with tempfile.TemporaryDirectory(prefix='wbb-kiss-c6-public-') as temporary:
        stage=Path(temporary);public_copy(stage)
        def run(name,cmd,cwd=stage,rejection=None):
            print(name+' START',flush=True)
            r=subprocess.run([str(x) for x in cmd],cwd=cwd,capture_output=True,text=True,timeout=900)
            output=r.stdout+r.stderr;(OUT/(name+'.log')).write_text(output)
            passed=r.returncode==0 if rejection is None else r.returncode!=0 and rejection in output
            rows.append(dict(name=name,passed=passed,exit_code=r.returncode,
                             command=[str(x).replace(str(stage),'$STAGE') for x in cmd]))
            print(name+(' PASS' if passed else ' FAIL'),flush=True)
            if not passed:raise RuntimeError(name+' failed: '+output[-4000:])
            return output
        common=['c++','-std=c++17','-fsanitize=address,undefined','-fno-sanitize-recover=all',
                '-Icontrol_tests/mock','-Icontrol_tests/pico_mock','-Iinclude','-Isrc',
                '-I'+str(ROOT/'.pio/libdeps/top_board_v2/ArduinoJson/src')]
        flags=['-DWBB_TOP_BOARD_KISS_C6=1','-DCONFIG_IDF_TARGET_ESP32C6=1',
               '-DARDUINO_USB_CDC_ON_BOOT=0','-DWBB_TOP_BOARD_V2=1',
               '-include','include/kiss_c6_build_config.h']
        binary=stage/'kiss-native'
        run('production_runtime_compile',common+flags+['-MMD','-MF','kiss.d','control_tests/test_kiss_c6_profile.cpp','-o',binary])
        assert 'include/config.h' not in (stage/'kiss.d').read_text()
        scenarios=['startup','status_budget','rails','adc_lease_deadline','arm_preload','arm_config','arm_hold','release_failure',
                   'reboot_usb','reboot_wifi','reboot_ble']
        scenarios += ['radio_usb_only','radio_adc_failure','radio_no_watchdog','radio_qualify',
                      'radio_battery_loss','radio_adc_loss','radio_stale','radio_init_loss','radio_gpio_loss',
                      'radio_init_fail','radio_service_fail','radio_advertise_fail','radio_wifi_fail',
                      'radio_prior_reset','radio_cold_cookie','radio_bad_cookie']
        scenarios += [f'motor_{op}_{pin}' for op in ('level','config') for pin in (22,23,20,21,18,19)]
        for scenario in scenarios:
            output=run(scenario,[binary,scenario])
            if scenario=='rails':
                rail=json.loads(next(l.removeprefix('KISS_RAILS ') for l in output.splitlines() if l.startswith('KISS_RAILS ')))
                (OUT/'rails.json').write_text(json.dumps(rail,indent=2)+'\n')
            if scenario=='startup':
                observed=json.loads(next(l.removeprefix('KISS_OBSERVED ') for l in output.splitlines() if l.startswith('KISS_OBSERVED ')))
                status=json.loads(next(l.removeprefix('KISS_STATUS ') for l in output.splitlines() if l.startswith('KISS_STATUS ')))
                (OUT/'observed.json').write_text(json.dumps(observed,indent=2)+'\n')
                (OUT/'status.json').write_text(json.dumps(status,indent=2)+'\n')
        net={}
        for n in ET.parse(LOGIC/'logic.net.xml').getroot().findall('./nets/net'):
            for p in n.findall('node'):net[p.attrib['ref'],p.attrib['pin']]=n.attrib['name']
        # Physical module pads independently retained from Espressif table 3-1.
        gpio_pad={0:12,1:13,2:5,3:6,6:15,7:16,14:19,16:31,17:30,18:24,19:25,20:26,21:27,22:28,23:29}
        assert observed['arm']==14 and net['U3',str(gpio_pad[observed['arm']])]=='MOTOR_ARM_REQUEST'
        assert observed['uart']==[16,17]
        assert observed['outputs']==[14,18,19,20,21,22,23]
        assert observed['encoders']==[0,1,2,3,6,7]
        assert observed['adc_gpio']==[4,5]
        assert net['U3','9']==net['R33','2']==net['R34','1']==net['C20','1']=='VBAT_SENSE'
        assert net['U3','10']==net['R35','2']==net['R36','1']==net['C21','1']=='VMOT_SENSE'
        assert observed['motor_config_groups']==[22,23,20,21,18,19] # gpio_config iterates low-to-high within each pair.
        for i,h in enumerate(('M0','M2','M1')):
            actual={net['U3',str(gpio_pad[p])] for p in observed['motor_config_groups'][2*i:2*i+2]}
            assert actual=={net[h,'2'],net[h,'3']}
        for pin,*deltas in observed['edge_responses']:
            wheel=0 if pin in (0,1) else 1 if pin in (6,7) else 2
            assert abs(deltas[wheel])==1 and all(v==0 for i,v in enumerate(deltas) if i!=wheel)
            h=('M0','M2','M1')[wheel]
            assert net['U3',str(gpio_pad[pin])] in {net[h,'4'],net[h,'5']}
        rows.append(dict(name='compiled_consumers_match_actual_kicad_netlist',passed=True))
        adc_binary=stage/'kiss-adc'
        run('adc_failure_boundaries_compile',common+flags+['control_tests/test_kiss_c6_voltage.cpp','-o',adc_binary])
        run('adc_failure_boundaries',[adc_binary])
        radio_binary=stage/'kiss-radio-power'
        run('radio_power_boundaries_compile',common+flags+['control_tests/test_kiss_c6_radio_power.cpp','-o',radio_binary])
        run('radio_power_boundaries',[radio_binary])
        transport_binary=stage/'transport-regression'
        run('shared_transport_regressions_compile',common+['control_tests/test_transport.cpp','-o',transport_binary])
        run('shared_transport_regressions',[transport_binary])
        # Reject incompatible builds before any runtime/flash could be produced.
        source=stage/'profile.cpp';source.write_text('#include "firmware_config.h"\nint main(){}\n')
        for name,changed,expected in [
            ('wrong_mcu',[f for f in flags if 'CONFIG_IDF_TARGET' not in f]+['-DCONFIG_IDF_TARGET_ESP32S3=1'],'requires ESP32-C6'),
            ('wrong_usb',[f for f in flags if 'CDC_ON_BOOT' not in f]+['-DARDUINO_USB_CDC_ON_BOOT=1'],'requires UART0'),
            ('wrong_entry',flags+['-DMOTOR_TEST_MODE=1'],'requires the gated'),
            ('mixed_profile',flags+['-DWBB_TOP_BOARD_R3=1'],'cannot be combined')]:
            run('reject_'+name,common+changed+[source,'-o',stage/'reject'],rejection=expected)
        # Shared command boundary, including authentication-time lease expiry.
        for name,extra in [('native',[]),('r3',['-DWBB_TOP_BOARD_R3=1','-DCONFIG_IDF_TARGET_ESP32C6=1','-DARDUINO_USB_CDC_ON_BOOT=0']),('kiss',flags)]:
            exe=stage/('commands-'+name)
            run('commands_'+name+'_compile',common+extra+['control_tests/test_runtime_commands.cpp','-o',exe])
            run('commands_'+name,[exe])
        # Shared C6 LOW helper and driver keep their retained R3 failure behavior.
        exe=stage/'r3-gpio'
        run('retained_r3_gpio_compile',common+['control_tests/test_r3_gpio_init.cpp','-o',exe])
        for scenario in ('cold','warm','preload_failure','config_failure','write_failure',
                         'provision_failure','overrun_failure','wireless_reboot_failure'):
            run('retained_r3_gpio_'+scenario,[exe,scenario])
        exe=stage/'r3-driver'
        r3=['-DWBB_TOP_BOARD_R3=1','-DCONFIG_IDF_TARGET_ESP32C6=1','-DARDUINO_USB_CDC_ON_BOOT=0','-DWBB_TOP_BOARD_V2=1','-include','include/config.example.h']
        run('retained_r3_driver_compile',common+r3+['control_tests/test_r3_motor_gpio.cpp','-o',exe])
        for scenario in ('cold','warm'):run('retained_r3_driver_'+scenario,[exe,scenario])
        run('host_regressions',[ROOT/'.venv/bin/python','-m','unittest','discover','-s','control_tests','-p','test_*host.py','-v'],ROOT)
        run('control_host_regressions',[ROOT/'.venv/bin/python','-m','unittest','discover','-s','control_tests','-p','test_host.py','-v'],ROOT)
        run('actual_status_host_validation',[ROOT/'.venv/bin/python','-c',
            'import sys,json;sys.path.insert(0,"scripts");import robot_control as h;h.check_status(json.load(open(sys.argv[1])),h.KISS_C6_PROFILE);print("Actual compiled status accepted")',OUT/'status.json'],ROOT)
        run('actual_rails_host_validation',[ROOT/'.venv/bin/python','-c',
            'import sys,json;sys.path.insert(0,"scripts");import robot_control as h;h.check_kiss_rail_report(json.load(open(sys.argv[1])),h.KISS_C6_PROFILE);print("Actual compiled rails accepted")',OUT/'rails.json'],ROOT)
        if args.build:
            for env in ('top_board_v2_kiss_c6','top_board_v2_r3'):
                run(env+'_cross_build',[ROOT/'.venv/bin/pio','run','-e',env])
    after=hashes();assert before==after,'Sources changed during verification'
    result=dict(time_utc=datetime.now(timezone.utc).isoformat(),all_passed=True,checks=rows,source_hashes=after,
                schematic_source_hashes=report['source_sha256'],cross_compiled=args.build,
                scope='Production C6 profile/runtime/driver/ISRs/transport with SDK and OS doubles; no hardware qualification',
                physical_hardware_tested=False,firmware_uploaded=False,private_configuration_read=False)
    (OUT/'report.json').write_text(json.dumps(result,indent=2)+'\n')
    print('All firmware/host checks passed; board power, PCB and physical qualification remain incomplete.',flush=True)


if __name__=='__main__':main()
