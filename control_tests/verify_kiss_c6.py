#!/usr/bin/env python3
"""Verify the KISS C6 firmware and host tool against the committed EasyEDA netlist.

Board: KISS C6 logic board (EasyEDA, 2026-10-29). ESP32-C6-MINI-1 with the native USB
Serial/JTAG CDC console (no USB-UART bridge), ARM/MOT_EN on GPIO0, IO5 = INA180 motor
current, IO8 = USB-power sense, on-board IP2326 2S charger.

Builds run in a public-only staged copy with a failing private-config sentinel. No device
IO and no upload. --build also cross-compiles top_board_v2_kiss_c6 and the retained
top_board_v2_r3 with PlatformIO.
"""
import argparse
from collections import defaultdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import subprocess
import tempfile

from verify_pico_profile import ROOT, public_copy

NETLIST = ROOT / 'hardware/top_board_v2/easyeda/logic_netlist.json'
PROFILE_HEADER = ROOT / 'include/board_profile_kiss_c6.h'
OUT = ROOT / 'hardware/top_board_v2/jlc_budget_6v/controller_core/logic/firmware_verification'
GENERATED_JSON = ('observed.json', 'status.json', 'rails.json', 'fault_status.json', 'profile.json', 'report.json')

# ESP32-C6-MINI-1 module pads, datasheet v1.5 Table 3-1 "Pin Definitions"
# (https://documentation.espressif.com/esp32-c6-mini-1_mini-1u_datasheet_en.pdf).
# RXD0 (pad 30) is GPIO17 and TXD0 (pad 31) is GPIO16. GPIO10/11 are not bonded out.
MINI1_GPIO_PAD = {0: 12, 1: 13, 2: 5, 3: 6, 4: 9, 5: 10, 6: 15, 7: 16, 8: 22, 9: 23, 12: 17, 13: 18,
                  14: 19, 15: 20, 16: 31, 17: 30, 18: 24, 19: 25, 20: 26, 21: 27, 22: 28, 23: 29}
MINI1_POWER_PADS = {'3V3': (3,), 'EN': (8,), 'GND': (1, 2, 11, 14, *range(36, 54))}
MINI1_NC_PADS = (4, 7, 21, 32, 33, 34, 35)
MODULE = 'U3'
USB_C = 'USB1'  # USB-C receptacle: D+ on A6/B6, D- on A7/B7
BOOT_GPIO, USB_DN_GPIO, USB_DP_GPIO = 9, 12, 13  # fixed C6 functions, not board choices


def load_netlist():
    raw = json.loads(NETLIST.read_text())
    assert isinstance(raw, dict) and raw, 'netlist must be a flat {"ref.pin": "net"} object'
    net = {}
    for key, name in raw.items():
        assert isinstance(key, str) and re.fullmatch(r'[A-Za-z]+\d*\.[A-Za-z0-9]+', key), key
        assert isinstance(name, str) and name, key
        net[tuple(key.split('.', 1))] = name
    members = defaultdict(set)
    for node, name in net.items():
        members[name].add(node)
    return net, members


def two_terminal(net, prefix):
    """{ref: (net of pin 1, net of pin 2)} for every two-pin part whose reference starts with prefix."""
    parts = defaultdict(dict)
    for (ref, pin), name in net.items():
        if re.fullmatch(prefix + r'\d+', ref):
            parts[ref][pin] = name
    return {ref: (pins['1'], pins['2']) for ref, pins in parts.items() if set(pins) == {'1', '2'}}


def parse_profile_header():
    """Board constants as written in include/board_profile_kiss_c6.h (cross-check of the compiled profile)."""
    text = PROFILE_HEADER.read_text()
    def one(pattern):
        found = re.findall(pattern, text)
        assert len(found) == 1, pattern
        return found[0]
    battery, motor = one(r'constexpr int batterySense = (\d+), motorSense = (\d+);')
    sda, scl, ir = one(r'constexpr int i2cSda = (\d+), i2cScl = (\d+), irEnableN = (\d+);')
    headers = re.findall(r'\{(\d+), (\d+), (\d+), (\d+), (\d+), "(M\d)"\}', text)
    assert len(headers) == 3, 'expected three motor headers in the profile'
    return dict(arm=int(one(r'constexpr int motorArmRequest = (\d+);')), battery_sense=int(battery),
                motor_sense=int(motor), usb_power_sense=int(one(r'constexpr int usbPowerSense = (\d+);')),
                i2c_sda=int(sda), i2c_scl=int(scl), ir_enable_n=int(ir),
                headers=[dict(header=h, fi=int(fi), bi=int(bi), enc_a=int(a), enc_b=int(b), physical=int(p))
                         for fi, bi, a, b, p, h in headers])


def check_usb_power_sense(io8, net, members, resistors):
    """IO8 isolation (2026-10-29): the only accepted circuit is VBUS -> R -> tap -> R -> GND, the tap reaching IO8
    through one SOT-23 N-MOSFET (1 G = 3V3, 2 S = IO8, 3 D = tap) and IO8 pulled up to 3V3 by its own resistor.
    Nothing else may sit on IO8 or on the tap. A divider tap wired straight to IO8 (the pre-isolation board) back-feeds
    the unpowered C6 while the pack charges with SW1 off, so it is rejected. Part values are not in the netlist: the
    low level without USB depends on them (see FIRMWARE.md) and is not checked here."""
    def parts(a, b):
        return sorted(ref for ref, pins in resistors.items() if set(pins) == {a, b})

    def refs(name):
        return sorted(ref for ref, _ in members.get(name, ()))

    assert io8 and io8 not in ('VBUS', 'GND', '3V3'), f'IO8 is on {io8}'
    assert not parts('VBUS', io8), 'a VBUS resistor reaches IO8 without the isolation FET'
    fets = sorted({ref for ref, pin in net if re.fullmatch(r'Q\d+', ref) and pin == '2' and net[(ref, pin)] == io8})
    assert len(fets) == 1, f'IO8 ({io8}) must reach the VBUS divider through exactly one isolation FET, found {fets}'
    fet = fets[0]
    assert sorted(pin for ref, pin in net if ref == fet) == ['1', '2', '3'], f'{fet} is not a three-pin SOT-23 FET'
    gate, tap = net[(fet, '1')], net[(fet, '3')]
    assert gate == '3V3', f'{fet} gate is on {gate}, not 3V3: the FET would not block while 3V3 is off'
    assert tap not in ('VBUS', 'GND', '3V3', io8), f'{fet} drain is on {tap}, not a divider tap'
    top, bottom, pullup = parts('VBUS', tap), parts(tap, 'GND'), parts('3V3', io8)
    assert len(top) == 1 and len(bottom) == 1, f'{fet} drain ({tap}) is not one VBUS/GND divider tap: {top} {bottom}'
    assert len(pullup) == 1, f'IO8 needs exactly one pull-up to 3V3, found {pullup}'
    assert refs(tap) == sorted([fet, top[0], bottom[0]]) and len(members[tap]) == 3, f'extra parts on the tap: {refs(tap)}'
    assert refs(io8) == sorted([MODULE, fet, pullup[0]]) and len(members[io8]) == 3, f'extra parts on IO8: {refs(io8)}'
    return f'VBUS divider {top[0]}/{bottom[0]} isolated by {fet} (gate 3V3), {pullup[0]} pull-up on IO8'


def usb_sense_regressions(net):
    """Netlist edits the IO8 check must reject: each returns a changed copy of the {(ref, pin): net} map."""
    io8 = net[(MODULE, str(MINI1_GPIO_PAD[8]))]
    fet = next(ref for (ref, pin), name in net.items() if re.fullmatch(r'Q\d+', ref) and pin == '2' and name == io8)
    tap = net[(fet, '3')]

    def edit(drop=(), **moves):
        changed = {k: v for k, v in net.items() if k[0] not in drop}
        for old, new in moves.items():
            changed = {k: (new if v == old else v) for k, v in changed.items()}
        return changed
    pullup = next(ref for (ref, _), name in net.items() if ref.startswith('R') and name == io8)
    yield 'divider_straight_to_io8', edit(drop=(fet, pullup), **{tap: io8})
    yield 'no_pull_up', edit(drop=(pullup,))
    yield 'gate_not_3v3', {**net, (fet, '1'): 'VBUS'}
    yield 'extra_part_on_io8', {**net, ('C99', '1'): io8, ('C99', '2'): 'GND'}
    yield 'extra_part_on_tap', {**net, ('R99', '1'): tap, ('R99', '2'): '3V3'}


def check_against_netlist(profile, observed, net, members):
    """Firmware pins (compiled profile and observed behaviour) against the EasyEDA board netlist."""
    resistors = two_terminal(net, 'R')
    capacitors = two_terminal(net, 'C')

    def joined(parts, a, b):
        return any({x, y} == {a, b} for x, y in parts.values())

    def pad(gpio):
        return net.get((MODULE, str(MINI1_GPIO_PAD[gpio])))

    # Module power/NC pads are where the datasheet puts them, and every U3 pad is a datasheet pad.
    for name, pads in MINI1_POWER_PADS.items():
        for number in pads:
            assert net.get((MODULE, str(number))) == name, f'{MODULE}.{number} should be {name}'
    known = set(MINI1_GPIO_PAD.values()) | set(MINI1_NC_PADS) | {p for pads in MINI1_POWER_PADS.values() for p in pads}
    for ref, number in net:
        if ref == MODULE:
            assert int(number) in known, f'{MODULE}.{number} is not an ESP32-C6-MINI-1 pad'
    for number in MINI1_NC_PADS:
        name = net.get((MODULE, str(number)))
        assert name is None or members[name] == {(MODULE, str(number))}, f'NC pad {number} is connected'

    headers = {h['header']: h for h in profile['headers']}
    order = profile['header_order']
    assert sorted(order) == sorted(headers) == ['M0', 'M1', 'M2'] and len(set(order)) == 3
    assert [headers[h]['physical'] for h in order] == [1, 2, 3], 'status order is not physical Motor1/2/3'

    # ARM request (MOT_EN): a driven-LOW output with an external pull-down for reset/unpowered time.
    arm = profile['arm']
    assert observed['arm'] == arm and observed['arm_output'] == [arm, True, 0], observed['arm_output']
    assert pad(arm) == 'MOT_EN' and joined(resistors, 'MOT_EN', 'GND'), 'ARM pin is not MOT_EN with a pull-down'
    assert profile['fault_reason'] == f'GPIO{arm}_SAFE_LOW_FAILED', profile['fault_reason']

    # Native USB console: C6 D-/D+ through series resistors to the USB-C data pins; firmware never touches 12/13.
    assert observed['console'] == 'usb_cdc'
    for gpio, pins in ((USB_DN_GPIO, ('A7', 'B7')), (USB_DP_GPIO, ('A6', 'B6'))):
        connector = {net.get((USB_C, p)) for p in pins}
        assert len(connector) == 1 and None not in connector, f'{USB_C} {pins} not one net'
        assert joined(resistors, pad(gpio), connector.pop()), f'GPIO{gpio} does not reach the USB-C data pins'
        assert gpio in observed['untouched'], f'firmware configured USB GPIO{gpio}'
    assert not any(n in members for n in ('ESP_RXD', 'ESP_TXD')), 'UART bridge nets still present'

    # ADC: IO4 VBAT divider (top resistor from VBAT, bottom to GND, filter cap), IO5 INA180 output (not VMOT).
    assert observed['adc_gpio'] == [profile['battery_sense'], profile['motor_sense']]
    vbat = pad(profile['battery_sense'])
    assert vbat == 'VBAT_SENSE' and joined(resistors, 'VBAT', vbat) and joined(resistors, vbat, 'GND')
    assert joined(capacitors, vbat, 'GND'), 'VBAT_SENSE filter capacitor missing'
    isense = pad(profile['motor_sense'])
    assert isense == 'MOT_ISENSE' and joined(capacitors, isense, 'GND')
    amp_out = net.get(('U7', '1'))
    assert amp_out and joined(resistors, isense, amp_out), 'IO5 is not the INA180 (U7) output through its filter resistor'
    assert net.get(('U7', '5')) == '3V3', 'INA180 supply is not 3V3 (output range would not match the 12 dB ADC)'
    assert 'VMOT_SENSE' not in members and not joined(resistors, 'VMOT', isense), 'a VMOT divider reaches IO5'

    # IO8 USB-power sense: a plain input; IO8 high = USB/charger 5 V present.
    sense = profile['usb_power_sense']
    assert observed['plain_inputs'] == [sense], observed['plain_inputs']
    topology = check_usb_power_sense(pad(sense), net, members, resistors)

    # BOOT strap with its pull-up; never configured by firmware.
    assert pad(BOOT_GPIO) == 'BOOT' and joined(resistors, 'BOOT', '3V3') and BOOT_GPIO in observed['untouched']

    # Sensor head J1: SDA, SCL, IR_EN_N (pulled up, IR off); application pins untouched by the motor service.
    app = (profile['i2c_sda'], profile['i2c_scl'], profile['ir_enable_n'])
    assert tuple(pad(g) for g in app) == tuple(net.get(('J1', p)) for p in ('3', '4', '5')) == ('I2C_SDA', 'I2C_SCL', 'IR_EN_N')
    assert joined(resistors, 'IR_EN_N', '3V3') and all(g in observed['untouched'] for g in app)

    # Motor headers: header pin 3 = FI, 2 = BI, 5 = ENC_A, 4 = ENC_B.
    for name, h in headers.items():
        for field, header_pin in (('fi', '3'), ('bi', '2'), ('enc_a', '5'), ('enc_b', '4')):
            assert pad(h[field]) == net.get((name, header_pin)), f'{name}.{field} GPIO{h[field]} is not header pin {header_pin}'
    motor = sorted(p for h in headers.values() for p in (h['fi'], h['bi']))
    encoders = sorted(p for h in headers.values() for p in (h['enc_a'], h['enc_b']))
    assert observed['outputs'] == sorted([arm] + motor), observed['outputs']
    assert observed['encoders'] == encoders, observed['encoders']
    # gpio_config enables each FI/BI pair together, low pin first, motor index order = status order.
    assert observed['motor_config_groups'] == [p for h in order for p in sorted((headers[h]['fi'], headers[h]['bi']))]
    assert sorted(e[0] for e in observed['edge_responses']) == encoders
    for pin, *deltas in observed['edge_responses']:
        h = next(h for h in headers.values() if pin in (h['enc_a'], h['enc_b']))
        wheel = h['physical'] - 1
        assert abs(deltas[wheel]) == 1 and all(v == 0 for i, v in enumerate(deltas) if i != wheel), (pin, deltas)

    # Completeness: every module GPIO pad that is wired to something is one of the functions checked above.
    classified = {arm, *motor, *encoders, profile['battery_sense'], profile['motor_sense'], sense, *app,
                  BOOT_GPIO, USB_DN_GPIO, USB_DP_GPIO}
    for gpio, number in MINI1_GPIO_PAD.items():
        name = net.get((MODULE, str(number)))
        if name is not None and members[name] != {(MODULE, str(number))}:
            assert gpio in classified, f'GPIO{gpio} ({name}) is wired but not covered by the firmware profile'
    return topology


def prerequisites():
    """Fail before touching the report folder when the local toolchain pieces are missing."""
    for env in ('top_board_v2_kiss_c6', 'top_board_v2'):
        json_include = ROOT / '.pio/libdeps' / env / 'ArduinoJson/src'
        if (json_include / 'ArduinoJson.h').is_file():
            break
    else:
        raise SystemExit('ArduinoJson headers missing (gitignored .pio/libdeps). Fetch them first: '
                         '.venv/bin/pio pkg install -e top_board_v2_kiss_c6')
    if not (ROOT / '.venv/bin/python').is_file():
        raise SystemExit('.venv missing: python3 -m venv .venv && .venv/bin/pip install platformio '
                         '-r scripts/requirements-control.txt')
    return json_include


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', action='store_true')
    args = parser.parse_args()
    json_include = prerequisites()
    OUT.mkdir(parents=True, exist_ok=True)
    for stale in list(OUT.glob('*.log')) + [OUT / n for n in GENERATED_JSON]:
        stale.unlink(missing_ok=True)
    rows = []
    net, members = load_netlist()
    watched = [p for folder in ('src', 'include', 'control_tests') for p in (ROOT / folder).rglob('*')
               if p != ROOT / 'include/config.h' and p.suffix in ('.h', '.cpp', '.c')]
    watched += [ROOT / n for n in ('scripts/pcb_control.py', 'scripts/robot_control.py', 'scripts/requirements-control.txt',
                                   'platformio.ini', 'control_tests/test_c6usb_host.py', 'control_tests/verify_kiss_c6.py')]
    watched += sorted((ROOT / 'control_tests').glob('test_*host.py')) + [NETLIST]

    def hashes():
        return {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() if p.is_file() else 'missing'
                for p in sorted(set(watched))}

    before = hashes()
    result = dict(all_passed=False, failed_step=None)
    try:
        with tempfile.TemporaryDirectory(prefix='wbb-kiss-c6-public-') as temporary:
            stage = Path(temporary)
            public_copy(stage)

            def clean(text):
                return text.replace(str(stage), '$STAGE').replace(str(ROOT), '$ROOT')

            def run(name, cmd, cwd=stage, rejection=None):
                print(name + ' START', flush=True)
                result['failed_step'] = name
                r = subprocess.run([str(x) for x in cmd], cwd=cwd, capture_output=True, text=True, timeout=900)
                output = r.stdout + r.stderr
                (OUT / (name + '.log')).write_text(clean(output))
                passed = r.returncode == 0 if rejection is None else r.returncode != 0 and rejection in output
                rows.append(dict(name=name, passed=passed, exit_code=r.returncode, command=[clean(str(x)) for x in cmd]))
                print(name + (' PASS' if passed else ' FAIL'), flush=True)
                if not passed:
                    raise RuntimeError(name + ' failed: ' + output[-4000:])
                result['failed_step'] = None
                return output

            def tagged(output, tag):
                return json.loads(next(l.removeprefix(tag + ' ') for l in output.splitlines() if l.startswith(tag + ' ')))

            common = ['c++', '-std=c++17', '-fsanitize=address,undefined', '-fno-sanitize-recover=all',
                      '-Icontrol_tests/mock', '-Icontrol_tests/pico_mock', '-Iinclude', '-Isrc',
                      '-I' + str(json_include)]
            # Same selection as platformio.ini env top_board_v2_kiss_c6: native USB CDC console.
            flags = ['-DWBB_TOP_BOARD_KISS_C6=1', '-DCONFIG_IDF_TARGET_ESP32C6=1', '-DARDUINO_USB_MODE=1',
                     '-DARDUINO_USB_CDC_ON_BOOT=1', '-DWBB_TOP_BOARD_V2=1', '-include', 'include/kiss_c6_build_config.h']
            binary = stage / 'kiss-native'
            run('production_runtime_compile', common + flags + ['-MMD', '-MF', 'kiss.d', 'control_tests/test_kiss_c6_profile.cpp', '-o', binary])
            assert 'include/config.h' not in (stage / 'kiss.d').read_text()
            output = run('startup', [binary, 'startup'])
            profile, observed, status = (tagged(output, t) for t in ('KISS_PROFILE', 'KISS_OBSERVED', 'KISS_STATUS'))
            for name, data in (('profile', profile), ('observed', observed), ('status', status)):
                (OUT / (name + '.json')).write_text(json.dumps(data, indent=2) + '\n')
            source_profile = parse_profile_header()
            assert {k: profile[k] for k in source_profile} == source_profile, 'compiled profile differs from board_profile_kiss_c6.h'
            result['failed_step'] = 'compiled_consumers_match_easyeda_netlist'
            topology = check_against_netlist(profile, observed, net, members)
            rows.append(dict(name='compiled_consumers_match_easyeda_netlist', passed=True, usb_power_sense=topology))
            print('compiled_consumers_match_easyeda_netlist PASS (' + topology + ')', flush=True)
            # The IO8 check must fail on the topologies it exists to exclude (un-isolated divider, missing pull-up...).
            result['failed_step'] = 'usb_power_sense_rejects_regressions'
            rejected = []
            for name, changed in usb_sense_regressions(net):
                changed_members = defaultdict(set)
                for node, label in changed.items():
                    changed_members[label].add(node)
                try:
                    check_usb_power_sense(changed[(MODULE, str(MINI1_GPIO_PAD[profile['usb_power_sense']]))],
                                          changed, changed_members, two_terminal(changed, 'R'))
                except AssertionError:
                    rejected.append(name)
                else:
                    raise AssertionError(f'IO8 check accepted the {name} netlist edit')
            rows.append(dict(name='usb_power_sense_rejects_regressions', passed=True, rejected=rejected))
            print('usb_power_sense_rejects_regressions PASS (' + ', '.join(rejected) + ')', flush=True)
            result['failed_step'] = None

            scenarios = ['status_budget', 'rails', 'rails_budget', 'adc_lease_deadline', 'arm_preload', 'arm_config',
                         'arm_hold', 'release_failure', 'reboot_usb', 'reboot_wifi', 'reboot_ble']
            scenarios += ['radio_usb_only', 'radio_adc_failure', 'radio_no_watchdog', 'radio_qualify',
                          'radio_battery_loss', 'radio_adc_loss', 'radio_stale', 'radio_init_loss', 'radio_gpio_loss',
                          'radio_start_gpio_fault', 'radio_init_fail', 'radio_service_fail', 'radio_advertise_fail',
                          'radio_wifi_fail', 'radio_prior_reset', 'radio_cold_cookie', 'radio_bad_cookie']
            # GPIO fault latched at boot with a healthy battery: read-only radio, supply-loss restarts, budgets.
            scenarios += ['fault_wireless', 'fault_no_watchdog', 'fault_init_loss', 'fault_battery_loss',
                          'fault_adc_loss', 'fault_stale', 'fault_budget']
            # Fault injection on every FI/BI output, taken from the compiled profile so it cannot drift.
            motor_pins = [p for h in profile['headers'] for p in (h['fi'], h['bi'])]
            scenarios += [f'motor_{op}_{pin}' for op in ('level', 'config') for pin in motor_pins]
            for scenario in scenarios:
                output = run(scenario, [binary, scenario])
                if scenario == 'rails':
                    (OUT / 'rails.json').write_text(json.dumps(tagged(output, 'KISS_RAILS'), indent=2) + '\n')
                if scenario == 'fault_wireless':
                    (OUT / 'fault_status.json').write_text(json.dumps(tagged(output, 'KISS_FAULT_STATUS'), indent=2) + '\n')
            adc_binary = stage / 'kiss-adc'
            run('adc_failure_boundaries_compile', common + flags + ['control_tests/test_kiss_c6_voltage.cpp', '-o', adc_binary])
            run('adc_failure_boundaries', [adc_binary])
            radio_binary = stage / 'kiss-radio-power'
            run('radio_power_boundaries_compile', common + flags + ['control_tests/test_kiss_c6_radio_power.cpp', '-o', radio_binary])
            run('radio_power_boundaries', [radio_binary])
            transport_binary = stage / 'transport-regression'
            run('shared_transport_regressions_compile', common + ['control_tests/test_transport.cpp', '-o', transport_binary])
            run('shared_transport_regressions', [transport_binary])
            # Reject incompatible builds before any runtime/flash could be produced.
            source = stage / 'profile.cpp'
            source.write_text('#include "firmware_config.h"\nint main(){}\n')
            no_cdc = [f for f in flags if 'CDC_ON_BOOT' not in f]
            for name, changed, expected in [
                    ('wrong_mcu', [f for f in flags if 'CONFIG_IDF_TARGET' not in f] + ['-DCONFIG_IDF_TARGET_ESP32S3=1'], 'requires ESP32-C6'),
                    ('wrong_usb', no_cdc + ['-DARDUINO_USB_CDC_ON_BOOT=0'], 'has no USB-UART bridge'),
                    ('missing_usb', no_cdc, 'has no USB-UART bridge'),
                    ('wrong_entry', flags + ['-DMOTOR_TEST_MODE=1'], 'requires the gated'),
                    ('mixed_profile', flags + ['-DWBB_TOP_BOARD_R3=1'], 'cannot be combined')]:
                run('reject_' + name, common + changed + [source, '-o', stage / 'reject'], rejection=expected)
            run('accept_usb_cdc', common + flags + [source, '-o', stage / 'accept'])
            # Shared command boundary, including authentication-time lease expiry.
            r3_flags = ['-DWBB_TOP_BOARD_R3=1', '-DCONFIG_IDF_TARGET_ESP32C6=1', '-DARDUINO_USB_CDC_ON_BOOT=0']
            for name, extra in [('native', []), ('r3', r3_flags), ('kiss', flags)]:
                exe = stage / ('commands-' + name)
                run('commands_' + name + '_compile', common + extra + ['control_tests/test_runtime_commands.cpp', '-o', exe])
                run('commands_' + name, [exe])
            # Shared C6 LOW helper and driver keep their retained R3 failure behavior.
            exe = stage / 'r3-gpio'
            run('retained_r3_gpio_compile', common + ['control_tests/test_r3_gpio_init.cpp', '-o', exe])
            for scenario in ('cold', 'warm', 'preload_failure', 'config_failure', 'write_failure',
                             'provision_failure', 'overrun_failure', 'wireless_reboot_failure'):
                run('retained_r3_gpio_' + scenario, [exe, scenario])
            exe = stage / 'r3-driver'
            r3 = r3_flags + ['-DWBB_TOP_BOARD_V2=1', '-include', 'include/config.example.h']
            run('retained_r3_driver_compile', common + r3 + ['control_tests/test_r3_motor_gpio.cpp', '-o', exe])
            for scenario in ('cold', 'warm'):
                run('retained_r3_driver_' + scenario, [exe, scenario])
            python = ROOT / '.venv/bin/python'
            # test_*host.py includes test_c6usb_host.py (native USB 303A:1001 identity, KISS profile binding).
            run('host_regressions', [python, '-m', 'unittest', 'discover', '-s', 'control_tests', '-p', 'test_*host.py', '-v'], ROOT)
            run('control_host_regressions', [python, '-m', 'unittest', 'discover', '-s', 'control_tests', '-p', 'test_host.py', '-v'], ROOT)
            for name, check, path in (('actual_status_host_validation', 'check_status', 'status.json'),
                                      ('actual_fault_status_host_validation', 'check_status', 'fault_status.json'),
                                      ('actual_rails_host_validation', 'check_kiss_rail_report', 'rails.json')):
                run(name, [python, '-c', 'import sys,json;sys.path.insert(0,"scripts");import robot_control as h;'
                            f'h.{check}(json.load(open(sys.argv[1])),h.KISS_C6_PROFILE);print("Actual compiled frame accepted")',
                            OUT / path], ROOT)
            if args.build:
                pio = ROOT / '.platformio/penv/bin/pio'
                if not pio.is_file():
                    pio = ROOT / '.venv/bin/pio'
                for env in ('top_board_v2_kiss_c6', 'top_board_v2_r3'):
                    run(env + '_cross_build', [pio, 'run', '-e', env])
        after = hashes()
        assert before == after, 'Sources changed during verification: ' + ', '.join(k for k in after if before.get(k) != after[k])
        result.update(all_passed=True, failed_step=None)
    finally:
        result.update(time_utc=datetime.now(timezone.utc).isoformat(), checks=rows, source_hashes=hashes(),
                      netlist=str(NETLIST.relative_to(ROOT)), netlist_sha256=hashlib.sha256(NETLIST.read_bytes()).hexdigest(),
                      module_pad_table='ESP32-C6-MINI-1 datasheet v1.5, Table 3-1', cross_compiled=args.build,
                      scope='Production C6 profile/runtime/driver/ISRs/transport with SDK and OS doubles; no hardware qualification',
                      physical_hardware_tested=False, firmware_uploaded=False, private_configuration_read=False)
        (OUT / 'report.json').write_text(json.dumps(result, indent=2) + '\n')
    print('All firmware/host checks passed; board power, PCB and physical qualification remain incomplete.', flush=True)


if __name__ == '__main__':
    main()
