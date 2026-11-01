"""Offline KISS C6 rail-report parser/client checks; every transport is a fake.

Fixtures mirror src/top_v2_main.cpp fillVoltageReport() on the 2026-10-29 board:
IO4 = VBAT divider (gain 13, 0 dB), IO5 = INA180A1 motor current (2 V/A, 12 dB),
no VMOT divider, IO8 = USB power sense, IP2326 charge state not observable.
"""
import unittest
from test_host import BOOT, KEY, FakeWire, envelope
from test_c6usb_host import fault as kiss_fault, online_fault, status as kiss_status
import robot_control as host


def rails(**changes):
    d = dict(id=1, ok=True, error="", boot=BOOT, lease=0, protocol=2, firmware="wbb-control-2",
             profile=host.KISS_C6_PROFILE, kind="rails", device=kiss_status()["device"], uptime_ms=5000,
             state="ready", adc_error=0, adc_cleanup_error=0, failing_channel=-1,
             fresh=True, read_only=True, motion_enabled=False, hardware_qualified=False,
             divider_calibrated=False, divider_gain=13, wireless_power_state="qualifying",
             wireless_start_attempted=False, previous_radio_power_reset=False,
             adc_raw=[2450, 570], adc_pin_mv=[600, 460], motor_voltage_mv=None,
             battery_voltage_mv=7800, motor_current_ma=230, motor_current_saturated=False,
             seq=1, age_ms=2, span_ms=0, usb_power=True, charge="external_power")
    d.update(changes)
    return d


def unavailable(**changes):
    d = rails(fresh=False, state="adc_read", adc_error=263, failing_channel=1,
              adc_raw=[None, None], adc_pin_mv=[None, None], battery_voltage_mv=None,
              motor_current_ma=None, seq=None, age_ms=None, span_ms=None)
    del d["motor_current_saturated"]  # Only emitted with a fresh sample.
    d.update(changes)
    return d


def saturated(**changes):
    return rails(**(dict(adc_raw=[2450, 4095], adc_pin_mv=[600, 3300], motor_current_ma=1650,
                         motor_current_saturated=True) | changes))


class RailReportTests(unittest.TestCase):
    def check(self, d):
        return host.check_kiss_rail_report(d, host.KISS_C6_PROFILE)

    def rejected(self, d, **context):
        with self.subTest(**context), self.assertRaises(host.DiagnosticError):
            self.check(d)

    def test_base_fixtures_are_accepted(self):
        # Every negative case below changes one field of these; a stale base would make them vacuous.
        for d in (rails(), unavailable(), saturated(), rails(usb_power=False, charge="battery"),
                  unavailable(usb_power=False, charge="battery")):
            self.assertIs(self.check(d), d)

    def test_fresh_and_zero_samples(self):
        self.check(rails(adc_raw=[0, 0], adc_pin_mv=[0, 0], battery_voltage_mv=0, motor_current_ma=0))
        self.check(rails(adc_pin_mv=[600, 461], motor_current_ma=230))  # floor(pin_mv / 2)
        self.check(rails(battery_voltage_mv=7806))  # rounding of the 8-sample average, |error| <= 6
        old = rails()
        for field in ("wireless_power_state", "wireless_start_attempted", "previous_radio_power_reset"):
            del old[field]
        self.check(old)  # Earlier read-only firmware omitted the wireless supply metadata.

    def test_charge_reports_power_source_only(self):
        for usb, charge in ((True, "external_power"), (False, "battery")):
            self.check(rails(usb_power=usb, charge=charge))
            self.check(unavailable(usb_power=usb, charge=charge))
        for make in (rails, unavailable):
            for change in (dict(usb_power=True, charge="battery"), dict(usb_power=False, charge="external_power"),
                           dict(charge=None), dict(usb_power=False, charge=None), dict(charge="charging"),
                           dict(charge="full"), dict(usb_power=False, charge="charging"),
                           dict(usb_power=1), dict(usb_power=None), dict(usb_power="true"), dict(charge=1)):
                self.rejected(make(**change), make=make.__name__, change=change)
        self.rejected(rails(battery_voltage_mv=8255, adc_pin_mv=[635, 460], charge="full"), case="no full state")

    def test_motor_current_scaling_and_saturation(self):
        self.check(saturated(adc_raw=[2450, 3990], adc_pin_mv=[600, 3240], motor_current_ma=1620))
        self.check(rails(adc_pin_mv=[600, 2900], motor_current_ma=1450, motor_current_saturated=True))  # one sample
        self.check(rails(adc_pin_mv=[600, 3149], motor_current_ma=1574))
        self.check(saturated(adc_pin_mv=[600, 3150], motor_current_ma=1575, adc_raw=[2450, 3900]))
        for change in (dict(motor_current_ma=231), dict(motor_current_ma=229), dict(motor_current_ma=230.0),
                       dict(motor_current_ma=None), dict(motor_current_ma=-1), dict(motor_current_ma=True),
                       dict(adc_pin_mv=[600, 3301], motor_current_ma=1650), dict(adc_pin_mv=[600, -2], motor_current_ma=-1),
                       dict(adc_raw=[2450, 4096]), dict(adc_raw=[4095, 570]),  # VBAT channel never saturates validly
                       dict(adc_raw=[2450, 4095]),  # all-full-scale average without the flag
                       dict(adc_pin_mv=[600, 3150], motor_current_ma=1575),  # at the INA180 rail without the flag
                       dict(adc_pin_mv=[600, 3300], motor_current_ma=1650),
                       dict(motor_current_saturated=None), dict(motor_current_saturated=1),
                       dict(motor_voltage_mv=5980), dict(motor_voltage_mv=0)):
            self.rejected(rails(**change), change=change)
        d = rails()
        del d["motor_current_saturated"]
        self.rejected(d, case="fresh without saturation flag")

    def test_saturation_needs_a_reachable_average(self):
        # kiss_c6_voltage.h: the flag needs one of 8 samples >= 3150 mV, so the average is >= (3150+4)//8 = 394 mV;
        # a raw average of 4095 needs >= 4 full-scale samples (3300 mV each), so the average is >= 1650 mV.
        self.check(rails(adc_pin_mv=[600, 394], motor_current_ma=197, motor_current_saturated=True))
        self.check(saturated(adc_pin_mv=[600, 1650], motor_current_ma=825))  # adc_raw[1] stays 4095
        for change in (dict(adc_pin_mv=[600, 393], motor_current_ma=196, motor_current_saturated=True),
                       dict(adc_pin_mv=[600, 0], motor_current_ma=0, motor_current_saturated=True),
                       dict(adc_raw=[2450, 4095], adc_pin_mv=[600, 1649], motor_current_ma=824,
                            motor_current_saturated=True),
                       dict(adc_raw=[2450, 4095], adc_pin_mv=[600, 0], motor_current_ma=0,
                            motor_current_saturated=True),
                       dict(adc_raw=[2450, 4095], adc_pin_mv=[600, 460], motor_current_ma=230,
                            motor_current_saturated=True)):
            self.rejected(rails(**change), change=change)
        self.assertEqual((host.KISS_SATURATED_MIN_AVERAGE_MV, host.KISS_FULL_SCALE_MIN_AVERAGE_MV), (394, 1650))

    def test_wireless_supply_metadata_when_present(self):
        self.check(rails(wireless_power_state="online", wireless_start_attempted=True))
        self.check(rails(wireless_power_state="restart", wireless_start_attempted=True))
        self.check(rails(wireless_power_state="inhibited", wireless_start_attempted=False))
        self.check(rails(previous_radio_power_reset=True))
        for change in (dict(wireless_power_state="online"), dict(wireless_start_attempted=True),
                       dict(wireless_power_state="online", wireless_start_attempted=1),
                       dict(wireless_power_state="online", wireless_start_attempted=False),
                       dict(wireless_power_state="qualifying", wireless_start_attempted=True),
                       dict(wireless_power_state=[], wireless_start_attempted=False),
                       dict(wireless_power_state="guaranteed", wireless_start_attempted=True),
                       dict(previous_radio_power_reset=1), dict(previous_radio_power_reset=None)):
            self.rejected(rails(**change), change=change)
        for field in ("wireless_power_state", "wireless_start_attempted"):
            d = rails()
            del d[field]
            self.rejected(d, missing_half=field)

    def test_wrong_profile_or_claimed_hardware_qualification(self):
        for profile in (None, host.R3_PROFILE, host.PICO_PROFILE):
            with self.assertRaises(host.DiagnosticError):
                host.check_kiss_rail_report(rails(), profile)
        for change in (dict(profile=host.R3_PROFILE), dict(kind="cells"), dict(hardware_qualified=True),
                       dict(divider_calibrated=True), dict(motion_enabled=True), dict(read_only=False),
                       dict(protocol=1), dict(firmware="wbb-r3-review-diag")):
            self.rejected(rails(**change), change=change)

    def test_null_on_unavailable_and_no_old_sample_reuse(self):
        d = unavailable()
        self.check(d)
        self.check(unavailable(state="stale", adc_error=0, failing_channel=-1))
        self.check(unavailable(state="stopped", adc_error=0, failing_channel=-1))
        for field in ("battery_voltage_mv", "motor_current_ma", "seq", "age_ms", "span_ms"):
            self.rejected(dict(d, **{field: 0}), field=field)
            missing = dict(d)
            del missing[field]
            self.rejected(missing, missing=field)
        for value in (False, True):
            self.rejected(dict(d, motor_current_saturated=value), saturated=value)
        self.check(dict(d, motor_current_saturated=None))
        for field in ("adc_raw", "adc_pin_mv"):
            self.rejected(dict(d, **{field: [None, 0]}), field=field)
        self.rejected(dict(d, state="ready"), case="ready but not fresh")
        self.rejected(dict(d, motor_voltage_mv=5980), case="stale VMOT")

    def test_data_types_range_and_pair_consistency(self):
        for change in (dict(fresh=1), dict(divider_gain=True), dict(divider_gain=11), dict(adc_error=False),
                       dict(failing_channel=True), dict(failing_channel=2), dict(adc_raw=[True, 570]),
                       dict(adc_pin_mv=[600]), dict(adc_pin_mv=[1001, 460], battery_voltage_mv=13013),
                       dict(battery_voltage_mv=7800.0), dict(battery_voltage_mv=7807), dict(battery_voltage_mv=13001),
                       dict(seq=0), dict(seq=2**32), dict(age_ms=251), dict(span_ms=26), dict(age_ms=1, span_ms=2),
                       dict(state="stale"), dict(state=[]), dict(state="charging"), dict(adc_error=1),
                       dict(adc_cleanup_error=1), dict(failing_channel=0)):
            self.rejected(rails(**change), change=change)

    def test_missing_and_malformed_fields(self):
        for field in ("fresh", "state", "adc_raw", "adc_pin_mv", "battery_voltage_mv", "adc_error",
                      "adc_cleanup_error", "failing_channel", "divider_gain", "usb_power", "charge",
                      "motor_voltage_mv", "motor_current_ma", "motor_current_saturated", "read_only",
                      "hardware_qualified", "divider_calibrated", "seq", "age_ms", "span_ms"):
            d = rails()
            del d[field]
            self.rejected(d, missing=field)  # DiagnosticError, never KeyError/TypeError
        for raw in (None, [], "", 123):
            with self.assertRaises(host.DiagnosticError):
                self.check(raw)


class RailClientTests(unittest.IsolatedAsyncioTestCase):
    async def test_same_read_only_command_over_usb_and_authenticated_wireless(self):
        for wireless in (False, True):
            wire = FakeWire([envelope(kiss_status(boot=BOOT)), envelope(rails())])
            wire.wireless = wireless
            c = host.Controller(wire, KEY, host.KISS_C6_PROFILE)
            await c.hello()
            result = await c.request("rail_status")
            self.assertEqual(result["battery_voltage_mv"], 7800)
            self.assertEqual(result["motor_current_ma"], 230)
            self.assertEqual(result["charge"], "external_power")
            self.assertEqual(c.lease, 0)

    async def test_rails_stay_readable_in_the_gpio_fault_state(self):
        # The voltage monitor keeps running after a latched fault; the radio answers read-only queries.
        for wireless, discovery in ((False, kiss_fault()), (True, online_fault())):
            report = rails(usb_power=False, charge="battery", wireless_power_state="online",
                           wireless_start_attempted=True)
            wire = FakeWire([envelope(discovery), envelope(report)])
            wire.wireless = wireless
            c = host.Controller(wire, KEY, host.KISS_C6_PROFILE)
            self.assertEqual((await c.hello())["reason"], "GPIO0_SAFE_LOW_FAILED")
            self.assertEqual((await c.request("rail_status"))["charge"], "battery")

    async def test_changed_board_rejected(self):
        wire = FakeWire([envelope(kiss_status(boot=BOOT)), envelope(rails(device="another-board"))])
        c = host.Controller(wire, KEY, host.KISS_C6_PROFILE)
        await c.hello()
        with self.assertRaises(host.DiagnosticError):
            await c.request("rail_status")

    async def test_wrong_selected_profile_rejected_before_transport_write(self):
        wire = FakeWire([])
        c = host.Controller(wire, KEY, host.R3_PROFILE)
        c.profile = host.R3_PROFILE
        with self.assertRaises(host.CommandRejected):
            await c.request("rail_status")
        self.assertEqual(wire.sent, [])


if __name__ == "__main__":
    unittest.main()
