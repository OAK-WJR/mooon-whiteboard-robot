"""Offline rail-report parser/client checks; every transport is a fake."""
import copy
import unittest
from test_host import BOOT, KEY, FakeWire, envelope
from test_ch343g_host import status as kiss_status
import robot_control as host


def rails(**changes):
    d=dict(id=1,ok=True,error="",boot=BOOT,lease=0,protocol=2,firmware="wbb-control-2",
           profile=host.KISS_C6_PROFILE,kind="rails",device=kiss_status()["device"],
           state="ready",adc_error=0,adc_cleanup_error=0,failing_channel=-1,
           fresh=True,read_only=True,motion_enabled=False,hardware_qualified=False,
           divider_calibrated=False,divider_gain=13,adc_raw=[2000,1500],adc_pin_mv=[600,460],
           battery_voltage_mv=7800,motor_voltage_mv=5980,seq=1,age_ms=2,span_ms=0)
    d.update(changes);return d


class RailReportTests(unittest.TestCase):
    def check(self,d):return host.check_kiss_rail_report(d,host.KISS_C6_PROFILE)

    def test_fresh_and_zero_samples(self):
        self.check(rails())
        self.check(rails(adc_raw=[0,0],adc_pin_mv=[0,0],battery_voltage_mv=0,motor_voltage_mv=0))

    def test_wireless_supply_metadata_when_present(self):
        self.check(rails(wireless_power_state="online",wireless_start_attempted=True))
        self.check(rails(wireless_power_state="qualifying",wireless_start_attempted=False))
        self.check(rails(previous_radio_power_reset=True))
        for change in (dict(wireless_power_state="online"),dict(wireless_start_attempted=True),
                       dict(wireless_power_state="online",wireless_start_attempted=1),
                       dict(wireless_power_state="online",wireless_start_attempted=False),
                       dict(wireless_power_state="qualifying",wireless_start_attempted=True),
                       dict(wireless_power_state=[],wireless_start_attempted=False),
                       dict(wireless_power_state="guaranteed",wireless_start_attempted=True),
                       dict(previous_radio_power_reset=1),dict(previous_radio_power_reset=None)):
            with self.subTest(change=change),self.assertRaises(host.DiagnosticError):self.check(rails(**change))

    def test_wrong_profile_or_claimed_hardware_qualification(self):
        for profile in (None,host.R3_PROFILE,host.PICO_PROFILE):
            with self.assertRaises(host.DiagnosticError):host.check_kiss_rail_report(rails(),profile)
        for change in (dict(profile=host.R3_PROFILE),dict(kind="cells"),dict(hardware_qualified=True),
                       dict(divider_calibrated=True),dict(motion_enabled=True),dict(read_only=False)):
            with self.subTest(change=change),self.assertRaises(host.DiagnosticError):self.check(rails(**change))

    def test_null_on_unavailable_and_no_old_sample_reuse(self):
        d=rails(fresh=False,state="adc_read",adc_error=263,failing_channel=1,
                adc_raw=[None,None],adc_pin_mv=[None,None],battery_voltage_mv=None,
                motor_voltage_mv=None,seq=None,age_ms=None,span_ms=None)
        self.check(d)
        for field in ("battery_voltage_mv","motor_voltage_mv","seq","age_ms","span_ms"):
            with self.subTest(field=field),self.assertRaises(host.DiagnosticError):self.check(dict(d,**{field:0}))
        for field in ("adc_raw","adc_pin_mv"):
            with self.assertRaises(host.DiagnosticError):self.check(dict(d,**{field:[None,0]}))

    def test_data_types_range_and_pair_consistency(self):
        for change in (dict(fresh=1),dict(divider_gain=True),dict(adc_error=False),dict(failing_channel=True),
                       dict(adc_raw=[4095,0]),dict(adc_raw=[True,0]),dict(adc_pin_mv=[600]),
                       dict(adc_pin_mv=[1001,0]),dict(battery_voltage_mv=7800.0),dict(motor_voltage_mv=6000),
                       dict(seq=0),dict(age_ms=251),dict(span_ms=26),dict(age_ms=1,span_ms=2),
                       dict(state="stale"),dict(state=[]),dict(adc_cleanup_error=1),dict(failing_channel=0)):
            with self.subTest(change=change),self.assertRaises(host.DiagnosticError):self.check(rails(**change))

    def test_missing_and_malformed_fields(self):
        for field in ("fresh","adc_raw","adc_pin_mv","battery_voltage_mv","adc_error","divider_gain"):
            d=rails();del d[field]
            with self.subTest(field=field),self.assertRaises(host.DiagnosticError):self.check(d)
        for raw in (None,[],"",123):
            with self.assertRaises(host.DiagnosticError):self.check(raw)


class RailClientTests(unittest.IsolatedAsyncioTestCase):
    async def test_same_read_only_command_over_usb_and_authenticated_wireless(self):
        for wireless in (False,True):
            wire=FakeWire([envelope(kiss_status(boot=BOOT)),envelope(rails())]);wire.wireless=wireless
            c=host.Controller(wire,KEY,host.KISS_C6_PROFILE)
            await c.hello();result=await c.request("rail_status")
            self.assertEqual(result["battery_voltage_mv"],7800)
            self.assertEqual(c.lease,0)

    async def test_changed_board_rejected(self):
        wire=FakeWire([envelope(kiss_status(boot=BOOT)),envelope(rails(device="another-board"))])
        c=host.Controller(wire,KEY,host.KISS_C6_PROFILE);await c.hello()
        with self.assertRaises(host.DiagnosticError):await c.request("rail_status")

    async def test_wrong_selected_profile_rejected_before_transport_write(self):
        wire=FakeWire([]);c=host.Controller(wire,KEY,host.R3_PROFILE)
        c.profile=host.R3_PROFILE
        with self.assertRaises(host.CommandRejected):await c.request("rail_status")
        self.assertEqual(wire.sent,[])


if __name__=="__main__":unittest.main()
