import copy
import importlib.util
import io
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("pcb_diagnostics", ROOT / "scripts/pcb_diagnostics.py")
host = importlib.util.module_from_spec(spec)
spec.loader.exec_module(host)


def status(seq=0, session=7):
    pins = [(0,1,16,17,1),(2,3,18,19,3),(4,5,20,23,2)]
    return {"type":"status", "protocol":1, "firmware":"wbb-usb-diag-1", "session":session,
            "seq":seq, "uptime_ms":1000,"profile":"adapter","reason":"POWER_ON",
            "pwm_ready":True,"armed":False,"running":False,"voltage_feedback":False,
            "current_feedback":False,"power_disconnect":False,"tx_dropped":0,
            "motors":[{"index":i,"physical":p[4],"command":0,"fi":p[0],"bi":p[1],
            "enc_a":p[2],"enc_b":p[3],"count":0,"edges_a":0,"edges_b":0,"invalid":0}
            for i,p in enumerate(pins)]}


class HostTests(unittest.TestCase):
    def test_bad_boolean_motor_record_and_pin_map(self):
        for key,value in (("running","false"),("motors",[None,None,None])):
            frame=status();frame[key]=value
            with self.assertRaises(host.DiagnosticError):host.validate_status(frame)
        frame=status();frame['motors'][0]['enc_a']=7
        with self.assertRaises(host.DiagnosticError):host.validate_status(frame)
        client=host.Client(None,io.StringIO())
        with self.assertRaises(host.DiagnosticError):
            client.ingest({'type':'ack','id':1,'ok':'false','session':7})
    def test_fragmented_and_multiple_frames(self):
        decoder = host.FrameDecoder()
        data = (json.dumps(status()) + "\n" + json.dumps(status(1)) + "\n").encode()
        out = []
        for byte in data:
            out.extend(decoder.feed(bytes([byte])))
        self.assertEqual([x["seq"] for x in out], [0, 1])
        self.assertEqual(decoder.invalid, 0)

    def test_corruption_and_overflow_recover_only_at_newline(self):
        decoder = host.FrameDecoder()
        data = b"X"*5000 + b'{"type":"status"}\n\xff\n[]\n' + json.dumps(status()).encode() + b"\n"
        self.assertEqual(decoder.feed(data), [status()])
        self.assertEqual(decoder.invalid, 3)

    def test_reset_and_sequence_integrity(self):
        client = host.Client(None, io.StringIO())
        client.ingest(status())
        client.ingest(status(3))
        self.assertEqual(client.sequence_gaps, 2)
        with self.assertRaises(host.DiagnosticError):
            client.ingest(status(4,session=99))
        with self.assertRaises(host.DiagnosticError):
            client.ingest(status(3))

    def test_uint32_sequence_wrap(self):
        client = host.Client(None, io.StringIO())
        client.ingest(status(0xFFFFFFFF))
        client.ingest(status(0))
        self.assertEqual(client.sequence_gaps, 0)

    def test_missing_channel_is_not_pass(self):
        frame = status()
        frame["motors"].pop()
        with self.assertRaises(host.DiagnosticError):
            host.validate_status(frame)
        result = host.motor_result(status(), status(1), 0)
        self.assertEqual(result[0]["state"], "REVIEW")

    def test_single_phase_and_wrong_motor(self):
        before = status()
        after = copy.deepcopy(before)
        after["motors"][0].update(edges_a=50, count=0)
        after["motors"][1].update(edges_a=25,edges_b=25,count=50)
        result = host.motor_result(before,after,0)
        self.assertEqual(result[0]["state"], "REVIEW")
        self.assertEqual(result[1]["state"], "REVIEW")

    def test_bidirectional_counts_and_noise(self):
        before,after = status(),status(1)
        after["motors"][0].update(edges_a=20,edges_b=20,count=-40)
        self.assertEqual(host.motor_result(before,after,0)[0]["state"],"PASS")
        after["motors"][0]["invalid"] = 1
        self.assertEqual(host.motor_result(before,after,0)[0]["state"],"REVIEW")


if __name__ == "__main__":
    unittest.main()
