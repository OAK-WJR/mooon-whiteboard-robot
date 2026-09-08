import copy
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
import pcb_control as recovery
from test_host import status


def port(path="/dev/first", serial="board-A", location="1-2", vid=0x303a, pid=0x1001):
    return SimpleNamespace(device=path, serial_number=serial, location=location, vid=vid, pid=pid)


class RecoveryTests(unittest.TestCase):
    def test_identity_survives_port_rename_but_rejects_wrong_board(self):
        target = recovery.identity(port())
        self.assertEqual(recovery.match_port([port("/dev/new")], target), "/dev/new")
        self.assertIsNone(recovery.match_port([port(serial="board-B")], target))
        with self.assertRaises(recovery.DiagnosticError):
            recovery.match_port([port(), port("/dev/duplicate")], target)

    def test_location_fallback_and_path_alone_is_not_identity(self):
        self.assertEqual(recovery.match_port([port("/dev/new", serial=None)], recovery.identity(port(serial=None))), "/dev/new")
        with self.assertRaises(recovery.DiagnosticError):
            recovery.match_port([port()], recovery.identity(port(serial=None, location=None)))

    def test_selection_requires_unique_native_usb(self):
        with self.assertRaises(recovery.DiagnosticError):
            recovery.select_port([port(), port("/dev/second")])
        with self.assertRaises(recovery.DiagnosticError):
            recovery.select_port([port(vid=0x1a86)])
        self.assertEqual(recovery.select_port([port(),port("/dev/second")],"/dev/second").device,"/dev/second")

    def test_rom_command_never_modifies_flash(self):
        for stay in (True, False):
            command = recovery.rom_command("/dev/first", stay)
            self.assertEqual(command[-1],"read-mac")
            self.assertIn("usb-reset", command)
            self.assertIn("no-reset" if stay else "hard-reset", command)
            self.assertIn("--no-stub", command)
            self.assertFalse(any("flash" in token for token in command))

    def test_reboot_requires_new_session_and_idle_all_motors(self):
        before, after = status(), status(session=8)
        after["reset_reason"] = 3
        recovery.verify_restart(before, after, "adapter", True)
        for key, value in (("session",7),("reset_reason",1),("uptime_ms",40000),("running",True)):
            bad = dict(after, **{key:value})
            with self.assertRaises(recovery.DiagnosticError):
                recovery.verify_restart(before, bad, "adapter", True)
        bad = copy.deepcopy(after)
        bad["motors"][2]["command"] = 1
        with self.assertRaises(recovery.DiagnosticError):
            recovery.verify_restart(before, bad, "adapter", True)


if __name__ == "__main__":
    unittest.main()
