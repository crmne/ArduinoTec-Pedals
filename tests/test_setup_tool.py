import importlib.util
import unittest
from pathlib import Path
from unittest.mock import patch

spec = importlib.util.spec_from_file_location("pedals", Path(__file__).parents[1] / "tools/pedals.py")
pedals = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pedals)


class FakePort:
    def __init__(self, replies):
        self.replies = iter(replies)
        self.writes = []

    def write(self, data):
        self.writes.append(data)

    def readline(self):
        return next(self.replies, b"")


class Device:
    def __init__(self, fail=None, clutch=True):
        self.commands = []
        self.fail = fail
        self.clutch = clutch

    def identify(self):
        return dict(protocol="1", firmware="test", calibrated="1", brake_ok="1",
                    target_hz="1000", clutch=str(int(self.clutch)))

    def command(self, command):
        self.commands.append(command)
        if command == self.fail:
            raise pedals.ProtocolError("UNSTABLE_HOLD_STILL")
        if command == "CAL SHOW":
            return "OK CAL rest=100,900,8000000 full=900,100,8500000 lower=5,5,5 upper=5,5,5"
        return "OK SAVED"


class SetupTests(unittest.TestCase):
    def test_async_protocol(self):
        port = FakePort([b"OK WAIT\n", b"", b"OK CAPTURE\n"])
        self.assertEqual(pedals.Pedals(port).command("CAL REST"), "OK CAPTURE")
        self.assertEqual(port.writes, [b"CAL REST\n"])

    def test_error_and_unknown_protocol(self):
        with self.assertRaisesRegex(pedals.ProtocolError, "BRAKE_NOT_READY"):
            pedals.Pedals(FakePort([b"ERR BRAKE_NOT_READY\n"])).command("CAL REST")
        with self.assertRaisesRegex(pedals.ProtocolError, "protocol 1"):
            pedals.Pedals(FakePort([b"OK INFO protocol=2\n"])).identify()

    def test_timeout(self):
        with patch.object(pedals.time, "monotonic", side_effect=[0, 5]):
            with self.assertRaisesRegex(pedals.ProtocolError, "No response"):
                pedals.Pedals(FakePort([])).command("INFO")

    def test_save(self):
        device = Device()
        answers = iter(["", "", "", "", "SAVE"])
        self.assertTrue(pedals.calibration(device, ask=lambda _: next(answers), write=lambda _: None))
        self.assertEqual(device.commands[-1], "CAL SAVE")
        self.assertNotIn("CAL CANCEL", device.commands)

    def test_cancel(self):
        device = Device(clutch=False)
        self.assertFalse(pedals.calibration(device, ask=lambda _: "", write=lambda _: None))
        self.assertNotIn("CAL FULL clutch", device.commands)
        self.assertNotIn("CAL SAVE", device.commands)
        self.assertEqual(device.commands[-1], "CAL CANCEL")

    def test_sensor_failure_cancels(self):
        device = Device(fail="CAL FULL brake")
        with self.assertRaises(pedals.ProtocolError):
            pedals.calibration(device, ask=lambda _: "", write=lambda _: None)
        self.assertEqual(device.commands[-1], "CAL CANCEL")
        self.assertNotIn("CAL SAVE", device.commands)

    def test_interrupt_cancels(self):
        device = Device()
        def interrupt(_):
            raise KeyboardInterrupt
        with self.assertRaises(KeyboardInterrupt):
            pedals.calibration(device, ask=interrupt, write=lambda _: None)
        self.assertEqual(device.commands[-1], "CAL CANCEL")


if __name__ == "__main__":
    unittest.main()
