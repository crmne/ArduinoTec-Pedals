"""Exercise the real Python wizard against the compiled firmware state machine."""
import io
import os
import select
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

from test_setup_tool import pedals


class ProcessPort:
    def __init__(self, process):
        self.process = process

    def write(self, data):
        self.process.stdin.write(data)
        self.process.stdin.flush()

    def readline(self):
        ready, _, _ = select.select([self.process.stdout], [], [], 2)
        return self.process.stdout.readline() if ready else b""


@unittest.skipUnless(os.name == "posix" and shutil.which("g++"), "requires POSIX and g++")
class FirmwareToolTest(unittest.TestCase):
    def test_full_wizard_and_saved_values(self):
        root = Path(__file__).parents[1]
        with tempfile.TemporaryDirectory() as directory:
            executable = Path(directory) / "simulator"
            subprocess.run(["g++", "-std=c++11", "-Wall", "-Wextra", "-Werror",
                            "-Itests/stubs", "tests/simulator.cpp", "-o", str(executable)],
                           cwd=root, check=True, timeout=30)
            process = subprocess.Popen([str(executable)], stdin=subprocess.PIPE,
                                       stdout=subprocess.PIPE, bufsize=0)
            try:
                device = pedals.Pedals(ProcessPort(process))
                def answer(prompt):
                    if "throttle fully" in prompt:
                        device.command("@INPUT 900 900 8000000")
                    elif "clutch fully" in prompt:
                        device.command("@INPUT 100 100 8000000")
                    elif "pressure you want" in prompt:
                        device.command("@INPUT 100 900 8500000")
                    elif "Type SAVE" in prompt:
                        device.command("@INPUT 100 900 8000000")
                        return "SAVE"
                    return ""
                output = io.StringIO()
                self.assertTrue(pedals.calibration(device, ask=answer,
                                                  write=lambda line: print(line, file=output)))
                self.assertEqual(device.identify()["calibrated"], "1")
                self.assertEqual(pedals.fields(device.command("READ"))["out"], "0,0,0")
                device.command("@INPUT 900 100 8500000")
                self.assertEqual(pedals.fields(device.command("READ"))["out"], "32767,32767,32767")
                self.assertIn("Saved on the pedals", output.getvalue())
            finally:
                process.stdin.close()
                process.wait(timeout=5)
                process.stdout.close()
