#!/usr/bin/env python3
"""ArduinoTec-Pedals setup tool. GPL-3.0; see the accompanying LICENSE."""

import argparse
import sys
import time
from contextlib import contextmanager


class ProtocolError(RuntimeError):
    pass


def fields(line):
    return dict(part.split("=", 1) for part in line.split() if "=" in part)


class Pedals:
    def __init__(self, port, timeout=4):
        self.port = port
        self.timeout = timeout

    def command(self, command):
        self.port.write((command + "\n").encode("ascii"))
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            line = self.port.readline().decode("ascii", errors="replace").strip()
            if not line or line == "OK WAIT":
                continue
            if line.startswith("ERR "):
                raise ProtocolError(line[4:])
            if line.startswith("OK "):
                return line
            raise ProtocolError(f"Unexpected response: {line}")
        raise ProtocolError("No response. Check the port, firmware, and USB cable.")

    def identify(self):
        info = fields(self.command("INFO"))
        if info.get("protocol") != "1":
            raise ProtocolError("This tool needs ArduinoTec-Pedals protocol 1 firmware.")
        return info


def print_info(info, write=print):
    write(f"Firmware {info['firmware']} | calibration: "
          f"{'saved' if info['calibrated'] == '1' else 'needed'} | "
          f"brake: {'ready' if info['brake_ok'] == '1' else 'not ready'}")
    write(f"USB report target: {info['target_hz']} Hz (not the brake sample rate).")


def calibration(device, deadzone=5, ask=input, write=print):
    info = device.identify()
    print_info(info, write)
    if info["brake_ok"] != "1":
        raise ProtocolError("Brake is not ready. Wait two seconds after power-on; "
                            "if it persists, check HX711 power, DT and SCK.")
    write("Close your game. Outputs stay at zero during calibration.")
    write("Cancel retains saved settings; an interrupted save retains a complete record.")
    begun = False
    try:
        device.command("CAL BEGIN")
        begun = True
        ask("Release ALL pedals, keep your feet clear, then press Enter: ")
        device.command("CAL REST")
        axes = ["throttle"]
        if info.get("clutch") == "1":
            axes.append("clutch")
        axes.append("brake")
        for axis in axes:
            if axis == "brake":
                ask("Hold the brake at the pressure you want to mean 100%, "
                    "then press Enter and keep holding: ")
            else:
                ask(f"Hold {axis} fully pressed, then press Enter and keep holding: ")
            device.command(f"CAL FULL {axis}")
            device.command(f"CAL DZ {axis} {deadzone} {deadzone}")
            write(f"Captured {axis}; you can release it.")
        result = fields(device.command("CAL SHOW"))
        rest = result["rest"].split(",")
        full = result["full"].split(",")
        write("\nReview (raw sensor counts):")
        for i, axis in enumerate(("throttle", "clutch", "brake")):
            if axis not in axes:
                continue
            direction = "increasing" if int(full[i]) > int(rest[i]) else "decreasing"
            write(f"  {axis:8} released {rest[i]:>8}  full {full[i]:>8}  {direction}")
        write(f"Deadzone at each end: {deadzone / 10:g}%")
        if ask("Release all pedals. Type SAVE to store, or Enter to cancel: ").strip().upper() != "SAVE":
            write("Cancelled; saved settings retained.")
            return False
        device.command("CAL SAVE")
        begun = False
        write("Saved on the pedals. Unplug/replug, then verify released/full travel "
              "in your game. Rebind axes if needed.")
        return True
    finally:
        if begun:
            try:
                device.command("CAL CANCEL")
            except (ProtocolError, OSError):
                write("Could not confirm cancellation. Disconnect USB before opening "
                      "your game, then check which calibration is saved.")


def monitor(device, seconds):
    print_info(device.identify())
    print("Raw: throttle, clutch, brake | Output: 0–32767 | Brake health")
    start = time.monotonic()
    first = fields(device.command("READ"))
    last = first
    while time.monotonic() - start < seconds:
        last = fields(device.command("READ"))
        print(f"raw={last['raw']:>24} out={last['out']:>18} "
              f"brake={'OK' if last['brake_ok'] == '1' else 'FAULT'} age={last['age_ms']}ms")
        time.sleep(0.1)
    elapsed = (int(last["uptime_ms"]) - int(first["uptime_ms"])) & 0xFFFFFFFF
    if elapsed:
        print("\nFirmware counters over this observation:")
        for name, label in (("reports", "USB reports submitted"),
                            ("hall_samples", "Hall sample pairs"),
                            ("brake_samples", "Valid brake conversions")):
            count = (int(last[name]) - int(first[name])) & 0xFFFFFFFF
            print(f"  {label}: {count * 1000 / elapsed:.1f} Hz")
        print("These are firmware rates, not a host USB latency measurement. "
              "This monitor also adds serial traffic.")


@contextmanager
def connect(serial_module, port):
    # 57600 avoids the ATmega32u4's 1200-baud bootloader trigger.
    with serial_module.Serial(port, 57600, timeout=0.2, write_timeout=2) as connection:
        time.sleep(2.2)
        connection.reset_input_buffer()
        yield Pedals(connection)


def deadzone_percent(text):
    value = float(text)
    if not 0 <= value <= 10:
        raise argparse.ArgumentTypeError("Use a percentage from 0 to 10.")
    return round(value * 10)


def main(argv=None):
    parser = argparse.ArgumentParser(description="Set up ArduinoTec-Pedals over USB.")
    sub = parser.add_subparsers(dest="action", required=True)
    sub.add_parser("ports", help="List serial ports; no device is modified")
    for action, help_text in (("info", "Show firmware and calibration status"),
                              ("calibrate", "Guided calibration saved on the pedals"),
                              ("monitor", "Show sensor values and observed firmware rates"),
                              ("tare", "Temporarily zero a released brake")):
        command = sub.add_parser(action, help=help_text)
        command.add_argument("--port", required=True, help="e.g. /dev/ttyACM0 or COM4")
        if action == "calibrate":
            command.add_argument("--deadzone-percent", type=deadzone_percent, default=5,
                                 help="Deadzone at EACH endpoint, 0–10%% (default: 0.5%%)")
        if action == "monitor":
            command.add_argument("--seconds", type=int, default=10)
    args = parser.parse_args(argv)
    try:
        import serial
        from serial.tools import list_ports
    except ImportError:
        parser.exit(1, "Install the setup tool dependency first:\n"
                       "  python -m pip install -r tools/requirements.txt\n")
    try:
        if args.action == "ports":
            for port in list_ports.comports():
                print(f"{port.device}: {port.description}")
            return 0
        with connect(serial, args.port) as device:
            if args.action == "calibrate":
                calibration(device, args.deadzone_percent)
            elif args.action == "monitor":
                if args.seconds < 1:
                    parser.error("--seconds must be positive")
                monitor(device, args.seconds)
            elif args.action == "tare":
                device.identify()
                input("Release the brake, then press Enter: ")
                device.command("TARE")
                print("Brake zero updated for this session. Full calibration saves across power cycles.")
            else:
                print_info(device.identify())
        return 0
    except (ProtocolError, serial.SerialException, OSError) as error:
        print(f"Setup failed: {error}", file=sys.stderr)
        return 1
    except (KeyboardInterrupt, EOFError):
        print("\nSetup interrupted.", file=sys.stderr)
        return 130


if __name__ == "__main__":
    sys.exit(main())
