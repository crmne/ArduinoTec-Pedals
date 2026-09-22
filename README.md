# ArduinoTec-Pedals

Turn an ATmega32u4 board, two analog pedal sensors and an HX711 load-cell
amplifier into standalone USB racing pedals. Originally made for Fanatec
ClubSport V1/V2 pedals; other pedals can work with compatible sensors and wiring.
The pedals connect to your **computer by USB**, separately from your wheelbase.

**Development preview:** the new calibration firmware builds for all three
boards below and passes simulated regression tests. It has not yet been tested
on physical pedals. Do the checks below before using it in a race. No published
release is implied by this README.

This version adds saved calibration, automatic sensor direction detection,
a guided setup tool, a 1,000 Hz report target, brake fault detection, and builds
with pinned dependencies. Hardware limits still apply: the HX711 supplies
**10 or 80 new brake readings per second**, not 1,000.

## What you need

| Part | Requirement |
| --- | --- |
| Controller | Arduino Leonardo, Arduino Micro, or ATmega32u4 Pro Micro **5 V / 16 MHz** |
| Throttle and clutch | Analog sensors: stock compatible Hall sensors or potentiometers |
| Brake | Load cell connected to an HX711 amplifier, channel A |
| Wiring | USB data cable, connectors, insulated wire, mounting/strain relief |
| Tools | Multimeter; soldering iron, solder and heat shrink for permanent assembly |
| Computer | Arduino IDE for installation; Python 3.9+ for the guided setup tool |

Uno/Nano with ATmega328P, Teensy, RP2040, and 3.3 V / 8 MHz Pro Micro boards are
not targets of the supplied binaries. A board with a similar shape is not
necessarily compatible. Two-pedal builds are possible from source by setting
`CLUTCH_ENABLED` to `false`; downloadable defaults expect all three pedals.

The original Hall sensors' voltage rating must be established before powering
them. A 5 V controller does **not** mean an unknown sensor tolerates 5 V.
See [wiring and assembly](docs/wiring.md) for the pin table, load-cell types,
checks before connecting USB, and the original circuit drawing.

## 1. Wire and check the pedals

Follow [the wiring guide](docs/wiring.md). Prototype and verify sensor readings
before making permanent joints. Disconnect the original controller from any
sensors you reconnect here. Do not connect this USB controller to the wheelbase
pedal socket.

Default signals are throttle **A0**, clutch **A2**, HX711 **DT → D3** and
**SCK → D5**, with a common ground. Use the labels printed on your specific
board and module. The old brake sensitivity potentiometer is optional and
ignored by default: calibration now sets the pressure that means full braking.

## 2. Install the firmware

### Arduino IDE: the easiest starting point

1. Download this repository using **Code → Download ZIP**, then extract it.
2. Install [Arduino IDE](https://www.arduino.cc/en/software). In Boards Manager,
   install **Arduino AVR Boards 1.8.8**. For a Pro Micro, also install
   **SparkFun AVR Boards 1.1.13** using the additional board URL below.
3. In Library Manager, install **HX711_ADC 1.2.12** by Olav Kallhovd.
4. Download [ArduinoJoystickLibrary v2.1.1](https://github.com/MHeironimus/ArduinoJoystickLibrary/archive/refs/tags/v2.1.1.zip).
   In the IDE choose **Sketch → Include Library → Add .ZIP Library**.
5. Open `ArduinoTec-Pedals/ArduinoTec-Pedals.ino`. Keep **all files in that
   folder together**; downloading just the `.ino` does not work.
6. Select your board and USB port. For SparkFun Pro Micro, select
   **ATmega32U4 (5V, 16 MHz)** under Processor. Click **Upload**.
7. Close the IDE's Serial Monitor before using the setup tool.

SparkFun's additional Boards Manager URL (IDE Settings/Preferences):

```text
https://raw.githubusercontent.com/sparkfun/Arduino_Boards/master/IDE_Board_Manager/package_sparkfun_index.json
```

No pedal motion is reported until you complete and save calibration.

### Prebuilt firmware: no compilation

When available, download the `pedals-VERSION.zip` asset from
[Releases](https://github.com/crmne/ArduinoTec-Pedals/releases), or a development
artifact from [Actions](https://github.com/crmne/ArduinoTec-Pedals/actions).
Extract it. The package includes three board-specific HEX files, this guide,
and the setup tool. [The prebuilt installation guide](docs/install-prebuilt.md)
shows how to choose and upload the correct file without replacing your bootloader.

## 3. Calibrate once

Open a terminal in the extracted project/package folder. Create a Python
environment and install the small serial dependency:

```sh
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r tools/requirements.txt
python tools/pedals.py ports
```

On Windows, use `py -3 -m venv .venv`, then `.venv\Scripts\activate.bat` in
Command Prompt (or `.venv\Scripts\Activate.ps1` in PowerShell). The remaining
`python` commands are the same. On macOS, the port is typically
`/dev/cu.usbmodem…`; on Linux it is usually `/dev/ttyACM…`; Windows uses `COM…`.
Choose the port that appears when you plug in the pedals.

Replace the example port with yours:

```sh
python tools/pedals.py calibrate --port /dev/ttyACM0
```

The tool guides you through:

1. Release every pedal and keep your feet clear.
2. Hold throttle fully pressed, then clutch fully pressed.
3. Hold the brake at the pressure you want to represent **100% braking**.
4. Keep still during each capture (about 0.8 seconds).
5. Review the captured endpoints, release everything, and type **SAVE**.

Calibration is stored in the controller's EEPROM and survives power cycles.
It supports sensors whose readings increase or decrease when pressed. Default
deadzones are 0.5% at each end; `--deadzone-percent 1` sets 1% instead.
The brake capture sets a personal force preference; it is not a measurement
of kilograms without separate mechanical/load-cell calibration.

Press Ctrl+C or decline SAVE to cancel. Previous saved settings are retained.
An abandoned session expires after two minutes without a command. Keep games
closed during setup: output resumes after save, cancel or expiry. A subsequent
capture of the released position requires recapturing all full endpoints.

## 4. Check, then drive

```sh
python tools/pedals.py info --port /dev/ttyACM0
python tools/pedals.py monitor --port /dev/ttyACM0 --seconds 10
```

Check that every released pedal reports zero, each pressed pedal reaches 32767
in this tool, and the others remain at zero. Unplug/replug USB and verify the
calibration remains. The USB library maps this range into 16-bit HID fields;
your game or OS may display a different number for the same full travel.

In your game, bind throttle, brake and clutch separately. This firmware uses
the throttle, brake and Rx (clutch) axes. Clear old game calibration and rebind
if upgrading from the original firmware. The new USB device type is joystick;
Windows/Linux enumeration and individual games still need physical validation.

Linux tools such as `evtest` or `evdev-joystick` can inspect the event device.
`jscal` calibrates the separate legacy `/dev/input/js*` interface. They cannot
save settings on this controller; use the supplied tool for that. See
[calibration protocol and Linux notes](docs/calibration.md).

For a small brake-zero drift after warming up, release it and run:

```sh
python tools/pedals.py tare --port /dev/ttyACM0
```

This adjusts only the current session's brake zero while retaining its travel.
For a lasting change, run the full calibration again. The firmware never
silently tares a pressed brake when plugged in.

## Performance: what the numbers mean

| Part of the system | Capability / limit |
| --- | --- |
| Firmware report schedule | Target 1,000 reports/s, one complete report per interval |
| USB endpoint | Requests 1 ms polling; delivery depends on the host |
| Hall sensors | 10-bit ADC: up to 1,024 raw counts across the entire input voltage range |
| Brake ADC | HX711: 24-bit raw output, **10 or 80 samples/s** selected in hardware |
| Calibrated output | 0–32767 (32,768 possible values), mapped into 16-bit USB fields |
| Actual precision / end-to-end latency | Not measured yet; depends on mechanics, signal span, noise, filtering and host |

`monitor` reports observed firmware submission/sampling rates. Repeating the
latest brake reading at 1,000 Hz does not create new brake measurements.
80 Hz gives a new conversion about every 12.5 ms; 10 Hz gives one about every
100 ms. The HX711 and library filters add delay. The default library's filter
still uses three conversions even with `setSamplesInUse(1)`.

See [performance and measurement](docs/performance.md) for 80 Hz setup, the
filter tradeoff, and a validation checklist. We keep your existing hardware;
software cannot remove the HX711's conversion-rate limit.

## Troubleshooting

| Symptom | What to check |
| --- | --- |
| No serial port | Use a USB **data** cable, check the board selection and OS port permissions. Unplug/replug to identify it. |
| Port busy / access denied | Close Serial Monitor and other tools. On Linux inspect the device's group and follow your distribution's serial-access instructions; do not run the whole setup as root. |
| `BRAKE_NOT_READY` | Wait two seconds after power-on. Check HX711 power, ground, DT/D3 and SCK/D5; a stalled or saturated input is rejected. |
| `UNSTABLE_HOLD_STILL` | Repeat the capture holding the pedal still. Inspect loose wiring, sensor mounting and raw noise. The tool cancels the session on an error. |
| `TRAVEL_TOO_SMALL` | Check sensor/magnet movement or load-cell wiring; released and full readings are too close. |
| Pedals remain at zero | Run `info`; calibration may be missing or invalid. Check that the firmware configuration matches the saved calibration. |
| Brake falls to zero during use | No valid conversion arrived for 250 ms, or the input is invalid/saturated. Inspect the raw readings and wiring. |
| Upload fails | Select the exact board/clock. Pro Micro bootloaders may need reset/double-reset to expose a temporary upload port; see the board maker's instructions. |
| Missing header / wrong Joystick library | Keep the entire sketch folder and install the named library/version above; several unrelated libraries have similar names. |

## Development and releases

[Developer instructions](docs/development.md) cover tests, the pinned build
script, package contents and releases. Pushes and pull requests run simulated
firmware tests, setup-tool tests and builds for Leonardo, Micro and Pro Micro.
Version tags must match the firmware version and be reachable from `master`;
CI prepares a **draft release** with binaries, sources and checksums for review.

This project builds on [jssting/ArduinoTec-Pedals](https://github.com/jssting/ArduinoTec-Pedals).
The September 2026 firmware/setup changes are maintained here. The original
history, circuit assets and [GPL-3.0 license](LICENSE) are retained.
