# Calibration protocol and Linux integration

The normal gaming interface is USB HID. Configuration uses the board's USB
serial port at **57600 baud**, with ASCII commands terminated by newline.
Only one application should open the serial port at a time. There is no custom
Linux kernel driver or privileged service to install.

Protocol version **1** returns one `OK ...` or `ERR ...` line per command.
A capture returns `OK WAIT`, followed by `OK CAPTURE` or an error after about
800 ms. Wait for completion before sending another command. `INFO` identifies
the protocol; clients should reject unknown versions. Maximum command length
is 63 characters, excluding newline; overflow discards the entire line.
The retained single-byte lowercase `t` command also works without newline.

| Command | Meaning |
| --- | --- |
| `INFO` | Firmware/protocol version, saved-calibration flag, session, clutch, brake health and report target |
| `READ` | Absolute raw and normalized throttle/clutch/brake values; brake health/age; uptime and cumulative report/sample counters |
| `CAL BEGIN` | Begin a candidate calibration; outputs become neutral |
| `CAL REST` | Hold all pedals released; capture stable absolute rest readings |
| `CAL FULL throttle` | Hold throttle fully pressed and capture its endpoint |
| `CAL FULL clutch` | Hold clutch fully pressed and capture its endpoint (if enabled) |
| `CAL FULL brake` | Hold desired maximum braking force and capture it |
| `CAL DZ throttle 5 5` | Set lower/upper deadzones in parts per thousand (0–100 each); replace axis as appropriate |
| `CAL SHOW` | Show candidate calibration in a session, otherwise active calibration |
| `CAL SAVE` | Validate all required captures, save and activate |
| `CAL CANCEL` | Discard candidate, resume active settings; also cancels a temporary tare capture |
| `TARE` or `t` | Capture a new released brake zero, preserving span, until reboot; requires existing calibration |

Readings are ordered **throttle, clutch, brake**. Hall raw values are 0–1023.
Brake raw values are the HX711 library's unsigned-offset representation of the
ADC output (0–16777215), with no scale factor or automatic boot tare. They are
not kilograms. Values at the brake ADC rails are invalid. `out` is 0–32767;
zero may also mean calibration mode, warm-up or a brake fault, so inspect status.

Calibration rejects Hall spans below 32 counts and brake spans below 4096 raw
counts. Captures require at least 100 Hall samples and four brake samples;
peak-to-peak variation must be at most eight Hall counts or 1000 brake counts.
These are initial validation limits, not claims about measured sensor quality.
A reversed span is valid. Recapturing REST invalidates prior full captures.

Two EEPROM slots use a schema version, hardware-profile key, generation counter
and CRC. A save commits its target slot last and leaves the previous slot
untouched. No writes occur during normal driving or temporary tare. Calibration
is invalidated when compiled sensor options/profile change. First boot and
invalid records produce neutral output until calibration. All axes are neutral
through the initial two-second sensor warm-up and while calibrating.
A stale (250 ms) or invalid brake reading neutralizes only the brake.
An abandoned calibration expires after 120 seconds without a command.

## Reusing Linux tools

[`evdev-joystick`](https://man.archlinux.org/man/extra/linuxconsole/evdev-joystick.1.en)
can inspect/change axis limits and flat/fuzz settings for `/dev/input/event*`.
Use `--listdevs` and `--showcal DEVICE` to identify the actual pedals. `evtest`
is useful for observing events. A GUI candidate is
[Calibrate Joystick](https://github.com/dkosmari/calibrate-joystick), which saves
host profiles; it has not been tested with this hardware.

[`jscal` and `jstest`](https://kernel.org/doc/html/latest/input/joydev/joystick-api.html)
work on the separate legacy `/dev/input/js*` interface. A correction there
should not be assumed to affect a game using evdev. Games may have their own
calibration too.

These tools can verify or adjust what Linux receives. They do not implement
this serial protocol or write the pedal EEPROM. The supplied CLI is the small
piece needed for that, and a future GUI can reuse exactly the same commands.
