# Review and improvement plan — 2026-09-22

User-visible UI impact: no graphical UI changes. The reliability patch changes
pedal reports on zero/negative readings and near deadzones. Physical pedal and
game testing is still required before claiming a hardware issue resolved.

The project is a useful, small hardware prototype. Its main weakness is implicit
calibration and fragile sampling/output behavior, not a need for more classes.
Keep the sketch as orchestration; extract only shared arithmetic and persistent
configuration that can be tested without hardware.

## Reliability patch

- Always report each axis, including a sensor reading of zero. Previously the
  `val > 0` guards left old values on USB; a released brake could stay pressed,
  and a decreasing Hall sensor reaching ADC zero could stop updating.
- Retain the last brake conversion between HX711 samples. Previously a local
  variable reset to zero every loop and output depended on the guards above.
- Poll HX711 from the main loop. Previously `getData()` could read the multi-byte
  sample dataset while the interrupt modified it. The library supports polling.
- Clamp negative brake readings to zero and bound positive float-to-int
  conversion before applying the sensitivity pot and output limits.
- Share deadzone clipping across axes, apply the brake lower deadzone after
  sensitivity adjustment, and prevent a negative ceiling during startup.
- Use the same dual-input averaging for baseline capture and live readings.
- Send one complete USB report per loop instead of a report per setter.

## Remaining work, in priority order

1. **Explicit, persistent calibration.** Startup baselines require released
   pedals, use a maximum of 25 samples, and cannot distinguish noise from rest.
   Continuously learned maxima move when an outlier arrives and only clip;
   they do not map physical full travel to USB full scale. Store released and
   fully pressed endpoints, direction, deadzones, and desired brake force.
   Normalize each calibrated axis to 0–1023 with 32-bit intermediate math;
   reject endpoints with insufficient span. Avoid `abs()` treating movement
   in the wrong direction as valid pressure.
2. **Sensor health and noise.** A disconnected HX711 can retain its last brake
   value indefinitely, and startup failure loops forever. Add a timestamp and
   explicit fault status, decide a timeout from measured 10/80 SPS behavior,
   and report a neutral brake after timeout. Handle a faulted Hall input where
   detectable. Measure idle noise first, then choose a small filter with a
   measured latency budget. `setSamplesInUse(1)` requests minimal averaging;
   do not claim increasing it cures wiring or power faults.
3. **Predictable timing.** Remove debug `delay(100)` calls and dynamic `String`
   construction; rate-limit diagnostics without blocking. Establish a USB
   reporting cadence independently of sensor conversion timing. Measure timing
   on hardware with debug on/off, including USB host disconnection.
4. **Compatibility and repeatable builds.** Check Windows enumeration before
   changing `JOYSTICK_TYPE_MULTI_AXIS`; upstream #6 reports improvement with
   `JOYSTICK_TYPE_JOYSTICK`. Preserve existing axis usage mappings unless game
   tests justify changing them. Add CI builds with pinned AVR core/libraries
   and host regression tests. Remove or implement the unused `use_Dual_Brk`
   option. Confirm each board's physical pins against the schematic.

## Calibration design proposal (not implemented)

Keep standard USB HID for driving: games should need no custom driver. Reuse
USB serial, already present, for setup. Start with a tiny line-based protocol,
not a custom HID driver or web application:

| Proposed command | Purpose |
| --- | --- |
| `INFO` | Protocol version, firmware version, calibration/fault status |
| `READ` | Raw and normalized readings, age of last brake sample |
| `CAL BEGIN` | Start a separate candidate calibration |
| `CAL REST` | Capture stable released positions and tare brake asynchronously |
| `CAL FULL throttle`, `CAL FULL clutch`, `CAL FULL brake` | Capture full travel or desired maximum braking force |
| `CAL SHOW` | Review candidate endpoints, direction and deadzones |
| `CAL SAVE` | Validate, write EEPROM once, activate calibration |
| `CAL CANCEL` | Discard candidate and retain active calibration |

Use bounded input buffers, explicit `OK`/`ERR` replies, and completion replies
for asynchronous operations. Keep the existing `t` command compatible. Never
write EEPROM on every loop. Use versioned records with integrity checking,
defaults for invalid data, and two slots so a power interruption during save
leaves the previous record available. Define safe behavior while calibrating
and reject save until all required captures are valid.

A small serial CLI can guide the user through this process on Linux, Windows
and macOS. Existing Linux evdev tools remain useful for verifying output and
host adjustment; they cannot send this proposed protocol without an extension.
Only build a custom GUI once the calibration workflow is proven on hardware.

## Upstream issues

Reviewed the four open issues in `jssting/ArduinoTec-Pedals`:

| Issue | Assessment and next action |
| --- | --- |
| [#17: sensor voltage](https://github.com/jssting/ArduinoTec-Pedals/issues/17) | Requires identification and ratings of the actual Hall sensor. A firmware change cannot establish that 5 V is safe for a sensor powered at 3.3 V by its stock board. Leave open. |
| [#16: wheelbase connection](https://github.com/jssting/ArduinoTec-Pedals/issues/16) | This sketch is a USB peripheral. Direct wheelbase support needs protocol/electrical investigation and likely another interface. Leave open. |
| [#15: input noise](https://github.com/jssting/ArduinoTec-Pedals/issues/15) | Need per-axis raw recordings, sensor details and sampling/filter measurements. The sampling fixes here are not proof that the reporter's Hall noise is resolved. Leave open. |
| [#13: upload/build help](https://github.com/jssting/ArduinoTec-Pedals/issues/13) | Existing reply suspects a missing `confOptions.h`. README now explicitly requires the complete sketch folder. Reporter confirmation is needed to call it resolved. |

Closed issues also expose useful documentation gaps: #12 documents Teensy
incompatibility with this joystick implementation (README corrected), and #6
reports Windows joystick recognition problems (needs device testing).

No upstream closure or resolution claim is warranted from software tests alone.

## Validation

- Host regression executable passed with GCC, `-Wall -Wextra -Werror`, address
  sanitizer and undefined-behavior sanitizer.
- Arduino CLI 1.5.1 builds passed for `arduino:avr:leonardo` and
  `arduino:avr:micro`, using Arduino AVR core 1.8.8, HX711_ADC 1.2.12 and
  ArduinoJoystickLibrary v2.1.1. Usage: approximately 16 KB flash and 801 bytes
  global RAM. A Micro build is not a physical Pro Micro validation.
- Pre-commit formatting and general file checks passed. Its cppcheck and
  cpplint hooks could not run because those executables are absent locally.
- No firmware was flashed; electrical operation, pedal feel, USB enumeration,
  Linux calibration tools and behavior in games have not been tested.

To reproduce the board builds after installing the versions above:

```sh
arduino-cli compile --fqbn arduino:avr:leonardo ArduinoTec-Pedals
arduino-cli compile --fqbn arduino:avr:micro ArduinoTec-Pedals
```
