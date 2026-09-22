# ArduinoTec-Pedals
This project uses an Arduino Leonardo, Micro or compatible Pro Micro to replace the controller board of the Fanatec ClubSport Pedals V1/V2.

See the [Circuit diagram](./Circuit.png) for the wiring layout.

![Circuit diagram](./Circuit.png)

## Install dependencies

1. Install [Arduino IDE](https://www.arduino.cc/en/software)
2. Install the [Joystick Library](https://github.com/MHeironimus/ArduinoJoystickLibrary) by downloading the code as zip and in the Arduino IDE click on Sketch -> Include Library -> Add .ZIP Library.
3. Install the [HX711 Library](https://github.com/olkal/HX711_ADC) by installing HX711_ADC from the Tools -> Manage Libraries in the Arduino IDE.

## Hardware dependencies

In addition to this you would need:

1. [Arduino Leonardo](https://store.arduino.cc/usa/leonardo) or [Pro Micro 5V 16mhz](https://www.sparkfun.com/products/12640).
2. A small breadboard to place it on if you don't want to solder the cables onto the board directly
3. Connectors if you want to be able to connect the load cells and HE sensors via a connector block
4. A HX711 chip for the load cell brake, the load cell, and two 1k ohm resistors.
5. I re-used the Fanatec Hall Effect sensors as mine were still in good working order. If you do want to use another sensor, ensure it is a linear sensor with Analog output that can sense the distance of the magnet/and field variation. Do not use arduino ones as they one sense on/off and not field variation.

Connect the items as per the diagram, write the code to the Arduino, plug into the USB, calibrate via driver calibration screen, and you are ready to race.

## Tuning notes

1. The fixed position of the stock magnet may make it difficult to determine where to place the replacement Hall Effect sensor. Another magnet will confirm the HE sensor is working as expected. Depending on the sensor used it may help to rotate it 90 degrees.
2. When determining the location of the HE sensor it helps to use reusable adhesive (e.g. Blu-Tack:registered:) to temporarily affix it. Depending on sensor packaging it should be electrically isolated to avoid contact with the metal pedal. This could be done with cardboard or some other material.
3. Alternate magnet and HE sensor locations can be considered. For example: a magnet moved to the bottom of the pedal and HE sensor affixed to the frame. This would be similar to how the Thrustmaster T-LCM pedals work. Another approach would be to place magnet and HE sensor so they are inline at either end of the compression spring.

## Opening the sketch

Download or clone the entire repository. Open
`ArduinoTec-Pedals/ArduinoTec-Pedals.ino` in the Arduino IDE; keep
`confOptions.h` and `pedalMath.h` in that same folder. Copying the `.ino` alone
will fail to compile. Select the board and port that match your hardware.

The supplied Joystick library targets supported USB-capable Arduino boards;
this sketch targets ATmega32u4 boards (Leonardo/Micro and compatible Pro Micro).
Teensy requires a different USB joystick implementation and is not supported
by this sketch. See the [Joystick library's board support](https://github.com/MHeironimus/ArduinoJoystickLibrary#features).

## Calibration and Linux

Keep all pedals released during startup: the firmware measures the Hall sensor
rest positions and tares the brake. Slowly exercise each pedal through its full
travel before calibrating in a game. The firmware tracks the largest travel
seen since startup for upper deadzone clipping; it does **not** scale that
travel to the full USB range or save calibration across power cycles.

The brake sensitivity potentiometer reduces the reported brake value. Set it
before doing game calibration. The serial monitor at 57600 baud accepts `t`
to tare the released brake and replies `Tare complete` when finished. There
are no commands yet for setting endpoints or saving settings. Leave debug
printing disabled when racing: it currently adds delays to the sampling loop.

On Linux, distinguish the two input interfaces:

- `/dev/input/event*` (evdev): use `evtest` to inspect pedal events and
  `evdev-joystick` to inspect/set axis limits, flat and fuzz. Run
  `evdev-joystick --listdevs`, then `evdev-joystick --showcal DEVICE`, replacing
  `DEVICE` with the pedal device path. Identify it rather than assuming an
  event number. See the [command manual](https://man.archlinux.org/man/extra/linuxconsole/evdev-joystick.1.en).
- `/dev/input/js*` (legacy joydev): `jstest` and `jscal -c DEVICE` can test and
  calibrate this interface. This is a separate correction mechanism; do not
  assume it changes games using evdev. See the
  [kernel joystick API documentation](https://kernel.org/doc/html/latest/input/joydev/joystick-api.html).

[Calibrate Joystick](https://github.com/dkosmari/calibrate-joystick) is a GUI
candidate for evdev range/deadzone calibration with saved host profiles.
It has not been tested with these pedals. Games may apply their own mappings;
verify released/full travel in the actual game after host calibration.
These tools change host settings, not the HX711 tare or Arduino EEPROM.
For portable calibration stored on the pedals, see the proposal in
[REVIEW.md](REVIEW.md).

## Regression tests

Run from the repository root with a C++ compiler:

```sh
g++ -std=c++11 -Wall -Wextra -Werror -fsanitize=undefined,address \
  -Itests/stubs tests/pedals_test.cpp -o /tmp/pedals-test
/tmp/pedals-test
```

These tests run the actual sketch against simulated sensor/USB interfaces.
They cover pedal release, holding brake values between conversions, reporting
all axes together, dual-input baselines, tare dispatch and deadzone bounds.
They do not validate electrical behavior or USB enumeration on hardware.
