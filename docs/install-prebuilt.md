# Install a prebuilt firmware image

A firmware HEX is the program for the controller; uploading it does not require
compiling C++. Keep the existing bootloader. These instructions use
[Arduino CLI](https://arduino.github.io/arduino-cli/1.5/installation/), available
for Linux, macOS and Windows. If you prefer buttons to terminal commands, use
the Arduino IDE instructions in [README](../README.md).

## Pick the board image

| Your board | File | Board identifier (FQBN) |
| --- | --- | --- |
| Arduino Leonardo | `pedals-leonardo.hex` | `arduino:avr:leonardo` |
| Arduino Micro | `pedals-micro.hex` | `arduino:avr:micro` |
| SparkFun-compatible Pro Micro, 5 V / 16 MHz | `pedals-promicro-5v16.hex` | `SparkFun:avr:promicro:cpu=16MHzatmega32U4` |

Do not choose Micro just because a Pro Micro is small. Board variant, USB IDs,
clock and bootloader configuration matter. The default images have a throttle,
clutch and brake; custom pins, two pedals or the optional pot require a source
build. A board's EEPROM calibration persists through normal firmware upload,
but an incompatible calibration schema/profile will require setup again.

## Upload

Extract the release ZIP. Install Arduino CLI 1.5.1 and the matching board core:

```sh
arduino-cli core update-index
arduino-cli core install arduino:avr@1.8.8
```

For a Pro Micro, also run:

```sh
arduino-cli core update-index --additional-urls https://raw.githubusercontent.com/sparkfun/Arduino_Boards/master/IDE_Board_Manager/package_sparkfun_index.json
arduino-cli core install SparkFun:avr@1.1.13 --additional-urls https://raw.githubusercontent.com/sparkfun/Arduino_Boards/master/IDE_Board_Manager/package_sparkfun_index.json
```

List connected boards:

```sh
arduino-cli board list
```

Close the game, Serial Monitor and setup tool. Replace the port and board/file
below with your selections. For a **Leonardo** on Linux:

```sh
arduino-cli upload --fqbn arduino:avr:leonardo --port /dev/ttyACM0 --input-file pedals-leonardo.hex --verify
```

For a **5 V / 16 MHz Pro Micro**, the board argument is
`--fqbn SparkFun:avr:promicro:cpu=16MHzatmega32U4` and the file is
`pedals-promicro-5v16.hex`. On Windows use the `COM` port from `board list`;
on macOS use its `/dev/cu.*` port. During reset the bootloader may enumerate
on a different port. Use the reset procedure documented for your board if
automatic reset fails, then upload while that port is present.

These are application-only images. Do not use “Burn Bootloader,” change fuses,
or erase EEPROM to install them. After uploading, return to
[step 3: calibration](../README.md#3-calibrate-once).

## Verify downloads

The release also supplies `SHA256SUMS`, covering the ZIP, source archive,
individual HEX files and manifest. Download the assets you wish to verify.
Compare a file's `sha256sum FILE` output (Linux), `shasum -a 256 FILE` (macOS),
or `Get-FileHash FILE -Algorithm SHA256` (PowerShell) against its entry.
The manifest records the source commit, board identifiers and dependency
versions; it does not certify physical hardware testing.
