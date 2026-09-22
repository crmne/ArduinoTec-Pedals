# Development and release process

The sketch handles hardware, scheduling and the serial protocol.
`calibration.h` contains normalization/capture rules; `calibrationStore.h`
contains a small two-slot EEPROM store with explicit byte encoding. Keep these
independent of Arduino so the behavior can be exercised on a host compiler.
`tools/pedals.py` is a thin serial client; it does not duplicate calibration math.

## Run tests

From the repository root:

```sh
g++ -std=c++11 -Wall -Wextra -Werror -fsanitize=address,undefined \
  -Itests/stubs tests/pedals_test.cpp -o /tmp/pedals-test
/tmp/pedals-test
python3 -m unittest discover -s tests -p 'test_*.py'
```

Firmware tests compile the actual sketch against simulated Arduino, HX711,
EEPROM and joystick interfaces. They cover monotonic/reversed calibration,
bounds, captures, serial framing, cancellation, stale/invalid brake readings,
rate scheduling, temporary tare, corrupt EEPROM, hardware-profile mismatch,
and simulated power loss at every byte mutation in a save. The setup tests
cover protocol responses, save/cancel, errors and interruption, and run the
actual Python wizard against a compiled firmware simulator.
These tests do not validate USB/electrical behavior or benchmark an AVR CPU.

## Build and package

Install **Arduino CLI 1.5.1**, Git and Python 3.9+. Then:

```sh
python3 tools/build.py --setup
```

This downloads pinned Arduino AVR 1.8.8, SparkFun AVR 1.1.13, HX711_ADC 1.2.12
and ArduinoJoystickLibrary v2.1.1 (verified against its commit). Dependencies
live under ignored `build/toolchain`, separate from your regular Arduino IDE.
The script uses the public Arduino/SparkFun package indexes and GitHub; initial
setup requires network access. Later builds can omit `--setup`.

The build checks installed core/library versions, compiles all three board
profiles, and writes **`build/dist/`**. It replaces that generated directory on
each run. Dependency versions and source commit are recorded in the manifest.
Pinning versions is for repeatable inputs; bit-for-bit reproducibility across
all operating systems has not been established.

The package contains:

- Application-only HEX images for Leonardo, Micro and 5 V / 16 MHz Pro Micro.
- A ZIP with those images, manifest, README, circuit drawing, guides and tools.
- A source archive with our firmware/tools/tests and the exact library/core
  source trees used by the build, including their license notices.
- SHA256SUMS for every generated asset.

The source archive can be extracted and built using `tools/build.py --setup`.
It has no Git metadata; the supplied manifest identifies its source revision.
The `vendor` directory preserves dependency sources for inspection and manual
Arduino setups. AVR compiler/tool packages are obtained through the pinned
board package indexes rather than embedded in the source archive.

## Configuration and compatibility

Edit `confOptions.h` only when building from source. `CLUTCH_ENABLED=false`
allows two pedals; `BRAKE_POT_ENABLED=true` restores post-calibration sensitivity
scaling from A3. Dual-input options average paired compatible sensor outputs.
These options contribute to the EEPROM profile key. Increment
`CALIBRATION_PROFILE` when changing pin assignments or raw sensor processing
so old endpoints cannot be silently reused. Do not simply increase the report
rate constant expecting the HX711 to sample faster.

The joystick USB type is now `JOYSTICK_TYPE_JOYSTICK`. Axis usages remain Rx,
throttle and brake. Host/game re-enumeration and re-binding may be necessary.
Changing hardware options requires firmware upload and recalibration; the
release binaries intentionally use one documented default configuration.

## Release CI

Pushes to `master`, pull requests and manual runs execute tests and build all
three boards. The resulting `pedals-firmware` Actions artifact is suitable for
preview testing, with its validation limitations clearly stated. On version
tags, the same workflow additionally verifies:

1. The tag format is `vX.Y.Z`, `vX.Y.Z-rc.N` or `vX.Y.Z-beta.N`.
2. `PEDALS_VERSION` in `version.h` exactly matches the tag without `v`.
3. The tagged commit is an ancestor of the repository's default branch.
4. The checkout is the tagged commit and has no local modifications.

Only after successful tests/builds does the release job use write permissions
to create a **draft GitHub release** with all assets. Ordinary pushes and pull
requests have no release-write permissions.

For a release, commit the version and release notes to `master`, push it, and
run the hardware checklist. Create/push a version tag only at that reviewed
commit. CI prepares the draft; inspect its assets and hardware-validation notes
before publishing. Mark a first hardware-test candidate as a prerelease. A
passing build alone must not be presented as a tested stable hardware release.

## Hardware acceptance checklist

Record the board, sensor/cell, HX711 module/rate, wiring configuration, OS and
game versions along with results:

- [ ] Correct port and joystick enumeration on physical hardware.
- [ ] No output before calibration, during capture, or during warm-up.
- [ ] All axes reach zero/full; increasing and decreasing sensor directions work.
- [ ] Settings survive a power cycle; cancellation preserves the saved record.
- [ ] Unplugged/stalled HX711 releases brake after timeout and recovers sensibly.
- [ ] Idle/fixed-pressure noise is acceptable and no axis sticks after release.
- [ ] Observed USB and sensor rates are recorded; no unsupported latency claims.
- [ ] Serial monitor disconnect and USB suspend/resume behave acceptably.
- [ ] Target games bind all axes and apply expected brake pressure.

## Project identity

Keeping the GitHub fork relationship does not prevent independent releases or
maintenance. Preserve original attribution, history and license. GitHub's
[detach documentation](https://docs.github.com/en/pull-requests/how-tos/work-with-forks/detaching-a-fork)
states that detachment is permanent and lists metadata that is not retained.
Decide on detachment after a hardware-tested release, with an inventory/backup
of repository metadata. No detachment or repository deletion is performed by
these scripts.
