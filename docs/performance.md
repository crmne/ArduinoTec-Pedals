# Performance and measurement

## Three different rates

1. **USB polling:** the pinned Joystick library requests a 1 ms interrupt
   endpoint interval. That is a host polling opportunity, not proof that the
   firmware or a game processes a fresh sensor value each millisecond.
2. **Firmware sampling/reporting:** this firmware targets a 1 ms interval for
   Hall reads and one complete HID report. It skips missed intervals instead
   of replaying a backlog. Serial work is bounded and there are no debug sleeps
   or heap-allocated strings in the main loop. A stalled USB host can still
   delay the Arduino USB stack; it is not a hard real-time guarantee.
3. **Brake conversions:** the HX711's internal oscillator supports 10 or 80 SPS,
   chosen by its RATE pin. Those correspond to approximately 100 or 12.5 ms
   between fresh conversions. Faster reports reuse the latest conversion.

The earlier sketch had no explicit report schedule and sent a separate USB
report from each axis setter. Its exact rate was not measured. This version's
1,000 Hz target is a scheduling choice, not a measured speedup claim.

## Precision is not the size of a USB field

The ATmega32u4 Hall inputs have a 10-bit ADC. The theoretical maximum is 1024
levels over the full ADC voltage range; if the mechanism produces only 300
counts of travel, there are roughly 300 useful position intervals before
noise. Rescaling into more USB numbers does not create physical information.
Good magnet alignment, a useful voltage span, stable supply and solid mechanics
matter more than increasing the reported bit count.

The HX711 returns 24 bits, but those are not 24 noise-free bits of brake force.
The new firmware retains its raw readings for calibration instead of dividing
by a fixed scale factor and truncating into a small integer range. Normalization
produces 0–32767: up to 32768 distinguishable output values, subject to raw span
and noise. The Joystick library maps them into 16-bit HID fields. This input
range also avoids overflowing the library's signed 32-bit mapping intermediate.

Calibration uses a linear response with lower/upper deadzones and handles
increasing/decreasing sensors. There is no response curve, oversampling claim,
or effective-bit claim based only on the ADC specification.

## Filtering and latency

With HX711_ADC 1.2.12, `setSamplesInUse(1)` still includes its default high- and
low-outlier rejection. That makes the output a median of three conversions,
not an unfiltered sample. A monotonic transition normally gains about one
sample interval of software-filter delay. The ADC also filters internally;
the datasheet quotes 400 ms settling at 10 SPS and 50 ms at 80 SPS after a
reset/gain/channel change. Those settling figures are not interchangeable with
total measured pedal-to-game latency.

Changing to 80 SPS improves responsiveness but the datasheet specifies higher
input noise (90 nV RMS versus 50 nV RMS at gain 128). Keep the short filter until
bench measurements justify changing it. Use the module's documented RATE
configuration; there is no firmware command to overclock the HX711.

## Measure your actual pedals

Run `python tools/pedals.py monitor --port PORT --seconds 10` after calibration.
It reports changes in firmware counters divided by device uptime:

- USB reports **submitted** to the library per second.
- Hall sample pairs per second.
- Valid brake conversions per second (expect near 10 or 80).

Telemetry adds some work. It does not count USB packets received by the host,
measure noise bandwidth, or measure end-to-end latency. For stronger evidence:

1. Log idle and fixed-pressure raw readings and quantify peak-to-peak and RMS
   noise relative to the calibrated span. Capture individual conversions with
   dedicated instrumentation; the 10 Hz terminal monitor is insufficient for
   an 80 SPS noise spectrum.
2. Inspect USB endpoint descriptors (`lsusb -v`) and actual interrupt transfers
   with a USB capture/logic analyzer. Timestamp sensor transitions and host
   events using a shared reference before claiming end-to-end latency.
3. Test full release, fast repeated presses, different USB ports/hubs, serial
   monitoring, host suspend/resume, and a disconnected brake module.
4. Run the intended game, verify bindings/endpoints, and compare stability with
   monitoring turned off. Game rendering/physics/input scheduling adds latency.

An HX711 at 80 SPS can be substantially more responsive than the same hardware
at 10 SPS. For genuinely fresh brake measurements at hundreds or thousands of
Hz, a different ADC/interface would be needed. That is outside this hardware
version; the wheelbase does not alter these sensor limits.

## Sources

- [HX711 manufacturer datasheet](https://image.dfrobot.com/image/data/SEN0160/hx711_english.pdf): rates, resolution, filtering/settling and noise.
- [Arduino analogRead reference](https://docs.arduino.cc/language-reference/en/functions/analog-io/analogRead/): ADC resolution and AVR analog-reading behavior.
- [HX711_ADC 1.2.12 configuration](https://github.com/olkal/HX711_ADC/blob/1.2.12/src/config.h): sample and outlier-filter settings.
- [Joystick v2.1.1 USB endpoint](https://github.com/MHeironimus/ArduinoJoystickLibrary/blob/v2.1.1/src/DynamicHID/DynamicHID.cpp): 1 ms endpoint request.
- [Joystick v2.1.1 mapping](https://github.com/MHeironimus/ArduinoJoystickLibrary/blob/v2.1.1/src/Joystick.cpp): report size and integer mapping.
