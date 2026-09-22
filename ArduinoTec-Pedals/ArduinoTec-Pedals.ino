// ArduinoTec-Pedals: persistent calibration and serial setup, September 2026.
// Derived from jssting/ArduinoTec-Pedals; distributed under GPL-3.0 (LICENSE).
#include <Arduino.h>
#include <EEPROM.h>
#include <HX711_ADC.h>
#include <Joystick.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "calibrationStore.h"
#include "confOptions.h"
#include "version.h"  // NOLINT(build/include_subdir): Arduino sketch-local header.

using pedals::Calibration;
using pedals::kBrake;
using pedals::kClutch;
using pedals::kThrottle;

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_JOYSTICK, 0, 0,
                   false, false, false, CLUTCH_ENABLED, false, false, false,
                   true, false, true, false);
HX711_ADC LoadCell(HX711_dout, HX711_sck);
const uint32_t hardwareProfile = CALIBRATION_PROFILE * 16 + CLUTCH_ENABLED +
                                 2 * use_Dual_Thr + 4 * use_Dual_Cl +
                                 8 * BRAKE_POT_ENABLED;
pedals::CalibrationStore<EEPROMClass> storage(EEPROM, hardwareProfile,
                                              CLUTCH_ENABLED);
Calibration active = {}, candidate = {};
bool calibrated = false, session = false, brakeFault = true;
int32_t sensorRaw[3] = {};
uint16_t output[3] = {};
uint32_t startedMs, lastBrakeMs, lastFrameUs, lastCommandMs;
uint32_t reports = 0, hallSamples = 0, brakeSamples = 0;
uint8_t capturesDone = 0;
// -1: idle; 0..2: full endpoint; 3: all rest endpoints; 4: temporary brake
// tare.
int8_t capturing = -1;
uint32_t captureStarted;
pedals::Capture captures[3];
char command[64] = {}, response[256] = {};
uint8_t commandLength = 0;
bool overflow = false;
uint16_t responseLength = 0, responsePosition = 0;

void reply(const char* format, ...) {
  va_list args;
  va_start(args, format);
  int count = vsnprintf_P(response, sizeof(response) - 1, format, args);
  va_end(args);
  if (count < 0 || count >= static_cast<int>(sizeof(response) - 1)) {
    count = sizeof("ERR RESPONSE_SIZE") - 1;
    memcpy(response, "ERR RESPONSE_SIZE", count);
  }
  response[count++] = '\n';
  responseLength = count;
  responsePosition = 0;
}

bool brakeHealthy(uint32_t now) {
  return uint32_t(now - startedMs) >= SENSOR_WARMUP_MS && brakeSamples >= 3 &&
         !brakeFault && uint32_t(now - lastBrakeMs) <= BRAKE_TIMEOUT_MS;
}

void neutral() {
  for (uint8_t i = 0; i < 3; ++i) output[i] = 0;
}

void sendReport() {
  Joystick.setThrottle(output[kThrottle]);
  Joystick.setRxAxis(output[kClutch]);
  Joystick.setBrake(output[kBrake]);
  Joystick.sendState();
  ++reports;  // Submitted reports, not a measurement of host USB delivery.
}

int readPedal(int pin, int secondPin, bool dual) {
  int value = analogRead(pin);
  return dual ? (value + analogRead(secondPin)) / 2 : value;
}

void setup() {
  Serial.begin(57600);
  pinMode(Throttle, INPUT);
  if (CLUTCH_ENABLED) pinMode(Clutch, INPUT);
  if (use_Dual_Thr) pinMode(Throttle_I2, INPUT);
  if (CLUTCH_ENABLED && use_Dual_Cl) pinMode(Clutch_I2, INPUT);
  if (BRAKE_POT_ENABLED) pinMode(BrakeResistance, INPUT);
  LoadCell.begin();
  LoadCell.setSamplesInUse(
      1);  // Default library also rejects one high/low sample.
  LoadCell.setCalFactor(1.0f);
  LoadCell.setTareOffset(
      0);  // Keep absolute readings; saved endpoints define zero.
  calibrated = storage.load(&active);
  Joystick.setRxAxisRange(0, pedals::kOutputMax);
  Joystick.setThrottleRange(0, pedals::kOutputMax);
  Joystick.setBrakeRange(0, pedals::kOutputMax);
  Joystick.begin(false);
  startedMs = millis();
  lastFrameUs = micros();
}

int axisIndex(const char* name) {
  if (strcmp(name, "throttle") == 0) return kThrottle;
  if (strcmp(name, "clutch") == 0 && CLUTCH_ENABLED) return kClutch;
  if (strcmp(name, "brake") == 0) return kBrake;
  return -1;
}

void startCapture(int8_t kind) {
  if (!brakeHealthy(millis())) {
    reply(PSTR("ERR BRAKE_NOT_READY"));
    return;
  }
  if (kind <= kBrake && !(capturesDone & 8)) {
    reply(PSTR("ERR CAPTURE_REST_FIRST"));
    return;
  }
  for (uint8_t i = 0; i < 3; ++i) captures[i] = pedals::Capture();
  capturing = kind;
  captureStarted = millis();
  neutral();
  reply(PSTR("OK WAIT"));
}

bool parseDeadzone(char** input, uint16_t* result, char end) {
  char* p = *input;
  if (*p < '0' || *p > '9') return false;
  uint16_t value = 0;
  while (*p >= '0' && *p <= '9') {
    value = value * 10 + (*p++ - '0');
    if (value > 100) return false;
  }
  if (*p != end) return false;
  *input = end ? p + 1 : p;
  *result = value;
  return true;
}

// Printf %ld/%lu require long even when the host tests use 64-bit long.
// NOLINTBEGIN(runtime/int)
void handleCommand() {
  lastCommandMs = millis();
  if (!strcmp(command, "CAL CANCEL")) {
    session = false;
    capturing = -1;
    reply(PSTR("OK CANCELLED"));
    return;
  }
  if (capturing >= 0) {
    reply(PSTR("ERR BUSY"));
    return;
  }
  if (!strcmp(command, "INFO")) {
    reply(PSTR("OK INFO protocol=1 firmware=" PEDALS_VERSION
               " calibrated=%u session=%u clutch=%u brake_ok=%u target_hz=%lu"),
          calibrated, session, CLUTCH_ENABLED, brakeHealthy(millis()),
          1000000UL / REPORT_INTERVAL_US);
  } else if (!strcmp(command, "READ")) {
    reply(PSTR("OK READ raw=%ld,%ld,%ld out=%u,%u,%u brake_ok=%u age_ms=%lu"
               " uptime_ms=%lu reports=%lu hall_samples=%lu brake_samples=%lu"),
          static_cast<long>(sensorRaw[0]), static_cast<long>(sensorRaw[1]),
          static_cast<long>(sensorRaw[2]), output[0], output[1], output[2],
          brakeHealthy(millis()),
          static_cast<unsigned long>(millis() - lastBrakeMs),
          static_cast<unsigned long>(millis()),
          static_cast<unsigned long>(reports),
          static_cast<unsigned long>(hallSamples),
          static_cast<unsigned long>(brakeSamples));
  } else if (!strcmp(command, "CAL BEGIN")) {
    if (session) {
      reply(PSTR("ERR ALREADY_CALIBRATING"));
      return;
    }
    candidate = {};
    for (uint8_t i = 0; i < 3; ++i) {
      candidate.axes[i].lower = DEFAULT_DEADZONE;
      candidate.axes[i].upper = DEFAULT_DEADZONE;
    }
    session = true;
    capturesDone = 0;
    neutral();
    reply(PSTR("OK BEGIN"));
  } else if (!strcmp(command, "CAL SHOW")) {
    const Calibration& cal = session ? candidate : active;
    reply(PSTR("OK CAL rest=%ld,%ld,%ld full=%ld,%ld,%ld lower=%u,%u,%u "
               "upper=%u,%u,%u"),
          static_cast<long>(cal.axes[0].rest),
          static_cast<long>(cal.axes[1].rest),
          static_cast<long>(cal.axes[2].rest),
          static_cast<long>(cal.axes[0].full),
          static_cast<long>(cal.axes[1].full),
          static_cast<long>(cal.axes[2].full), cal.axes[0].lower,
          cal.axes[1].lower, cal.axes[2].lower, cal.axes[0].upper,
          cal.axes[1].upper, cal.axes[2].upper);
  } else if (!strcmp(command, "t") || !strcmp(command, "TARE")) {
    if (session || !calibrated) {
      reply(PSTR("ERR CALIBRATION_REQUIRED"));
      return;
    }
    startCapture(4);
  } else if (!strncmp(command, "CAL ", 4)) {
    if (!session) {
      reply(PSTR("ERR BEGIN_REQUIRED"));
      return;
    }
    if (!strcmp(command, "CAL REST")) {
      startCapture(3);
    } else if (!strncmp(command, "CAL FULL ", 9)) {
      int axis = axisIndex(command + 9);
      if (axis < 0)
        reply(PSTR("ERR AXIS"));
      else
        startCapture(axis);
    } else if (!strncmp(command, "CAL DZ ", 7)) {
      char* name = command + 7;
      char* separator = strchr(name, ' ');
      if (!separator) {
        reply(PSTR("ERR DEADZONE_RANGE"));
        return;
      }
      *separator = 0;
      int axis = axisIndex(name);
      char* values = separator + 1;
      uint16_t lower = 0, upper = 0;
      if (axis < 0 || !parseDeadzone(&values, &lower, ' ') ||
          !parseDeadzone(&values, &upper, '\0')) {
        reply(PSTR("ERR DEADZONE_RANGE"));
        return;
      }
      candidate.axes[axis].lower = lower;
      candidate.axes[axis].upper = upper;
      reply(PSTR("OK DEADZONE"));
    } else if (!strcmp(command, "CAL SAVE")) {
      uint8_t required = CLUTCH_ENABLED ? 15 : 13;
      if (capturesDone != required ||
          !pedals::validCalibration(candidate, CLUTCH_ENABLED)) {
        reply(PSTR("ERR INCOMPLETE_CALIBRATION"));
        return;
      }
      if (!brakeHealthy(millis())) {
        reply(PSTR("ERR BRAKE_NOT_READY"));
        return;
      }
      neutral();
      sendReport();  // EEPROM writes happen only in neutral setup mode.
      if (!storage.save(candidate)) {
        reply(PSTR("ERR EEPROM"));
        return;
      }
      active = candidate;
      calibrated = true;
      session = false;
      reply(PSTR("OK SAVED"));
    } else {
      reply(PSTR("ERR COMMAND"));
    }
  } else {
    reply(PSTR("ERR COMMAND"));
  }
}

// NOLINTEND

void serviceSerial() {
  // Bounded work and no blocking writes when a terminal stops reading.
  if (!Serial) {
    responseLength = responsePosition = 0;
    commandLength = 0;
    overflow = false;
    return;
  }
  if (responsePosition < responseLength) {
    int room = Serial.availableForWrite();
    uint16_t remaining = responseLength - responsePosition;
    if (room > 0) {
      size_t count = remaining < room ? remaining : room;
      responsePosition += Serial.write(
          reinterpret_cast<const uint8_t*>(response) + responsePosition, count);
    }
    return;
  }
  for (uint8_t budget = 0; budget < 16 && Serial.available(); ++budget) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      command[commandLength] = 0;
      if (overflow)
        reply(PSTR("ERR LINE_TOO_LONG"));
      else if (commandLength)
        handleCommand();
      commandLength = 0;
      overflow = false;
      return;
    }
    // Preserve the original single-byte 't' command (no newline required).
    if (c == 't' && commandLength == 0 && !overflow) {
      command[0] = 't';
      command[1] = 0;
      handleCommand();
      return;
    }
    if (static_cast<uint8_t>(c) < 32 || static_cast<uint8_t>(c) > 126)
      overflow = true;
    if (!overflow && commandLength < sizeof(command) - 1)
      command[commandLength++] = c;
    else
      overflow = true;
  }
}

void finishCapture() {
  if (capturing < 0 || uint32_t(millis() - captureStarted) < CAPTURE_MS ||
      responsePosition < responseLength)
    return;
  int8_t kind = capturing;
  capturing = -1;
  if (!brakeHealthy(millis())) {
    reply(PSTR("ERR BRAKE_NOT_READY"));
    return;
  }
  for (uint8_t i = 0; i < 3; ++i) {
    if (i == kClutch && !CLUTCH_ENABLED) continue;
    if (kind == 3 || i == kind || (kind == 4 && i == kBrake)) {
      if (!captures[i].stable(i)) {
        reply(PSTR("ERR UNSTABLE_HOLD_STILL"));
        return;
      }
    }
  }
  if (kind == 3) {
    for (uint8_t i = 0; i < 3; ++i) candidate.axes[i].rest = captures[i].mean();
    capturesDone =
        8;  // Recapturing rest invalidates every previous full endpoint.
  } else if (kind == 4) {
    Calibration adjusted = active;
    int32_t delta = captures[kBrake].mean() - adjusted.axes[kBrake].rest;
    adjusted.axes[kBrake].rest += delta;
    adjusted.axes[kBrake].full += delta;
    if (!pedals::validCalibration(adjusted, CLUTCH_ENABLED)) {
      reply(PSTR("ERR TARE_RANGE"));
      return;
    }
    active = adjusted;  // Temporary offset only; never writes EEPROM.
  } else {
    candidate.axes[kind].full = captures[kind].mean();
    if (!pedals::validAxis(candidate.axes[kind], kind)) {
      capturesDone &= ~(1 << kind);
      reply(PSTR("ERR TRAVEL_TOO_SMALL"));
      return;
    }
    capturesDone |= 1 << kind;
  }
  reply(PSTR("OK CAPTURE"));
}

void loop() {
  uint32_t now = millis();
  bool freshBrake = LoadCell.update();
  if (freshBrake) {
    float sample = LoadCell.getData();
    if (sample > 0 && sample < pedals::kBrakeMax) {
      sensorRaw[kBrake] = static_cast<int32_t>(sample);
      lastBrakeMs = now;
      brakeFault = false;
      ++brakeSamples;
      if (capturing >= 0 && uint32_t(now - captureStarted) < CAPTURE_MS)
        captures[kBrake].add(sensorRaw[kBrake]);
    } else {
      brakeFault = true;
    }
  }
  if ((session || capturing >= 0) &&
      uint32_t(now - lastCommandMs) >= CALIBRATION_IDLE_MS) {
    session = false;
    capturing = -1;
  }
  uint32_t tick = micros();
  if (uint32_t(tick - lastFrameUs) >= REPORT_INTERVAL_US) {
    lastFrameUs =
        tick;  // Do not replay a backlog after a slow host or EEPROM save.
    sensorRaw[kThrottle] = readPedal(Throttle, Throttle_I2, use_Dual_Thr);
    sensorRaw[kClutch] =
        CLUTCH_ENABLED ? readPedal(Clutch, Clutch_I2, use_Dual_Cl) : 0;
    ++hallSamples;
    if (capturing >= 0 && uint32_t(now - captureStarted) < CAPTURE_MS) {
      captures[kThrottle].add(sensorRaw[kThrottle]);
      if (CLUTCH_ENABLED) captures[kClutch].add(sensorRaw[kClutch]);
    }
    neutral();
    if (calibrated && !session && capturing < 0 &&
        uint32_t(now - startedMs) >= SENSOR_WARMUP_MS) {
      output[kThrottle] =
          pedals::normalize(sensorRaw[kThrottle], active.axes[kThrottle]);
      if (CLUTCH_ENABLED)
        output[kClutch] =
            pedals::normalize(sensorRaw[kClutch], active.axes[kClutch]);
      if (brakeHealthy(now))
        output[kBrake] =
            pedals::normalize(sensorRaw[kBrake], active.axes[kBrake]);
      if (BRAKE_POT_ENABLED)
        output[kBrake] = static_cast<uint32_t>(output[kBrake]) *
                         (1023 - analogRead(BrakeResistance)) / 1023;
    }
    sendReport();
  }
  finishCapture();
  serviceSerial();
}
