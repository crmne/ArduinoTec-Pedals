#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

#include "Arduino.h"
#include "EEPROM.h"
// Checks must execute even when a host compiler defines NDEBUG.
#define REQUIRE(condition)                                  \
  do {                                                      \
    if (!(condition)) throw std::runtime_error(#condition); \
  } while (false)

uint32_t clockUs = 0;
int inputs[9] = {};
SerialStub Serial;
EEPROMClass EEPROM;
#include "../ArduinoTec-Pedals/ArduinoTec-Pedals.ino"  // NOLINT(build/include)

void advance(int ms, bool connected = true) {
  for (int i = 0; i < ms; ++i) {
    clockUs += 1000;
    LoadCell.ready = connected && millis() % 100 == 0;  // Slow 10 SPS hardware.
    loop();
  }
}
std::string request(const std::string& line, int ms = 20) {
  Serial.tx.clear();
  for (char c : line + "\n") Serial.rx.push_back(c);
  advance(ms);
  return Serial.tx;
}
void expect(const std::string& text, const std::string& expected, int ms = 20) {
  std::string result = request(text, ms);
  if (result != expected) std::cerr << text << " -> " << result;
  REQUIRE(result == expected);
}

void testStore() {
  EEPROMClass memory;
  using Store = pedals::CalibrationStore<EEPROMClass>;
  Store store(memory, 17, true);
  Calibration first = {
      {{100, 900, 5, 5}, {900, 100, 5, 5}, {8000000, 8500000, 5, 5}}};
  Calibration second = first, loaded = {};
  second.axes[0].full = 800;
  REQUIRE(!store.load(&loaded));
  REQUIRE(store.save(first));
  EEPROMClass before = memory;
  REQUIRE(store.save(second));
  int writes = memory.writes - before.writes;
  // Simulate loss of power before every byte mutation in a complete save.
  for (int cut = 0; cut <= writes; ++cut) {
    memory = before;
    Store attempt(memory, 17, true);
    REQUIRE(attempt.load(&loaded));
    memory.failAfter = memory.writes + cut;
    try {
      attempt.save(second);
    } catch (const std::runtime_error&) {
    }
    Store reboot(memory, 17, true);
    REQUIRE(reboot.load(&loaded));
    REQUIRE(loaded.axes[0].full == (cut == writes ? 800 : 900));
  }
  memory.failAfter = -1;
  // Corrupt newest slot and recover old calibration.
  memory.data[pedals::kSlotSize + 20] ^= 1;
  Store recovery(memory, 17, true);
  REQUIRE(recovery.load(&loaded) && loaded.axes[0].full == 900);
  Store wrongProfile(memory, 18, true);
  REQUIRE(!wrongProfile.load(&loaded));
  for (int address = 0; address < pedals::kSlotSize; ++address) {
    memory = before;
    memory.data[address] ^= 1;
    Store corrupted(memory, 17, true);
    REQUIRE(!corrupted.load(&loaded));
  }
  REQUIRE(pedals::validCalibration(first, true));
  first.axes[0].full = first.axes[0].rest;
  REQUIRE(!store.save(first));
}

void testMath() {
  pedals::AxisCalibration up = {100, 900, 5, 5}, down = {900, 100, 5, 5};
  uint16_t previous = 0;
  for (int rawValue = 0; rawValue <= 1023; ++rawValue) {
    auto value = pedals::normalize(rawValue, up);
    REQUIRE(value >= previous && value <= pedals::kOutputMax);
    REQUIRE(value == pedals::normalize(1000 - rawValue, down));
    previous = value;
  }
  REQUIRE(pedals::normalize(100, up) == 0);
  REQUIRE(pedals::normalize(900, up) == pedals::kOutputMax);
  pedals::AxisCalibration wide = {pedals::kBrakeMax - 1, 1, 5, 5};
  REQUIRE(pedals::normalize(1, wide) == pedals::kOutputMax);
  REQUIRE(pedals::normalize(pedals::kBrakeMax, wide) == 0);
}

void runTests() {
  testStore();
  testMath();
  inputs[Throttle] = 100;
  inputs[Clutch] = 900;
  setup();
  advance(2100);
  REQUIRE(Joystick.throttle == 0 && Joystick.brake == 0);
  REQUIRE(reports == 2100 && hallSamples == 2100);
  REQUIRE(!Joystick.automatic);
  expect("CAL SAVE", "ERR BEGIN_REQUIRED\n");
  expect("CAL BEGIN", "OK BEGIN\n");
  expect("CAL FULL brake", "ERR CAPTURE_REST_FIRST\n");
  expect("CAL REST", "OK WAIT\nOK CAPTURE\n", 900);
  expect("CAL SAVE", "ERR INCOMPLETE_CALIBRATION\n");
  // Bad endpoints cannot be saved.
  expect("CAL FULL throttle", "OK WAIT\nERR TRAVEL_TOO_SMALL\n", 900);
  inputs[Throttle] = 900;
  expect("CAL FULL throttle", "OK WAIT\nOK CAPTURE\n", 900);
  inputs[Clutch] = 100;
  expect("CAL FULL clutch", "OK WAIT\nOK CAPTURE\n", 900);
  LoadCell.sample = 8500000;
  expect("CAL FULL brake", "OK WAIT\nOK CAPTURE\n", 900);
  REQUIRE(Joystick.throttle == 0 &&
          Joystick.brake == 0);  // Neutral throughout setup.
  expect("CAL DZ throttle 65536 0", "ERR DEADZONE_RANGE\n");
  expect("CAL DZ throttle -1 0", "ERR DEADZONE_RANGE\n");
  expect("CAL DZ throttle 101 5", "ERR DEADZONE_RANGE\n");
  expect("CAL DZ throttle 5 5 extra", "ERR DEADZONE_RANGE\n");
  expect("CAL DZ throttle 5 5", "OK DEADZONE\n");
  expect("CAL SAVE", "OK SAVED\n");
  REQUIRE(Joystick.throttle == pedals::kOutputMax &&
          Joystick.brake == pedals::kOutputMax);
  Calibration rebooted;
  pedals::CalibrationStore<EEPROMClass> reboot(EEPROM, hardwareProfile, true);
  REQUIRE(reboot.load(&rebooted) && rebooted.axes[2].full == 8500000);
  inputs[Throttle] = 100;
  inputs[Clutch] = 900;
  LoadCell.sample = 8000000;
  advance(120);
  REQUIRE(Joystick.throttle == 0 && Joystick.clutch == 0 &&
          Joystick.brake == 0);
  LoadCell.sample = 8250000;
  advance(120);
  REQUIRE(Joystick.brake > 16000 && Joystick.brake < 16500);
  advance(150, false);
  REQUIRE(Joystick.brake > 0);  // Hold latest valid sample between conversions.
  advance(260, false);
  REQUIRE(Joystick.brake == 0);  // Disconnected ADC must release the brake.
  advance(120);
  REQUIRE(Joystick.brake > 0);
  LoadCell.sample = NAN;
  advance(120);
  REQUIRE(Joystick.brake == 0);
  LoadCell.sample = 8000100;
  advance(120);
  expect("TARE", "OK WAIT\nOK CAPTURE\n", 900);
  REQUIRE(active.axes[kBrake].rest == 8000100);
  REQUIRE(reboot.load(&rebooted) && rebooted.axes[kBrake].rest == 8000000);
  // Cancellation retains active endpoints. Oversized input is discarded
  // entirely.
  expect("CAL BEGIN", "OK BEGIN\n");
  expect("CAL CANCEL", "OK CANCELLED\n");
  expect(std::string(100, 'A'), "ERR LINE_TOO_LONG\n");
  REQUIRE(request("INFO").find("protocol=1") != std::string::npos);
  // A non-reading serial host cannot block sampling.
  Serial.room = 0;
  request("READ");
  uint32_t count = reports;
  advance(500);
  REQUIRE(reports == count + 500);
  Serial.room = 64;
  advance(50);
  // Reject noisy captures, and expire abandoned calibration sessions.
  expect("CAL BEGIN", "OK BEGIN\n");
  expect("CAL REST", "OK WAIT\n");
  inputs[Throttle] = 500;
  advance(500);
  inputs[Throttle] = 100;
  advance(400);
  REQUIRE(Serial.tx.find("ERR UNSTABLE_HOLD_STILL") != std::string::npos);
  advance(CALIBRATION_IDLE_MS);
  REQUIRE(!session && capturing == -1);
  // Report scheduling survives micros() wrap without catch-up bursts.
  clockUs = UINT32_MAX - 500;
  lastFrameUs = clockUs;
  count = reports;
  clockUs += 1000;
  loop();
  REQUIRE(reports == count + 1);
  std::cout
      << "Calibration, EEPROM power-loss, protocol and pedal tests passed\n";
}

int main() {
  try {
    runTests();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << "\n";
    return 1;
  }
}
