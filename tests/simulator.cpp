// A line-oriented transport for running the real sketch from Python tests.
#include <iostream>
#include <sstream>
#include <string>

#include "Arduino.h"
#include "EEPROM.h"
uint32_t clockUs = 0;
int inputs[9] = {};
SerialStub Serial;
EEPROMClass EEPROM;
#include "../ArduinoTec-Pedals/ArduinoTec-Pedals.ino"  // NOLINT(build/include)

void advanceSimulation(int ms) {
  for (int i = 0; i < ms; ++i) {
    clockUs += 1000;
    LoadCell.ready = millis() % 100 == 0;
    loop();
  }
}
int main() {
  try {
    inputs[Throttle] = 100;
    inputs[Clutch] = 900;
    setup();
    advanceSimulation(2100);
    std::string line;
    while (std::getline(std::cin, line)) {
      // Simulation-only movement instruction, never sent to the firmware.
      if (line.compare(0, 7, "@INPUT ") == 0) {
        std::istringstream values(line.substr(7));
        values >> inputs[Throttle] >> inputs[Clutch] >> LoadCell.sample;
        advanceSimulation(200);
        std::cout << "OK INPUT\n" << std::flush;
        continue;
      }
      Serial.tx.clear();
      for (char c : line + "\n") Serial.rx.push_back(c);
      advanceSimulation(1000);
      std::cout << Serial.tx << std::flush;
    }
  } catch (const std::exception& error) {
    std::cerr << error.what() << "\n";
    return 1;
  }
}
