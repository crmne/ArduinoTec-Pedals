#ifndef TESTS_STUBS_EEPROM_H_
#define TESTS_STUBS_EEPROM_H_
#include <stdint.h>

#include <array>
#include <stdexcept>
struct EEPROMClass {
  std::array<uint8_t, 1024> data;
  int writes = 0, failAfter = -1;
  EEPROMClass() { data.fill(255); }
  uint8_t read(int address) const { return data.at(address); }
  void update(int address, uint8_t value) {
    if (data.at(address) == value) return;
    if (failAfter == writes) throw std::runtime_error("power loss");
    data.at(address) = value;
    ++writes;
  }
};
extern EEPROMClass EEPROM;
#endif  // TESTS_STUBS_EEPROM_H_
