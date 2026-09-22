#ifndef ARDUINOTEC_PEDALS_CALIBRATIONSTORE_H_
#define ARDUINOTEC_PEDALS_CALIBRATIONSTORE_H_

#include <string.h>

#include "calibration.h"  // NOLINT(build/include_subdir): Arduino sketch-local header.

namespace pedals {
// Fixed little-endian wire format, independent of compiler struct padding.
// Two slots; invalidate target first and commit it last. The other stays valid.
const uint8_t kSlotSize = 50;
inline void put32(uint8_t* p, uint32_t value) {
  for (uint8_t i = 0; i < 4; ++i) p[i] = value >> (8 * i);
}
inline uint32_t get32(const uint8_t* p) {
  uint32_t result = 0;
  for (uint8_t i = 0; i < 4; ++i)
    result |= static_cast<uint32_t>(p[i]) << (8 * i);
  return result;
}
inline uint16_t checksum(const uint8_t* bytes, uint8_t size) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < size; ++i) {
    crc ^= static_cast<uint16_t>(bytes[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
  }
  return crc;
}

template <typename Memory>
class CalibrationStore {
 public:
  CalibrationStore(Memory& memory, uint32_t profile, bool clutch)
      : memory_(memory), profile_(profile), clutch_(clutch) {}

  bool load(Calibration* result) {
    Calibration a = {}, b = {};
    uint32_t ga = 0, gb = 0;
    bool va = readSlot(0, &a, &ga), vb = readSlot(1, &b, &gb);
    slot_ = -1;
    generation_ = 0;
    if (!va && !vb) return false;
    bool useB = vb && (!va || (gb != ga && uint32_t(gb - ga) < 0x80000000UL));
    *result = useB ? b : a;
    slot_ = useB ? 1 : 0;
    generation_ = useB ? gb : ga;
    return true;
  }

  bool save(const Calibration& cal) {
    if (!validCalibration(cal, clutch_)) return false;
    uint8_t bytes[kSlotSize] = {0};
    bytes[0] = 0xA5;
    bytes[1] = 1;
    bytes[2] = 'P';
    bytes[3] = 'D';
    put32(bytes + 4, generation_ + 1);
    put32(bytes + 8, profile_);
    for (uint8_t i = 0; i < kAxes; ++i) {
      uint8_t* p = bytes + 12 + 12 * i;
      put32(p, cal.axes[i].rest);
      put32(p + 4, cal.axes[i].full);
      p[8] = cal.axes[i].lower;
      p[9] = cal.axes[i].lower >> 8;
      p[10] = cal.axes[i].upper;
      p[11] = cal.axes[i].upper >> 8;
    }
    uint16_t crc = checksum(bytes + 1, kSlotSize - 3);
    bytes[48] = crc;
    bytes[49] = crc >> 8;
    int target = slot_ == 0 ? 1 : 0;
    int address = target * kSlotSize;
    memory_.update(address, 0);
    for (uint8_t i = 1; i < kSlotSize; ++i)
      memory_.update(address + i, bytes[i]);
    memory_.update(address, bytes[0]);
    Calibration verify = {};
    uint32_t generation;
    if (!readSlot(target, &verify, &generation)) return false;
    slot_ = target;
    generation_ = generation;
    return true;
  }

 private:
  bool readSlot(uint8_t slot, Calibration* cal, uint32_t* generation) {
    uint8_t bytes[kSlotSize];
    for (uint8_t i = 0; i < kSlotSize; ++i)
      bytes[i] = memory_.read(slot * kSlotSize + i);
    if (bytes[0] != 0xA5 || bytes[1] != 1 || bytes[2] != 'P' ||
        bytes[3] != 'D' || get32(bytes + 8) != profile_)
      return false;
    uint16_t crc = bytes[48] | (static_cast<uint16_t>(bytes[49]) << 8);
    if (crc != checksum(bytes + 1, kSlotSize - 3)) return false;
    for (uint8_t i = 0; i < kAxes; ++i) {
      const uint8_t* p = bytes + 12 + 12 * i;
      // Reject out-of-range unsigned representations before signed conversion.
      if (get32(p) > kBrakeMax || get32(p + 4) > kBrakeMax) return false;
      cal->axes[i] = {static_cast<int32_t>(get32(p)),
                      static_cast<int32_t>(get32(p + 4)),
                      static_cast<uint16_t>(p[8] | (uint16_t(p[9]) << 8)),
                      static_cast<uint16_t>(p[10] | (uint16_t(p[11]) << 8))};
    }
    *generation = get32(bytes + 4);
    return validCalibration(*cal, clutch_);
  }
  Memory& memory_;
  uint32_t profile_;
  bool clutch_;
  int8_t slot_ = -1;
  uint32_t generation_ = 0;
};
}  // namespace pedals
#endif  // ARDUINOTEC_PEDALS_CALIBRATIONSTORE_H_
