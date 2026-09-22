#ifndef ARDUINOTEC_PEDALS_CALIBRATION_H_
#define ARDUINOTEC_PEDALS_CALIBRATION_H_

#include <stdint.h>

namespace pedals {
const uint8_t kAxes = 3;
enum Axis { kThrottle, kClutch, kBrake };
// The Joystick library maps to a 16-bit USB field using signed 32-bit math.
// 32767 * 65535 fits that intermediate; 65535 * 65535 does not.
const uint16_t kOutputMax = 32767;
const int32_t kBrakeMax = 0xFFFFFF;

struct AxisCalibration {
  int32_t rest;
  int32_t full;
  uint16_t lower;  // Parts per thousand of calibrated travel.
  uint16_t upper;
};
struct Calibration {
  AxisCalibration axes[kAxes];
};

inline bool validAxis(const AxisCalibration& axis, uint8_t index) {
  int32_t limit = index == kBrake ? kBrakeMax : 1023;
  if (axis.rest < 0 || axis.rest > limit || axis.full < 0 || axis.full > limit)
    return false;
  int32_t span = axis.full - axis.rest;
  if (span < 0) span = -span;
  return span >= (index == kBrake ? 4096 : 32) && axis.lower <= 100 &&
         axis.upper <= 100;
}

inline bool validCalibration(const Calibration& cal, bool clutch) {
  for (uint8_t i = 0; i < kAxes; ++i) {
    if (i == kClutch && !clutch) continue;
    if (!validAxis(cal.axes[i], i)) return false;
  }
  return true;
}

inline uint16_t normalize(int32_t raw, const AxisCalibration& cal) {
  // Inputs and endpoints are range-checked before reaching this function.
  int32_t span = cal.full - cal.rest;
  int32_t travel = raw - cal.rest;
  if (span < 0) {
    span = -span;
    travel = -travel;
  }
  if (span == 0 || travel <= 0) return 0;
  if (travel >= span) return kOutputMax;
  float position = static_cast<float>(travel) / span;
  float lower = cal.lower / 1000.0f;
  float upper = 1.0f - cal.upper / 1000.0f;
  if (position <= lower) return 0;
  if (position >= upper) return kOutputMax;
  return static_cast<uint16_t>((position - lower) * kOutputMax /
                               (upper - lower));
}

struct Capture {
  int64_t sum = 0;
  int32_t low = kBrakeMax;
  int32_t high = 0;
  uint16_t count = 0;
  void add(int32_t value) {
    sum += value;
    if (value < low) low = value;
    if (value > high) high = value;
    ++count;
  }
  int32_t mean() const { return count ? static_cast<int32_t>(sum / count) : 0; }
  bool stable(uint8_t axis) const {
    return count >= (axis == kBrake ? 4 : 100) &&
           high - low <= (axis == kBrake ? 1000 : 8);
  }
};
}  // namespace pedals
#endif  // ARDUINOTEC_PEDALS_CALIBRATION_H_
