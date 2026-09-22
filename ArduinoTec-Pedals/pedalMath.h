#ifndef ARDUINOTEC_PEDALS_PEDALMATH_H_
#define ARDUINOTEC_PEDALS_PEDALMATH_H_

// Preserve the existing raw-unit deadzones without producing negative axes
// while a pedal's observed maximum is still smaller than its upper deadzone.
inline int apply_deadzones(int value, int maximum, int lower, int upper,
                           int range) {
  int ceiling = maximum > upper ? maximum - upper : 0;
  if (ceiling > range) ceiling = range;
  if (value > ceiling) value = ceiling;
  return value > lower ? value : 0;
}

#endif  // ARDUINOTEC_PEDALS_PEDALMATH_H_
