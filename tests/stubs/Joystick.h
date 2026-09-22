#ifndef TESTS_STUBS_JOYSTICK_H_
#define TESTS_STUBS_JOYSTICK_H_
#define JOYSTICK_DEFAULT_REPORT_ID 3
#define JOYSTICK_TYPE_JOYSTICK 4
class Joystick_ {
 public:
  template <typename... Args>
  explicit Joystick_(Args...) {}
  int brake = 0, throttle = 0, clutch = 0, reports = 0;
  bool automatic = true;
  void setRxAxisRange(int, int) {}
  void setThrottleRange(int, int) {}
  void setBrakeRange(int, int) {}
  void begin(bool value) { automatic = value; }
  void setRxAxis(int value) {
    clutch = value;
    if (automatic) sendState();
  }
  void setThrottle(int value) {
    throttle = value;
    if (automatic) sendState();
  }
  void setBrake(int value) {
    brake = value;
    if (automatic) sendState();
  }
  void sendState() { ++reports; }
};
#endif  // TESTS_STUBS_JOYSTICK_H_
