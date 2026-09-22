#include <cassert>
#include <cmath>
#include <iostream>
#include <string>

#define A0 0
#define A2 2
#define A3 3
#define A5 5
#define A8 8
#define INPUT 0
using boolean = bool;
class String : public std::string {
 public:
  template <typename T>
  explicit String(T value) : std::string(std::to_string(value)) {}
  String(const std::string& value) : std::string(value) {}
};
struct SerialStub {
  int command = -1;
  void begin(int) {}
  template <typename T>
  void print(T) {}
  template <typename T>
  void println(T) {}
  void println() {}
  int available() { return command >= 0; }
  int read() {
    int value = command;
    command = -1;
    return value;
  }
} Serial;
int inputs[9] = {};
int analogRead(int pin) { return inputs[pin]; }
void pinMode(int, int) {}
void delay(int) {}
int read_pedal(int, int, bool);
int get_baseline(int, int, bool, int);
#include "../ArduinoTec-Pedals/ArduinoTec-Pedals.ino"

void brake_sample(float value) {
  LoadCell.sample = value;
  LoadCell.ready = true;
  loop();
}

int main() {
  setup();
  assert(!Joystick.automatic);
  loop();
  assert(Joystick.brake == 0 && Joystick.throttle == 0 && Joystick.clutch == 0);
  brake_sample(100);
  assert(Joystick.brake == 95);
  int reports = Joystick.reports;
  loop();
  assert(Joystick.brake == 95);  // Hold between conversions.
  assert(Joystick.reports == reports + 1);
  brake_sample(0);
  assert(Joystick.brake == 0);  // Release must overwrite the previous report.
  brake_sample(-20);
  assert(Joystick.brake == 0);
  brake_sample(100);
  inputs[BrakeResistance] = 1023;
  loop();
  assert(Joystick.brake == 0);  // Deadzone applies after the sensitivity pot.
  inputs[Throttle] = inputs[Clutch] = 100;
  loop();
  assert(Joystick.throttle == 95 && Joystick.clutch == 95);
  inputs[Throttle] = inputs[Clutch] = 0;
  loop();
  assert(Joystick.throttle == 0 && Joystick.clutch == 0);
  blThr = 100;
  loop();
  assert(Joystick.throttle == 95);  // A decreasing sensor can reach ADC zero.
  inputs[Throttle] = 100;
  inputs[Throttle_I2] = 300;
  assert(get_baseline(Throttle, Throttle_I2, true, 25) == 200);
  assert(read_pedal(Throttle, Throttle_I2, true) == 200);
  assert(get_baseline(Throttle, Throttle_I2, false, 25) == 100);
  Serial.command = 't';
  loop();
  assert(LoadCell.tared);
  for (int maximum = 0; maximum <= 1023; ++maximum) {
    int previous = 0;
    for (int value = -10; value <= 1100; ++value) {
      int output = apply_deadzones(value, maximum, 5, 5, 1023);
      assert(output >= 0 && output <= 1023 && output >= previous);
      previous = output;
    }
  }
  assert(apply_deadzones(2000, 3000, 5, 5, 1023) == 1023);
  inputs[BrakeResistance] = 0;
  brake_sample(INFINITY);
  assert(Joystick.brake == 1023);
  brake_sample(NAN);
  assert(Joystick.brake == 0);
  std::cout << "Pedal regression tests passed\n";
}
