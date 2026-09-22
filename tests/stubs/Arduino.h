#ifndef TESTS_STUBS_ARDUINO_H_
#define TESTS_STUBS_ARDUINO_H_
#include <stdint.h>

#include <cstdio>
#include <deque>
#include <string>
#define A0 0
#define A2 2
#define A3 3
#define A5 5
#define A8 8
#define INPUT 0
#define PSTR(x) x
#define vsnprintf_P vsnprintf
extern uint32_t clockUs;
extern int inputs[9];
inline uint32_t millis() { return clockUs / 1000; }
inline uint32_t micros() { return clockUs; }
inline int analogRead(int pin) { return inputs[pin]; }
inline void pinMode(int, int) {}
struct SerialStub {
  std::deque<char> rx;
  std::string tx;
  bool connected = true;
  int room = 64;
  void begin(int) {}
  explicit operator bool() const { return connected; }
  int available() const { return rx.size(); }
  int availableForWrite() const { return room; }
  int read() {
    char c = rx.front();
    rx.pop_front();
    return c;
  }
  size_t write(const uint8_t* p, size_t n) {
    tx.append(reinterpret_cast<const char*>(p), n);
    return n;
  }
};
extern SerialStub Serial;
#endif  // TESTS_STUBS_ARDUINO_H_
