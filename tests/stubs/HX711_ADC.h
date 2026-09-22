#ifndef TESTS_STUBS_HX711_ADC_H_
#define TESTS_STUBS_HX711_ADC_H_
#include <stdint.h>
class HX711_ADC {
 public:
  HX711_ADC(int, int) {}
  bool ready = false;
  float sample = 8000000;
  void setSamplesInUse(int) {}
  void begin() {}
  void setCalFactor(float) {}
  void setTareOffset(int32_t) {}
  bool update() {
    bool result = ready;
    ready = false;
    return result;
  }
  float getData() const { return sample; }
};
#endif  // TESTS_STUBS_HX711_ADC_H_
