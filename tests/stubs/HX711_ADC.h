#ifndef TESTS_STUBS_HX711_ADC_H_
#define TESTS_STUBS_HX711_ADC_H_
class HX711_ADC {
 public:
  HX711_ADC(int, int) {}
  bool ready = false;
  float sample = 0;
  bool tared = false;
  void setSamplesInUse(int) {}
  void begin() {}
  void setReverseOutput() {}
  void start(int, bool) {}
  bool getTareTimeoutFlag() { return false; }
  void setCalFactor(float) {}
  int getSPS() { return 80; }
  bool update() {
    bool result = ready;
    ready = false;
    return result;
  }
  float getData() { return sample; }
  void tareNoDelay() { tared = true; }
  bool getTareStatus() { return false; }
};
#endif  // TESTS_STUBS_HX711_ADC_H_
