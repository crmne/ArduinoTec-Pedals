#include <HX711_ADC.h>
#include <Joystick.h>
#include <limits.h>

#include "confOptions.h"
#include "pedalMath.h"

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_MULTI_AXIS, 0, 0,
                   false, false, false, true, false, false, false, true, false,
                   true, false);

HX711_ADC LoadCell(HX711_dout, HX711_sck);

int lastBrakeReading = 0;

const int MaxRange = 1023;
const int MinRange = 0;

const int AxisMax = 1024;

int blBrk = 0;
int blThr = 0;
int blCth = 0;
int maxBrk = 0;  // max value for Brake
int maxThr = 0;  // max value for Throttle
int maxCth = 0;  // max value for Clutch

bool debug = false;
bool dbg_PrintThr = false;
bool dbg_PrintBrk = false;
bool dbg_PrintCTH = false;

void setup() {
  Serial.begin(57600);
  delay(10);
  Serial.println();
  Serial.println("Starting...");

  debug = Enable_Debug;
  dbg_PrintThr = Debug_Thr;
  dbg_PrintBrk = Debug_Brk;
  dbg_PrintCTH = Debug_Cth;

  // setting ranges
  Joystick.setRxAxisRange(MinRange, MaxRange);
  Joystick.setThrottleRange(MinRange, MaxRange);
  Joystick.setBrakeRange(MinRange, MaxRange);

  Joystick.begin(false);

  // Setting Pin Modes
  pinMode(Throttle, INPUT);  // Throttle
  pinMode(Clutch, INPUT);    // Clutch

  // Setting optional pins states
  if (use_Dual_Thr) pinMode(Throttle_I2, INPUT);  // Throttle input 2
  if (use_Dual_Cl) pinMode(Clutch_I2, INPUT);     // Clutch input 2

  pinMode(BrakeResistance, INPUT);  // BrakeResistance;

  // get the baseline registering values
  blThr = get_baseline(Throttle, Throttle_I2, use_Dual_Thr, 25);
  blCth = get_baseline(Clutch, Clutch_I2, use_Dual_Cl, 25);

  LoadCell.setSamplesInUse(1);
  LoadCell.begin();
  LoadCell.setReverseOutput();
  boolean _tare = true;  // set this to false if you don't want tare to be
                         // performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1) {
    }
  } else {
    LoadCell.setCalFactor(brake_calibration);  // set calibration value (float)
    Serial.println("Startup is complete");
  }

  Serial.print("SPS set to: ");
  Serial.println(LoadCell.getSPS());
}

void loop() {
  // reading the relevant values from the pedals
  int valThr = read_pedal(Throttle, Throttle_I2, use_Dual_Thr);
  int valCth = read_pedal(Clutch, Clutch_I2, use_Dual_Cl);

  // Poll in the main loop so getData() cannot race an ISR modifying its
  // dataset. Keep the last conversion between HX711 samples, including zero on
  // release.
  if (LoadCell.update()) {
    float sample = LoadCell.getData();
    lastBrakeReading = sample >= INT_MAX
                           ? INT_MAX
                           : (sample > 0 ? static_cast<int>(sample) : 0);
  }
  int valBrk = lastBrakeReading;

  // receive command from serial terminal, send 't' to initiate tare operation:
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') LoadCell.tareNoDelay();
  }

  // check if last tare operation is complete
  if (LoadCell.getTareStatus() == true) {
    Serial.println("Tare complete");
  }

  int valRestBrk = analogRead(BrakeResistance);
  double pers = 1.0 - double(valRestBrk) / AxisMax;

  // All values below are normalised and converted to absolute to ensure a
  // positive value
  int actCthVal = abs(valCth - blCth);  // normalised clutch value
  int actThrVal = abs(valThr - blThr);  // normalised Throttle value
  int actBrkVal = abs(valBrk - blBrk);  // normalised brake value
  int PersBrkVal =
      int(((valBrk - blBrk) *
           pers));  // normalised brake value with the resistance added

  // set/update the max value based on pedal progressions
  if (maxCth < actCthVal) maxCth = actCthVal;
  if (maxThr < actThrVal) maxThr = actThrVal;
  if (maxBrk < actBrkVal) maxBrk = actBrkVal;

  if (debug) {
    if (dbg_PrintThr) {
      String values = "Throttle - Baseline: " + String(blThr) +
                      ", Actual Value: " + String(valThr) +
                      ", Normalised Value: " + String(actThrVal) +
                      ", max value: " + String(maxThr);
      Serial.println(values);
      delay(100);
    }

    if (dbg_PrintBrk) {
      String values = "Brakes - Baseline: " + String(blBrk) +
                      ", Actual Value: " + String(valBrk) +
                      ", Normalised Value: " + String(actBrkVal) +
                      ", Normalised With Resistance:" + String(PersBrkVal) +
                      ", Pers:" + String(pers) +
                      ", Resistance: " + String(valRestBrk) +
                      ", max value: " + String(maxBrk);
      Serial.println(values);
      delay(100);
    }

    if (dbg_PrintCTH) {
      String values = "Clutch - Baseline: " + String(blCth) +
                      ", Actual Value: " + String(valCth) +
                      ", Normalised Value: " + String(actCthVal) +
                      ", Max Value: " + String(maxCth);
      Serial.println(values);
      delay(100);
    }
    Serial.println("");  // create an empty line
  }

  Joystick.setRxAxis(
      apply_deadzones(actCthVal, maxCth, clutch_L_DZ, clutch_U_DZ, MaxRange));
  Joystick.setThrottle(apply_deadzones(actThrVal, maxThr, throttle_L_DZ,
                                       throttle_U_DZ, MaxRange));
  Joystick.setBrake(
      apply_deadzones(PersBrkVal, maxBrk, brake_L_DZ, brake_U_DZ, MaxRange));
  Joystick.sendState();
}

int read_pedal(int pin, int secondPin, bool dual) {
  int value = analogRead(pin);
  return dual ? (value + analogRead(secondPin)) / 2 : value;
}

int get_baseline(int pin, int secondPin, bool dual, int count) {
  int maxVal = 0;
  for (int i = 0; i < count; i++) {
    int newVal = read_pedal(pin, secondPin, dual);
    if (maxVal < newVal) maxVal = newVal;
  }
  return maxVal;
}
