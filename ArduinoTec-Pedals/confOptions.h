#ifndef ARDUINOTEC_PEDALS_CONFOPTIONS_H_
#define ARDUINOTEC_PEDALS_CONFOPTIONS_H_

// Default wiring: 5 V / 16 MHz ATmega32u4 + HX711 channel A, gain 128.
#define Throttle A0
#define Clutch A2
#define Throttle_I2 A5
#define Clutch_I2 A8
#define BrakeResistance A3
#define HX711_dout 3
#define HX711_sck 5

#define CLUTCH_ENABLED true
#define use_Dual_Thr false
#define use_Dual_Cl false
// Saved maximum brake pressure replaces the old sensitivity knob by default.
#define BRAKE_POT_ENABLED false

#define REPORT_INTERVAL_US 1000UL
#define BRAKE_TIMEOUT_MS 250UL
#define SENSOR_WARMUP_MS 2000UL
#define CAPTURE_MS 800UL
#define CALIBRATION_IDLE_MS 120000UL
#define DEFAULT_DEADZONE 5  // 0.5%, in parts per thousand, at each endpoint.

// Bump when changing sensor wiring or processing so old EEPROM data is
// rejected.
#define CALIBRATION_PROFILE 1UL
#endif  // ARDUINOTEC_PEDALS_CONFOPTIONS_H_
