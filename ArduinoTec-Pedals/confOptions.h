// confOptions.h
// debug options
#ifndef ARDUINOTEC_PEDALS_CONFOPTIONS_H_
#define ARDUINOTEC_PEDALS_CONFOPTIONS_H_

#define Throttle A0
#define Clutch A2

// second input line (brown wire)
#define Throttle_I2 A5
// second input line (brown wire)
#define Clutch_I2 A8

#define BrakeResistance A3

// mcu > HX711 dout pin, must be external interrupt capable!
#define HX711_dout 3
// mcu > HX711 sck pin
#define HX711_sck 5

// tare precision can be improved by adding a few seconds of stabilizing time
#define stabilizingtime 2000

#define Enable_Debug false
#define Debug_Thr false
#define Debug_Cth false
#define Debug_Brk false

// Input Precision
#define use_Dual_Thr false
#define use_Dual_Cl false
#define use_Dual_Brk false

// initial Deadzone
#define brake_L_DZ 5
#define throttle_L_DZ 5
#define clutch_L_DZ 5

// maximum Deadzone
#define brake_U_DZ 5
#define throttle_U_DZ 5
#define clutch_U_DZ 5

// Brake calibration
#define brake_calibration 696.0

#endif  // ARDUINOTEC_PEDALS_CONFOPTIONS_H_
