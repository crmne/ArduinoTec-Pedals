# ArduinoTec-Pedals
This project uses a Arduino Leonardo, Pro Micro or Teensy to replace the controller board of the Fanatec CSPV1.

See the [Circuit diagram](https://github.com/crmne/ArduinoTec-Pedals/blob/master/Circuit%20Diagram.PNG) for the wiring layout. _Note that the diagram is not updated to use the HX711 chip for the load cell brake yet_.

## Install dependencies

1. Install [Arduino IDE](https://www.arduino.cc/en/software)
2. Install the [Joystick Library](https://github.com/MHeironimus/ArduinoJoystickLibrary) by downloading the code as zip and in the Arduino IDE click on Sketch -> Include Library -> Add .ZIP Library.
3. Install the [HX711 Library](https://github.com/olkal/HX711_ADC) by installing HX711_ADC from the Tools -> Manage Libraries in the Arduino IDE.

## Hardware dependencies

In addition to this you would need:

1. [Arduino Leonardo](https://store.arduino.cc/usa/leonardo) or [Pro Micro 5V 16mhz](https://www.sparkfun.com/products/12640).
2. a small breadboard or Arduino Shield (available on ebay and similar to the one shown [here](https://www.adafruit.com/product/51)) to place it on if you don't want to solder the cables onto the board directly
3. Connectors if you want to be able to connect the load cells and HE sensors via a connector block
4. A HX711 chip for the load cell brake, the load cell, and two 1k ohm resistors. Connect them as per [this diagram](https://circuitjournal.com/50kg-load-cells-with-HX711) for single load cell.
5. I re-used the Fanatec Hall Effect sensors as mine were still in good working order. If you do want to use another sensor, ensure it is a linear sensor with Analog output that can sense the distance of the magnet/and field variation. Do not use arduino ones as they one sense on/off and not field variation.

Connect the items as per the diagram, write the code to the Arduino, plug into the USB, calibrate via driver calibration screen, and you are ready to race.

## Tuning notes

1. The fixed position of the stock magnet may make it difficult to determine where to place the replacement Hall Effect sensor. Another magnet will confirm the HE sensor is working as expected. Depending on the sensor used it may help to rotate it 90 degrees.
2. When determining the location of the HE sensor it helps to use reusable adhesive (e.g. Blu-Tack:registered:) to temporarily affix it. Depending on sensor packaging it should be electrically isolated to avoid contact with the metal pedal. This could be done with cardboard or some other material.
3. Alternate magnet and HE sensor locations can be considered. For example: a magnet moved to the bottom of the pedal and HE sensor affixed to the frame. This would be similar to how the Thrustmaster T-LCM pedals work. Another approach would be to place magnet and HE sensor so they are inline at either end of the compression spring.
