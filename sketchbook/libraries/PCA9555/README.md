# PCA9555
Arduino Library to approach PCA9555 I2C expander with digitalRead, digitalWrite, pinMode and functions.
Implemented begin() to check connection with PCA9555 and optional interrupt functionality.

##Installation:
* a) Just download the zip file
* b) move PCA9555_LIB to the Users Arduino library (usually My documents\Arduino\Libraries)
* c) Create a new sketch and copy the example file into your sketch
* d) Compile, download and done.

## Support functions:

* pinMode() same as standard Arduino
* digitalRead() same as Arduino
* digitalWrite() same as Arduino
* begin() checks if PCA9555 is responsive
* pinStates() ISR retrieves state of all I/O pins
* stateOfPin()  returns the state of a particular pin
* setClock() sets clock speed

## Additional
The code supports either pinnumbers 0 - 15 or ED0 - ED15. This to stay more or less in sync with Arduino.
alertISR() is used to point to the actual ISR for the use of attachInterrupt() from Arduino.
