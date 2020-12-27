
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "clsPCA9555.h"
#include "Wire.h"

PCA9555 ioport(0x20);

/**
 * @name setup()
 * initialize the program
 */
void setup()
{
	//
	// start I2C
	//
	ioport.begin();
	//
	// sets the I2C clock to 400kHz
	//
	ioport.setClock(400000);
	//
	// set first 14 pins to output
	//
	for (uint8_t i = 0; i < 14; i++){
		ioport.pinMode(i, OUTPUT);
	}
	//
	// set pin 15  (=digital 15) to Input
	//
	ioport.pinMode(ED14, INPUT);
}

/**
 * @name loop()
 * main loop of program and runs endlessly
 */
void loop()
{
	//
	// check if button is LOW
	//
	if (ioport.digitalRead(ED14) == LOW) {
		//
		// turn all 14 leds on
		//
		for (uint8_t i = 0; i < 14; i++){
			ioport.digitalWrite(i, HIGH);
			delay(50);
		}
		//
		// turn all 14 leds off
		//
		for (uint8_t i = 0; i < 14; i++){
			ioport.digitalWrite(i, LOW);
			delay(50);
		}
	}
}
