/*
 * clsPCA9555.h
 *
 *  Created on: 27 jul. 2015
 *      Author: Nico
 */

#ifndef CLSPCA9555_H_
#define CLSPCA9555_H_

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define DEBUG 1

/** enum with names of ports ED0 - ED15 */
enum {
    ED0, ED1, ED2 , ED3 , ED4 , ED5 , ED6 , ED7 ,
    ED8, ED9, ED10, ED11, ED12, ED13, ED14, ED15
};

/** enum with names of the ports as they're referred to on the TI datasheet */
enum {
    P00, P01, P02, P03, P04, P05, P06, P07,
    P10, P11, P12, P13, P14, P15, P16, P17,
};

//
// PCA9555 defines
//
#define NXP_INPUT      0
#define NXP_OUTPUT     2
#define NXP_INVERT     4
#define NXP_CONFIG     6

class PCA9555 {
public:
    PCA9555(uint8_t address, int interruptPin = -1);     // optional interrupt pin in second argument
    void pinMode(uint8_t pin, uint8_t IOMode );          // pinMode
    uint8_t digitalRead(uint8_t pin);                    // digitalRead
    void digitalWrite(uint8_t pin, uint8_t value );      // digitalWrite
    uint8_t stateOfPin(uint8_t pin);                     // Actual ISR
    void setClock(uint32_t clockFrequency);              // Clock speed
    bool begin();                                        // Checks if PCA is responsive

private:
    static PCA9555* instancePointer;
    static void alertISR(void); // Function pointing to actual ISR
    void pinStates();           // Function tied to interrupt

    //
    // low level methods
    //
    uint16_t I2CGetValue(uint8_t address, uint8_t reg);
    void I2CSetValue(uint8_t address, uint8_t reg, uint8_t value);

    union {
        struct {
            uint8_t _stateOfPins_low;          // low order byte
            uint8_t _stateOfPins_high;         // high order byte
        };
        uint16_t _stateOfPins;                 // 16 bits presentation
    };
    union {
        struct {
            uint8_t _configurationRegister_low;          // low order byte
            uint8_t _configurationRegister_high;         // high order byte
        };
        uint16_t _configurationRegister;                 // 16 bits presentation
    };
    union {
        struct {
            uint8_t _valueRegister_low;                  // low order byte
            uint8_t _valueRegister_high;                 // high order byte
        };
        uint16_t _valueRegister;
    };
    uint8_t _address;                                    // address of port this class is supporting
    int _error;                                          // error code from I2C
};


#endif /* CLSPCA9555_H_ */
