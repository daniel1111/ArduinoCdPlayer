#ifndef IDE_IO_H_
#define IDE_IO_H_

#include "Arduino.h"
#include <Wire.h> 

class ide_io
{
  public:
    union data16
    {
      struct
      {
        uint8_t low;    // low order byte
        uint8_t high;   // high order byte
      };
      uint16_t val;     // 16 bits presentation
    };

    ide_io();
    data16 read(byte regval);
    void write (byte regval, byte dataLval, byte dataHval);
    void reset();

  private:
    uint8_t _control_reg;
    bool _debug_ide_rw;

    void port_exp_write(uint8_t address, uint8_t reg, uint8_t value);
    uint8_t port_exp_read(uint8_t address, uint8_t reg);
};

#endif
