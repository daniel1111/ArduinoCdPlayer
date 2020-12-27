/* #######################################################################

    IDE ATAPI controller and DAC for an Arduino Pro-Mini

    Copyright (C) 2020  Daniel Swann

    Heavily based on the ATAPIDUINO project:
    Copyright (C) 2012  Carlos Durandal

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

 * ####################################################################### */

#include "ide_io.h"

#define IO_EXP_DATALINES 0x22
#define IO_EXP_REG_SEL   0x20

#define CTL_MASK 0xE0

#define NXP_INPUT      0
#define NXP_OUTPUT     2
#define NXP_INVERT     4
#define NXP_CONFIG     6

// Corresponds to the bit position on the IO_EXP_REG_SEL port expander
#define CTL_A0    0
#define CTL_A1    1
#define CTL_A2    2
#define CTL_CS1FX 3
#define CTL_CS3FX 4
#define CTL_RST   5
#define CTL_DIOW  6
#define CTL_DIOR  7

ide_io::ide_io()
{
  _control_reg  = 0xF8;
  _debug_ide_rw = 0;
}

ide_io::data16 ide_io::read(byte regval)
{
  data16 retval;

  if (_debug_ide_rw)
  {
    Serial.print(F("--- IDE READ regval="));
    Serial.print(regval);
  }

  // Set datalines to inputs
  port_exp_write(IO_EXP_DATALINES, NXP_CONFIG  , 0xFF);
  port_exp_write(IO_EXP_DATALINES, NXP_CONFIG+1, 0xFF);

  regval = regval & ~CTL_MASK;                    // Don't allow regval to set DIOR/DIOW/RST
  _control_reg = _control_reg & CTL_MASK;         // Clear CS3FX/CS1FX/A2/A1/A0
  _control_reg = _control_reg | regval;           // Set CS3FX/CS1FX/A2/A1/A0 from regval passed in
  _control_reg = _control_reg & ~(1 << CTL_DIOR); // Set DIOR low

  port_exp_write(IO_EXP_REG_SEL, NXP_OUTPUT+1, _control_reg);

  retval.low  = port_exp_read(IO_EXP_DATALINES, NXP_INPUT);
  retval.high = port_exp_read(IO_EXP_DATALINES, NXP_INPUT + 1);

  _control_reg = _control_reg | (1 << CTL_DIOR); // Set DIOR high to signal end of read
  port_exp_write(IO_EXP_REG_SEL, NXP_OUTPUT+1, _control_reg);

  if (_debug_ide_rw)
  {
    Serial.print(F(". Finished IDE READ"));
    Serial.print(F(", dataLval="));
    Serial.print(retval.low);
    Serial.print(F(", dataHval = "));
    Serial.println(retval.high);
  }

  return retval;
}

void ide_io::write (byte regval, byte dataLval, byte dataHval)
{
  uint8_t reg_inital = _control_reg;

  regval = regval & ~CTL_MASK;                    // Don't allow regval to set DIOR/DIOW/RST
  _control_reg = _control_reg & CTL_MASK;         // Clear CS3FX/CS1FX/A2/A1/A0
  _control_reg = _control_reg | regval;           // Set CS3FX/CS1FX/A2/A1/A0 from regval passed in

  // Make sure DIOW is high to start with
  _control_reg = _control_reg | (1 << CTL_DIOW);
  port_exp_write(IO_EXP_REG_SEL  , NXP_OUTPUT+1, _control_reg);  // sets register address and DIOW low

  if (_debug_ide_rw)
  {
    Serial.print(F("--- IDE WRITE _control_reg inital="));
    Serial.print(reg_inital, BIN);
    Serial.print(F(", regval="));
    Serial.print(regval, BIN);
    Serial.print(F(", NEW _control_reg="));
    Serial.print(_control_reg, BIN);
    Serial.print(F(", dataLval="));
    Serial.print(dataLval);
    Serial.print(F(", dataHval = "));
    Serial.print(dataHval);
  }

  // set datalines as outputs
  port_exp_write(IO_EXP_DATALINES, NXP_CONFIG  , 0x00);
  port_exp_write(IO_EXP_DATALINES, NXP_CONFIG+1, 0x00);

  // write out data
  port_exp_write(IO_EXP_DATALINES, NXP_OUTPUT  , dataLval);
  port_exp_write(IO_EXP_DATALINES, NXP_OUTPUT+1, dataHval);

  // Set DIOW low to signal write
  _control_reg = _control_reg & ~(1 << CTL_DIOW);
  port_exp_write(IO_EXP_REG_SEL  , NXP_OUTPUT+1, _control_reg);

   // Set DIOW high to signal end of write
  _control_reg = _control_reg | (1 << CTL_DIOW);
  port_exp_write(IO_EXP_REG_SEL, NXP_OUTPUT+1, _control_reg);

  // set datalines back to inputs (highz)
  port_exp_write(IO_EXP_DATALINES, NXP_CONFIG  , 0xFF);
  port_exp_write(IO_EXP_DATALINES, NXP_CONFIG+1, 0xFF);

  if (_debug_ide_rw)
  {
    Serial.print(F(". IDE WRITE Done, set control reg = "));
    Serial.println(_control_reg, BIN);
  }
}

void ide_io::reset()
{
  port_exp_write(IO_EXP_REG_SEL, NXP_CONFIG+1  , 0x00);

  // Set RST low
  port_exp_write(IO_EXP_REG_SEL, NXP_OUTPUT+1, (byte)B11011111);

  delay(40);

  // Set RST high
  _control_reg = 0xFF;
  port_exp_write(IO_EXP_REG_SEL, NXP_OUTPUT+1, _control_reg);

  delay(20);
}

void ide_io::port_exp_write(uint8_t address, uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  int error = Wire.endTransmission();
  if (error)
  {
    Serial.print(F("I2C write error: "));
    Serial.println(error);
  }
}

uint8_t ide_io::port_exp_read(uint8_t address, uint8_t reg)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission();

  if (Wire.requestFrom((int)address, 1) != 1)
  {
    Serial.println(F("I2C read error"));
    return 255;
  }

  return Wire.read();
}
