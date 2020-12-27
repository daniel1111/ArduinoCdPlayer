#include "wm880x.h"


wm880x::wm880x(uint8_t devaddr)
{
  _address = devaddr;
}

void wm880x::init()
{
  byte error;
  Wire.beginTransmission(_address);
  error = Wire.endTransmission();

  if (error == 0)
  {
    Serial.println(F("WM8804: found"));
  }
  else if (error==4) 
  {
    Serial.println(F("WM8804: Unknown error at address 0x3A"));
  }
  else
  {
    Serial.println(F("WM8804: No response!"));
  }

  Serial.print(F("WM8804: Device ID: "));
  byte c = read_register(_address, 1);
  if (c < 10)
  {
    Serial.print('0');
  }
  Serial.print(c,HEX);

  c = read_register(_address, 0);
  if (c < 10) {
    Serial.print('0');
  }
  Serial.print(c,HEX);

  Serial.print(F(" Rev. "));
  c = read_register(_address, 2);
  Serial.println(c,HEX);

  loop();
  DeviceInit(_address);
  loop();
}

void wm880x::loop()
{
  _spdstat = read_register(_address, 0x0C);
}

bool wm880x::is_deemph()
{
  return (_spdstat & (1 << REG_SPDSTAT_DEEMPH));
}

byte wm880x::read_register(int devaddr, int regaddr)
{
  // Read a data register value
  Wire.beginTransmission(devaddr);
  Wire.write(regaddr);
  Wire.endTransmission(false);  // repeated start condition: don't send stop condition, keeping connection alive.
  Wire.requestFrom(devaddr, 1); // only one byte
  byte data = Wire.read();
  Wire.endTransmission(true);
  return data;
}

void wm880x::write_register(int devaddr, int regaddr, int dataval)
{
  // Write a data register value
  Wire.beginTransmission(devaddr); // device
  Wire.write(regaddr); // register
  Wire.write(dataval); // data
  Wire.endTransmission(true);
}

void wm880x::DeviceInit(int devaddr)
{                                              // resets, initializes and powers a wm8804
  // reset device
  write_register(devaddr, 0, 0);

  // REGISTER 7
  // bit 7:6 - always 0
  // bit 5:4 - CLKOUT divider select => 00 = 512 fs, 01 = 256 fs, 10 = 128 fs, 11 = 64 fs
  // bit 3 - MCLKDIV select => 0
  // bit 2 - FRACEN => 1
  // bit 1:0 - FREQMODE (is written by S/PDIF receiver) => 00
  write_register(devaddr, 7, B00000100);

  // REGISTER 8
  // set clock outputs and turn off last data hold
  // bit 7 - MCLK output source select is CLK2 => 0
  // bit 6 - always valid => 0
  // bit 5 - fill mode select => 1 (we need to see errors when they happen)
  // bit 4 - CLKOUT pin disable => 1
  // bit 3 - CLKOUT pin select is CLK1 => 0
  // bit 2:0 - 001 = RX1
  write_register(devaddr, 8, B00110001);

  // set the AIF RX
  // bit   7 - SYNC => 1
  // bit   6 - master mode => 1
  // bit   5 - LRCLK polarity => 0
  // bit   4 - BCLK invert => 0
  // bit 3:2 - data word length => 10 (24b) or 00 (16b)
  // bit 1:0 - format select: 11 (dsp), 10 (i2s), 01 (LJ), 00 (RJ)
  write_register(devaddr, 28, B11000010);

  // power up device
  write_register(devaddr, 30, 0);
}
