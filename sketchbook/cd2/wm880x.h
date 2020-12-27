#ifndef WM880X_H_
#define WM880X_H_

#include "Arduino.h"
#include <Wire.h> 

// Page numbers refer to the WM8805 datasheet "8:1 Digital Interface Transceiver with PLL", "Production Data, March 09, Rev 4.5", "WM8805_v4.5-1141952.pdf"


// S/PDIF Status Register, p42
#define REG_SPDSTAT            0x0C
#define REG_SPDSTAT_AUDIO_N    0
#define REG_SPDSTAT_PCM_N      1
#define REG_SPDSTAT_CPY_N      2
#define REG_SPDSTAT_DEEMPH     3
#define REG_SPDSTAT_REC_FREQ1  4
#define REG_SPDSTAT_REC_FREQ2  5
#define REG_SPDSTAT_UNLOCK     6

class wm880x
{
  public:
    wm880x(uint8_t devaddr);
    void init();
    void loop();
    bool is_deemph();

  private:
    byte read_register(int devaddr, int regaddr);
    void write_register(int devaddr, int regaddr, int dataval);
    void DeviceInit(int devaddr);

    uint8_t _address;
    uint8_t _spdstat;

};

#endif
