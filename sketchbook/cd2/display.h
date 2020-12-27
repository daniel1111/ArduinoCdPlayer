#ifndef DISPLAY_H_
#define DISPLAY_H_

#include <Arduino.h>
#include <ss_oled.h>

#define CHAR_STOP   123
#define CHAR_RR     124
#define CHAR_FF     125
#define CHAR_PAUSE  126
#define CHAR_PLAY   127

class display
{
  public:
    display(uint8_t address);
    void init();
    void clear();
    void show_position(uint8_t track, uint8_t M, uint8_t S, bool deemph, char state_char);
    void show_message(char *message);
    void loop();

  private:
    uint8_t _address;
    SSOLED _ssoled;
};

#endif

