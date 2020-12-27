#ifndef CONTROLS_H_
#define CONTROLS_H_

#include "Arduino.h"
#include <Wire.h>

#define IO_EXP_ADDR    0x20
#define NXP_INPUT      0
#define NXP_CONFIG     6

class controls
{
  public:
      enum Button
      {
        STOP  = 0,
        PLAY  = 1,
        PAUSE = 2,
        PREV  = 3,
        NEXT  = 4,
        RR    = 5,
        FF    = 6,
        EJECT = 7
      };

    controls();
    void loop();
    bool current_button_state(enum Button button);
    bool has_button_state_changed(enum Button button, bool *new_state);
    bool has_button_been_pressed(enum Button button);

  private:
    void config_port_exp();
    uint8_t port_exp_read();
    uint8_t _button_states;
    uint8_t _button_states_at_last_read;
};

#endif
