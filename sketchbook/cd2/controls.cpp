#include "controls.h"


controls::controls()
{
  config_port_exp();
}

void controls::loop()
{
  _button_states = ~port_exp_read();
}

bool controls::current_button_state(enum Button button)
{
  return (_button_states & (1 << button));
}

bool controls::has_button_state_changed(enum Button button, bool *new_state)
{
  bool retval;
  bool last_button_state    = (_button_states_at_last_read & (1 << button));
  *new_state = current_button_state(button);

  retval = (last_button_state != *new_state);

  if (*new_state)
    _button_states_at_last_read |= (1 << button);
  else
    _button_states_at_last_read &= ~(1 << button);

  return retval;
}

bool controls::has_button_been_pressed(enum Button button)
{
  bool button_state;
  if (has_button_state_changed(button, &button_state))
  {
    if (button_state)
    {
      return true;
    }
  }

  return false;
}

void controls::config_port_exp()
{
  // Set everything to inputs
  Wire.beginTransmission(IO_EXP_ADDR);
  Wire.write(NXP_CONFIG);
  Wire.write(0xFF);
  Wire.endTransmission();
}

uint8_t controls::port_exp_read()
{
  Wire.beginTransmission(IO_EXP_ADDR);
  Wire.write(NXP_INPUT);
  Wire.endTransmission();

  if (Wire.requestFrom((int)IO_EXP_ADDR, 1) != 1)
  {
    Serial.println(F("I2C read error"));
    return 255;
  }

  return Wire.read();
}
