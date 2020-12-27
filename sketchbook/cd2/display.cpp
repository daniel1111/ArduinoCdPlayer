#include "display.h"

static uint8_t _back_buffer[512];

display::display(uint8_t address)
{
  _address = address;
}


void display::init()
{
  int retval = oledInit(&_ssoled, OLED_128x32, -1, 0, 0, 1, -1, -1, -1, 400000L);       // Standard HW I2C bus at 400Khz
  if (retval == OLED_NOT_FOUND)
  {
    Serial.println(F("OLED not found!"));
  }
  else
  {
    oledFill(&_ssoled, 0, 1);
    oledWriteString(&_ssoled, 0, 0, 0, (char *)"CD INIT", FONT_STRETCHED, 0, 1);
    oledSetBackBuffer(&_ssoled, _back_buffer);
  }
}

void display::clear()
{
  oledFill(&_ssoled, 0, 1);
}

void display::show_position(uint8_t track, uint8_t M, uint8_t S, bool deemph, char state_char)
{
  char buffer[10] = {0};

  snprintf(buffer, sizeof(buffer)-1, "%02d %02d:%02d", track, M, S);
  oledFill(&_ssoled, 0, 0);
  oledWriteString(&_ssoled, 0, 0, 0, buffer, FONT_STRETCHED, 0, 0);

  if (deemph)
    oledWriteString(&_ssoled, 0, 21, 3, "DEEMPH", FONT_NORMAL, 0, 0);

  memset(buffer, 0, sizeof(buffer));
  buffer[0] = state_char;

  oledWriteString(&_ssoled, 0, 3, 2, buffer, FONT_STRETCHED, 0, 0);

  oledDumpBuffer(&_ssoled, _back_buffer);
}

void display::show_message(char *message)
{
  oledFill(&_ssoled, 0, 0);
  oledWriteString(&_ssoled, 0, 0, 0, message, FONT_STRETCHED, 0, 0);

  oledDumpBuffer(&_ssoled, _back_buffer);
}
