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
#include "atapi.h"
#include "controls.h"
#include "wm880x.h"
#include "display.h"

enum ScanState
{
  RR,
  FF,
  NONE
};

char get_state_char(struct atapi::sub_channel_data *sub_ch_data);
void reread_toc_on_disc_load(atapi::MediaStatus media_status);
void scan(enum ScanState scan);

#define ADDRESS_WM880x  0x3a
#define ADDRESS_DISPLAY 0x3C
#define PIN_DEEMPH 5

enum ScanState _scan_state;
ide_io *_ide   = new ide_io();
atapi  *_atapi = new atapi(_ide);
controls *_controls;
wm880x *_wm880x;
display *_display;

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  Serial.println("CD Start");

  _display = new display(ADDRESS_DISPLAY);
  _display->init();

  _controls = new controls();
  _wm880x = new wm880x(ADDRESS_WM880x);
  _wm880x->init();

  pinMode(PIN_DEEMPH, OUTPUT);
  digitalWrite(PIN_DEEMPH, LOW);

  ide_io::data16 read_val;
  _scan_state = ScanState::NONE;

  _atapi->reset();

  if (_atapi->is_atapi_device())
    Serial.println("Found ATAPI Dev.");
  else
  {
    Serial.println("No ATAPI Device!");
    while(1);                          // No need to go ahead.
  }

  // Initialise task file
  Serial.println("Init task file");
  _atapi->init_task_file();
  Serial.println("Init task file done");

  // Run Self Diagnostic
  delay(3000);
  Serial.print("Self Diag: ");

  if (_atapi->run_self_diag())
  {
    Serial.println("OK");
  }
  else
  {
    Serial.println("Fail");            // Units failing this may still work fine
  }

  Serial.println("ATAPI Device:");

  // Identify Device
  struct atapi::identify_drive_details drive_details;
  _atapi->identify_drive(&drive_details);
  Serial.println(drive_details.model);

  struct atapi::sense_data sd;
  _atapi->request_sense(&sd); // Send packet 'Request Sense'

  if (sd.asc == 0x29)                                 // Req. Sense returns 'HW Reset' 
  {                                                   // (ASC=29h) at first since we had one.
    _atapi->request_sense(&sd);                       // New Req. Sense returns if media
  }                                                   // is present or not.

  do
  {
                                                  // Wait until drive is ready.
    _atapi->request_sense(&sd);                   // Some devices take some time
  } while (sd.asc == 0x04);                        // ASC=04h -> LOGICAL DRIVE NOT READY
}

// This part reads the push buttons, checks device audio status, interprets operator commands
// and displays the corresponding data depending on the status and/or the commands resulting
// from pressing the push buttons.
void loop()
{
  static unsigned long last_subchannel_read = 0;
  static unsigned long last_mode_sense = 0;
  static uint8_t current_track = 1;
  static atapi::MediaStatus media_status;

  _controls->loop();

  bool button_state;

  if (_controls->has_button_been_pressed(controls::STOP))
    _atapi->stop();

  if (_controls->has_button_been_pressed(controls::PLAY))
    _atapi->play(current_track);

  if (_controls->has_button_been_pressed(controls::PAUSE))
    _atapi->pause_toggle();

  if (_controls->has_button_been_pressed(controls::PREV))
    _atapi->prev();

  if (_controls->has_button_been_pressed(controls::NEXT))
    _atapi->next();

  if (_controls->has_button_state_changed(controls::FF, &button_state))
  {
    if (button_state)
      scan(ScanState::FF);
    else
      scan(ScanState::NONE);
  }

  if (_controls->has_button_state_changed(controls::RR, &button_state))
  {
    if (button_state)
      scan(ScanState::RR);
    else
      scan(ScanState::NONE);
  }

  if (_controls->has_button_been_pressed(controls::EJECT))
    _atapi->eject();

  struct atapi::sub_channel_data sub_ch_data;

  if (millis() - last_mode_sense > 1000)
  {
    last_mode_sense = millis();
    media_status = _atapi->mode_sense_media_status();
  }

  if (media_status == atapi::MediaStatus::DOOR_CLOSED_NO_DISC)
  {
    _display->show_message("NO DISC");
  }

  else if (media_status == atapi::MediaStatus::DOOR_OPEN)
  {
    _display->show_message("(OPEN)");
  }

  else if (millis() - last_subchannel_read > 100)
  {
    bool deemph;
    _wm880x->loop();
    deemph = _wm880x->is_deemph();
    if (deemph)
      digitalWrite(PIN_DEEMPH, HIGH);
    else
      digitalWrite(PIN_DEEMPH, LOW);

    _atapi->read_subchannel(&sub_ch_data);
    last_subchannel_read = millis();

    char state_char = get_state_char(&sub_ch_data);

    _display->show_position(sub_ch_data.track_number, sub_ch_data.relative_address.M, sub_ch_data.relative_address.S, deemph, state_char);
  }

  reread_toc_on_disc_load(media_status);
}

void scan(enum ScanState scan)
{
  switch (scan)
  {
    case ScanState::FF:
      _atapi->scan(0);
      break;

    case ScanState::RR:
      _atapi->scan(1);
      break;

    case ScanState::NONE:
      _atapi->play(0xFF);
      break;
  }

  _scan_state = scan;
}

char get_state_char(struct atapi::sub_channel_data *sub_ch_data)
{
  switch (sub_ch_data->audio_status)
  {
    case atapi::sub_channel_data::PLAY_PAUSED:
      return CHAR_PAUSE;

    case atapi::sub_channel_data::PLAY_IN_PROGRESS:
      if (_scan_state == ScanState::RR)
        return CHAR_RR;
      else if (_scan_state == ScanState::FF)
        return CHAR_FF;
      else
        return CHAR_PLAY;

    default:
      return CHAR_STOP;
  }

  return ' ';
}

void reread_toc_on_disc_load(atapi::MediaStatus media_status)
{
  static atapi::MediaStatus last_media_status = atapi::MediaStatus::DOOR_OPEN;

  if (last_media_status != media_status)
  {
    Serial.print(F("MediaStatus change "));
    Serial.print(last_media_status);
    Serial.print(F(" -> "));
    Serial.println(media_status);

    if (media_status == atapi::MediaStatus::DOOR_CLOSED_VALID_DISC || media_status == atapi::MediaStatus::DOOR_CLOSED_UNKNOWN_DISC)
    {
      Serial.println(F("(re)reading TOC"));
      _display->show_message("READING");
      _atapi->read_toc();
      Serial.print(F("First track="));
      Serial.print(_atapi->_toc.first_track);
      Serial.print(F(", last track="));
      Serial.println(_atapi->_toc.last_track);
    }

    last_media_status = media_status;
  }
}
