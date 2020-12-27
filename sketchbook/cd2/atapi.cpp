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

#include "atapi.h"

// IDE Register addresses - table 6, p59 & p68
const byte DataReg = 0xF0;         // Addr. Data register of IDE device.
const byte ErrFReg = 0xF1;         // Addr. Error/Feature (rd/wr) register of IDE device.
const byte SecCReg = 0xF2;         // Addr. Sector Count register of IDE device.
const byte SecNReg = 0xF3;         // Addr. Sector Number register of IDE device.
const byte CylLReg = 0xF4;         // Addr. Cylinder Low register of IDE device.
const byte CylHReg = 0xF5;         // Addr. Cylinder High register of IDE device.
const byte HeadReg = 0xF6;         // Addr. Device/Head register of IDE device. aka "Drive Select"
const byte ComSReg = 0xF7;         // Addr. Command/Status (wr/rd) register of IDE device. aka "ATAPI Status" (p69)
const byte AStCReg = 0xEE;         // Addr. Alternate Status/Device Control (rd/wr) register of IDE dev


atapi::atapi(ide_io *ide_io)
{
  _ide = ide_io;
  memset(&_toc, 0, sizeof(_toc));
}

void atapi::reset()
{
  _ide->reset();                            // Do hard reset
  delay(3000);                              // This delay waits for the drive to initialise
  BSY_clear_wait();                         // The ATAPI spec. allows drives to take up to
  DRY_set_wait();                           // 31 sec. but all tested where alright within 3s.
}

bool atapi::is_atapi_device()
{
  ide_io::data16 read_val = _ide->read(CylLReg);            // Check device signature for ATAPI capability

  if (read_val.low == 0x14)         // See "5.18.3 Special Handling of ATA Read and Identify Drive Commands", p61
  {
    read_val = _ide->read(CylHReg);
    if (read_val.low == 0xEB)
    {
      return true;
    }
  }
  return false;
}

void atapi::init_task_file()
{
  _ide->write(ErrFReg, 0x00, 0xFF);            // Set Feature register = 0 (no overlapping and no DMA)
  _ide->write(CylHReg, 0x02, 0xFF);            // Set PIO buffer to max. transfer length (= 200h)
  _ide->write(CylLReg, 0x00, 0xFF);
  _ide->write(AStCReg, 0x02, 0xFF);            // Set nIEN, we don't care about the INTRQ signal
  BSY_clear_wait();                            // When conditions are met then IDE bus is idle,
  DRQ_clear_wait();                            // this check may not be necessary (???)
}

bool atapi::run_self_diag()
{
  ide_io::data16 read_val;
  _ide->write(ComSReg, 0x90, 0xFF);            // Issue Run Self Diagnostic Command
  read_val = _ide->read(ErrFReg);

  return (read_val.low == 0x01);
}

void atapi::identify_drive(struct identify_drive_details *drive_details)
{
  uint8_t cnt = 0;
  uint8_t model_idx = 0;
  ide_io::data16 read_val;

  memset(drive_details, 0, sizeof(struct identify_drive_details));

  _ide->write(ComSReg, 0xA1, 0xFF);           // Issue Identify Device Command     "0xA1 = ATAPI Identify Device". 7.1.7, p76
  delay(500);                                 // Instead of wait for IRQ. Needed by some dev.  
  do
  {
    read_val = _ide->read(DataReg);
  /*  Spec seems to suggest that all CD-ROM drives use 12 byte packets? (p78) so not bothering to support 16 byte packets
    if (cnt == 0)                                // Get supported packet lenght
    {
      if(read_val.low & (1<<0))                      // contained in lower byte of first word
      {
        paclen = 16;                              // 1st bit set -> use 16 byte packets
      }
    } */

    if ((cnt > 26) && (cnt < 47))                      // Read Model
    {
      drive_details->model[model_idx++] = read_val.high;
      drive_details->model[model_idx++] = read_val.low;
    }
    cnt++;

    read_val = _ide->read(ComSReg);                    // Read Status Register and check DRQ,
  } while (read_val.low & (1<<3));                     // skip rest of data until DRQ=0
}

// 10.8.20 REQUEST SENSE Command, p193
void atapi::request_sense(struct sense_data *sd)
{
  ide_io::data16 read_val;
  byte packet[12] = {0};
  memset(sd, 0, sizeof(struct sense_data));

  packet[0] = 0x03;
  packet[4] = 0xFF;

  send_packet(packet, sizeof(packet));
  delay(10);
  DRQ_set_wait();

  uint8_t cnt=0;
  do
  {
    read_val = _ide->read(DataReg);

    if (cnt == 1)
      sd->sense_key = read_val.low;

    if (cnt == 6)
      sd->asc = read_val.low;                          // Store Additional Sense Code

    cnt++;
    read_val = _ide->read(ComSReg);
  } while (read_val.low & (1<<3));                  // Skip rest of packet
}

// 10.8.9 PLAY AUDIO MSF Command (p142)
void atapi::play(uint8_t track)
{
  byte packet[12] = {0};
  struct msf_address *track_addr;

  if (track != 0xFF)
  {
    if (!get_track_address(track, &track_addr))
    {
      Serial.print(F("Play failed: didn't find track "));
      Serial.print(track);
      Serial.println(F(" in TOC"));
      return;
    }
  }

  packet[0] = 0x47; // operation code

  // set start position
  if (track == 0xFF)
  {
    // Play from current position
    // The spec suggests that sending M/S/F all set to 0xFF here should work, but it doesn't seem to... so get the current location and send that
    struct atapi::sub_channel_data sub_ch_data;

    // Get current position
    read_subchannel(&sub_ch_data);
    packet[3] = sub_ch_data.absolute_address.M;
    packet[4] = sub_ch_data.absolute_address.S;
    packet[5] = sub_ch_data.absolute_address.F;
  }
  else
  {
    packet[3] = track_addr->M;
    packet[4] = track_addr->S;
    packet[5] = track_addr->F;
  }

  // end position (end of disc)
  packet[6] = _toc.lead_out->address.M;
  packet[7] = _toc.lead_out->address.S;
  packet[8] = _toc.lead_out->address.F;

  send_packet(packet, sizeof(packet));
}

// "10.8.18 READ SUB-CHANNEL Command", p175
void atapi::read_subchannel(struct sub_channel_data *data)
{
  byte packet[12] = {0};
  ide_io::data16 read_val;
  memset(data, 0, sizeof(struct sub_channel_data));

  packet[0] = 0x42;
  packet[1] = 0x02;  // MSF  = 1
  packet[2] = 0x40;  // SubQ = 1
  packet[3] = 0x01;  // Sub-channel Data Format = 1 (CD-ROM current position)
  packet[7] = 0xFF;  // Alocation length
  packet[8] = 0xFF;  // ""

  send_packet(packet, sizeof(packet));

  uint8_t cnt=0;
  do
  {
    read_val = _ide->read(DataReg);

    switch (cnt)
    {
      case 0:
        data->audio_status = read_val.high;
        break;

      case 2:
        data->ADR     = (read_val.high & 0xF0) >> 4;
        data->control = (read_val.high & 0x0F);
        break;

      case 3:
        data->track_number = read_val.low;
        data->index_number = read_val.high;
        break;

      case 4:
        // read_val.low = reserved
        data->absolute_address.M = read_val.high;
        break;

      case 5:
        data->absolute_address.S = read_val.low;
        data->absolute_address.F = read_val.high;
        break;

      case 6:
        // read_val.low = reserved
        data->relative_address.M = read_val.high;
        break;

      case 7:
        data->relative_address.S = read_val.low;
        data->relative_address.F = read_val.high;
        break;
    }

    cnt++;
    read_val = _ide->read(ComSReg);
    if (cnt > 16)
    {
      Serial.print(F("Get sub chan data: ERROR >16 bytes received, but DRQ still set. reg="));
      Serial.println(read_val.low, BIN);
      break;
    }
  } while (read_val.low & (1<<3));                  // Skip rest of packet
}

// 10.8.19 READ TOC Command, p183
void atapi::read_toc()
{
  uint8_t toc_idx = 0;
  byte packet[12] = {0};
  ide_io::data16 read_val;
  memset(&_toc, 0, sizeof(_toc));

  packet[0] = 0x43;
  packet[1] = 0x02;  // MSF  = 1
  packet[7] = 0xFF;  // Alocation length
  packet[8] = 0xFF;  // ""

  send_packet(packet, sizeof(packet));

  read_val = _ide->read(DataReg); // get toc data length (don't care)
  read_val = _ide->read(DataReg); // get first + last track
  _toc.first_track = read_val.low;
  _toc.last_track  = read_val.high;

  // Get TOC Track Descriptors: descriptor is 8 bytes (so 4 x 16bit reads)
  read_val = _ide->read(ComSReg);
  while (read_val.low & (1<<3))     // Read data from DataRegister until DRQ=0
  {
    read_val = _ide->read(DataReg); // reserved, ADR & control (don't care. yet.)

    read_val = _ide->read(DataReg); // track number, reserved
    _toc.track[toc_idx].track_number = read_val.low;

    read_val = _ide->read(DataReg); // reserved, address:M
    _toc.track[toc_idx].address.M = read_val.high;

    read_val = _ide->read(DataReg); // address:S, address:F
    _toc.track[toc_idx].address.S = read_val.low;
    _toc.track[toc_idx].address.F = read_val.high;

    if (_toc.track[toc_idx].track_number == 0xAA) // "A track number of 0AAh indicates that the track descriptor is for the start of the lead-out area"
      _toc.lead_out = &_toc.track[toc_idx];

    if (toc_idx < MAX_TOC_LENGTH)
      toc_idx++;
    else
    {
      Serial.print(F("Read TOC: reached MAX_TOC_LENGTH, but DRQ still 0. reg="));
      Serial.println(read_val.low, BIN);
      break;
    }

    read_val = _ide->read(ComSReg);
  }
}

void atapi::prev()
{
  struct atapi::sub_channel_data sub_ch_data;

  // Get current position
  read_subchannel(&sub_ch_data);

  // If not already on first track, play prev track
  if (sub_ch_data.track_number > _toc.first_track)
    play(sub_ch_data.track_number - 1);
}

void atapi::next()
{
  struct atapi::sub_channel_data sub_ch_data;

  // Get current position
  read_subchannel(&sub_ch_data);

  // If not already on last track, play next track
  if (sub_ch_data.track_number < _toc.last_track)
    play(sub_ch_data.track_number + 1);
}

// 10.8.24 STOP PLAY / SCAN CD-ROM Command (p209)
void atapi::stop()
{
  byte packet[12] = {0};

  packet[0] = 0x4E; // operation code

  send_packet(packet, sizeof(packet));
}

void atapi::pause_toggle()
{
  uint8_t resume = 0;
  struct atapi::sub_channel_data sub_ch_data;

  // Get current state
  read_subchannel(&sub_ch_data);

  if (sub_ch_data.audio_status == sub_channel_data::PLAY_IN_PROGRESS)
    resume = 0;
  else if (sub_ch_data.audio_status == sub_channel_data::PLAY_PAUSED)
    resume = 1;
  else
    return;

  pause(resume);
}

// 10.8.7 PAUSE/RESUME Command (p137)
void atapi::pause(bool resume)
{
  byte packet[12] = {0};
  packet[0] = 0x4B;   // operation code
  packet[8] = resume;

  send_packet(packet, sizeof(packet));
}

// 10.8.21 SCAN Command (p201)
void atapi::scan(bool reverse)
{
  byte packet[12] = {0};
  struct atapi::sub_channel_data sub_ch_data;

  // Get current position
  read_subchannel(&sub_ch_data);

  packet[0] = 0xBA;             // operation code
  packet[1] = (reverse << 4);   // DIRECTion
  packet[2] = 0x00;
  packet[3] = sub_ch_data.absolute_address.M;
  packet[4] = sub_ch_data.absolute_address.S;
  packet[5] = sub_ch_data.absolute_address.F;
  packet[9] = (1 << 6);         // Type = AMIN, ASEC and AFRAME format

  send_packet(packet, sizeof(packet));
}

// 10.8.25 START/STOP UNIT Command, p211
void atapi::eject()
{
  byte packet[12] = {0};
  uint8_t load = 0;

  // First, see if the door is already open
  if (mode_sense_media_status() == atapi::MediaStatus::DOOR_OPEN)
  {
    Serial.println(F("DOOR OPEN"));
    load = 1;
  }

  packet[0] = 0x1B;   // operation code
  packet[4] =
    (1 << 1) |    // LoEj
    (load << 0);  // Start

  send_packet(packet, sizeof(packet));
}

// 0.8.5 MODE SENSE Command (p119)
atapi::MediaStatus atapi::mode_sense_media_status()
{
  ide_io::data16 read_val;
  uint8_t media_typ;
  byte packet[12] = {0};
  packet[0] = 0x5A;   // Operation code
  packet[2] = 0x01;   // Page Code = Read error recovery page
  packet[7] = 0xFF;   // Alocation length
  packet[8] = 0xFF;   // ""

  send_packet(packet, sizeof(packet));
  delay(10);
  DRQ_set_wait();
  read_val = _ide->read(DataReg); // get Mode Data Length (don't care)
  read_val = _ide->read(DataReg); // get Medium Type (see "Table 59 - CD-ROM Media Type Codes", p123)
  media_typ = read_val.low;

  // read and discard the rest of the data
  uint8_t count=0;
  do
  {
    read_val = _ide->read(DataReg);
    read_val = _ide->read(ComSReg);

    count++;
    if (count > 200)
    {
      Serial.print(F("mode_sense_media_status: count>200 but DRQ still 0"));
      break;
    }
  } while (read_val.low & (1<<3)); // Read data from DataRegister until DRQ=0

  return decode_media_type_code(media_typ);
}

//////////////////// Private ////////////////////

// Wait for BSY clear
void atapi::BSY_clear_wait()
{
  ide_io::data16 read_val;
  do
  {
    read_val = _ide->read(ComSReg); // 0xF7 (247)
  } while (read_val.low & (1<<7));
}

// Wait for DRQ clear
void atapi::DRQ_clear_wait()
{
  ide_io::data16 read_val;
  do
  {
    read_val = _ide->read(ComSReg);
  } while (read_val.low  & (1<<3));
}

// Wait for DRQ set
void atapi::DRQ_set_wait()
{
  ide_io::data16 read_val;
     do
     {
        read_val = _ide->read(ComSReg);
     } while ((read_val.low  & ~(1<<3)) == true);
}

// Wait for DRY set
void atapi::DRY_set_wait()
{
  ide_io::data16 read_val;
  do
  {
    read_val = _ide->read(ComSReg);
  } while ((read_val.low  & ~(1<<6)) == true);
}

void atapi::send_packet(byte *packet, uint8_t packet_length)
{
  byte dataLval;
  byte dataHval;

  _ide->write(AStCReg, B00001010, 0xFF);      // Set nIEN before you send the PACKET command!
  _ide->write(ComSReg, 0xA0, 0xFF);           // Write Packet Command Opcode

  for (int idx=0; idx < packet_length; idx+=2)         // Send packet with length of 'paclen'
  {                                                    // to IDE Data Registeraccording to idx value
    dataLval = packet[idx];
    dataHval = packet[idx + 1];
    _ide->write(DataReg, dataLval, dataHval);
  }
  BSY_clear_wait();
}

bool atapi::get_track_address(uint8_t track, struct msf_address **track_addr)
{
  for (uint8_t idx = 0; idx < MAX_TOC_LENGTH; idx++)
  {
    if (_toc.track[idx].track_number == track)
    {
      *track_addr = &_toc.track[idx].address;
      return true;
    }
  }

  return false;
}

// See "Table 59 - CD-ROM Media Type Codes", p123
atapi::MediaStatus atapi::decode_media_type_code(uint8_t status)
{
  if (status == 0x71)
    return atapi::MediaStatus::DOOR_OPEN;
  if (status == 0x70)
    return atapi::MediaStatus::DOOR_CLOSED_NO_DISC;
  if (status == 0x00) // Some CD-ROM drives seem to report this for audio (and probably any?) disc types
    return atapi::MediaStatus::DOOR_CLOSED_UNKNOWN_DISC; 
  if (status & (1 << 6)) // Bit 6 seems to mean error
    return atapi::MediaStatus::DOOR_CLOSED_INVALID_DISC;
  if (status & (1 << 1))
    return atapi::MediaStatus::DOOR_CLOSED_VALID_DISC;

  return atapi::MediaStatus::DOOR_CLOSED_INVALID_DISC;
}
