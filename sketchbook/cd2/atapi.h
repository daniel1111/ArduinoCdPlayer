#ifndef ATAPI_H_
#define ATAPI_H_

#include "Arduino.h"
#include "ide_io.h"

#define MAX_TOC_LENGTH 50

class atapi
{
  public:
      // This isn't directly mapped to anything in the spec, but is related to "Table 59 - CD-ROM Media Type Codes" (p123)
      enum MediaStatus
      {
        DOOR_CLOSED_VALID_DISC,
        DOOR_OPEN,
        DOOR_CLOSED_NO_DISC,
        DOOR_CLOSED_INVALID_DISC, // E.g. data only
        DOOR_CLOSED_UNKNOWN_DISC  // CD-ROM drive isn't reporting the media type, so it might be audio
      };

    // "Table 27 - MSF Address Format", p92
    struct msf_address
    {
      uint8_t M; // 0-99
      uint8_t S; // 0-59
      uint8_t F; // Frame 0-74
    };

    struct identify_drive_details
    {
      char model[41];
    };

    // See "Table 137 - Request Sense Standard Data", p194
    struct sense_data
    {
      uint8_t sense_key;
      uint8_t asc;        // Additional Sense Code
    };

    // See "Table 115 - CD-ROM Current Position Data Format (Format Code 01h)", p176
    struct sub_channel_data
    {
      enum AudioStatus
      {
        NOT_VALID               = 0x00,
        PLAY_IN_PROGRESS        = 0x11,
        PLAY_PAUSED             = 0x12,
        PLAY_COMPLETED_SUCCESS  = 0x13,
        PLAY_STOPPED_ERROR      = 0x14,
        NO_AUDIO_STATUS         = 0x14
      };
      uint8_t audio_status;
      uint8_t ADR;
      uint8_t control;
      uint8_t track_number;
      uint8_t index_number;
      struct msf_address absolute_address;
      struct msf_address relative_address;
    };

    struct toc_descriptor
    {
      uint8_t track_number;
      struct msf_address address;
    };

    struct
    {
      uint8_t first_track;
      uint8_t last_track;
      struct toc_descriptor track[MAX_TOC_LENGTH];
      struct toc_descriptor *lead_out;
    } _toc;

    atapi(ide_io *ideio);
    void reset();
    bool is_atapi_device();
    void init_task_file();
    bool run_self_diag();
    void identify_drive(struct identify_drive_details *drive_details);
    void request_sense(struct sense_data *sd);
    void play(uint8_t track);
    void read_subchannel(struct sub_channel_data *data);
    void read_toc();
    void prev();
    void next();
    void stop();
    void pause(bool resume);
    void pause_toggle();
    void scan(bool reverse);
    void eject();
    MediaStatus mode_sense_media_status();

  private:
    ide_io *_ide;

    void BSY_clear_wait();
    void DRQ_clear_wait();
    void DRQ_set_wait();
    void DRY_set_wait();
    void send_packet(byte *packet, uint8_t packet_length);
    bool get_track_address(uint8_t track, struct msf_address **track_addr);
    MediaStatus decode_media_type_code(uint8_t status);
};

#endif
