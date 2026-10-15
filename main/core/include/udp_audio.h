#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

// Largest PCM payload per packet, in bytes. 2-byte header + this must stay
// under 1472. 960 bytes = 30 ms of 16 kHz / 16-bit / mono.
#define UDP_AUDIO_MAX_PAYLOAD 960
// Frames buffered between the network and your audio code. Each is ~1 KB.
#define UDP_AUDIO_QUEUE_DEPTH 24

  typedef struct
  {
    uint16_t seq; // sequence number from the sender
    uint16_t len; // PCM bytes in data
    uint8_t  data[UDP_AUDIO_MAX_PAYLOAD];
  } udp_audio_frame_t;

  typedef struct
  {
    uint32_t tx_packets;
    uint32_t tx_dropped; // send failed (Wi-Fi buffers full, socket closed)
    uint32_t rx_packets;
    uint32_t rx_lost;     // gaps in the sequence numbers
    uint32_t rx_late;     // arrived out of order, discarded
    uint32_t rx_overrun;  // queue was full, oldest frame discarded
    uint32_t rx_oversize; // packet bigger than UDP_AUDIO_MAX_PAYLOAD
  } udp_audio_stats_t;

  esp_err_t udp_audio_start (void); // non-blocking, spawns the rx task

  // Send one packet of PCM (len <= UDP_AUDIO_MAX_PAYLOAD). Never blocks long;
  // if it can't send, the packet is dropped and an error is returned.
  esp_err_t udp_audio_send (const void *pcm,
                            size_t      len);

  // Wait up to timeout_ms for the next received frame.
  // Returns ESP_OK or ESP_ERR_TIMEOUT.
  esp_err_t udp_audio_recv (udp_audio_frame_t *out,
                            uint32_t           timeout_ms);

  // True if the socket is open.
  bool      udp_audio_is_open (void);

  // True if a packet arrived within the last timeout_ms.
  bool      udp_audio_peer_alive (uint32_t timeout_ms);

  void      udp_audio_get_stats (udp_audio_stats_t *out);

  uint32_t  udp_audio_queued (void);

#ifdef __cplusplus
}
#endif
