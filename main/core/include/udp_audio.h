#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* Audio format / framing. Change FRAME_MS here and everything below scales. */
#define SAMPLE_RATE   16000
#define FRAME_MS      8
#define FRAME_SAMPLES (SAMPLE_RATE / 1000 * FRAME_MS) // 128 @ 8 ms, 256 @ 16 ms
#define MONO_BYTES    (FRAME_SAMPLES * 2)             // one channel on the wire
#define SPEAKER_VOL   100
#define MIC_GAIN_DB   10.0f

/* Jitter buffer, in milliseconds so it does not shrink when FRAME_MS does.
 * Must cover Wi-Fi burstiness (your stats showed ~20 ms jitter, 80 ms+ gaps). */
#define PREBUFFER_MS 60  // frames queued before playback starts
#define MAX_FILL_MS  120 // above this, oldest frames are dropped to bound latency
#define PREBUFFER    ((PREBUFFER_MS + FRAME_MS - 1) / FRAME_MS)
#define MAX_FILL     (MAX_FILL_MS / FRAME_MS)

#define REMOTE_HOST  "10.108.150.8"
#define REMOTE_PORT  8765
#define RECONNECT_MS 2000
#define HDR_BYTES    2
#define RXQ_DEPTH    32 // must be > MAX_FILL

/* TX: out | ref | mic | dref, one block each. */
#define UDP_AUDIO_MAX_PAYLOAD (MONO_BYTES * 4)
/* RX: the PC sends one mono block. Kept small so the queue stays small. */
#define UDP_AUDIO_RX_MAX_PAYLOAD MONO_BYTES

/* Sequence jumps bigger than this are treated as a sender restart, not loss. */
#define SEQ_RESYNC_WINDOW 200

  typedef struct
  {
    uint16_t seq; // sequence number from the sender
    uint16_t len; // PCM bytes in data
    uint8_t  data[UDP_AUDIO_RX_MAX_PAYLOAD];
  } udp_audio_frame_t;

  typedef struct
  {
    uint32_t tx_packets;
    uint32_t tx_dropped; // send failed (Wi-Fi buffers full, socket closed, mutex busy)
    uint32_t rx_packets;
    uint32_t rx_lost;     // gaps in the sequence numbers
    uint32_t rx_late;     // arrived out of order, discarded
    uint32_t rx_overrun;  // queue was full, oldest frame discarded
    uint32_t rx_oversize; // packet bigger than UDP_AUDIO_RX_MAX_PAYLOAD
    uint32_t rx_resync;   // sender restarted / big sequence jump
  } udp_audio_stats_t;

  esp_err_t udp_audio_start (void); // non-blocking, spawns the rx task

  // Send one packet of PCM (len <= UDP_AUDIO_MAX_PAYLOAD). Never blocks: if it
  // can't send right now, the packet is dropped and an error is returned.
  esp_err_t udp_audio_send (const void *pcm,
                            size_t      len);

  // Wait up to timeout_ms for the next received frame.
  // Returns ESP_OK or ESP_ERR_TIMEOUT.
  esp_err_t udp_audio_recv (udp_audio_frame_t *out,
                            uint32_t           timeout_ms);

  // Drop the oldest frames until at most `keep` remain. Returns how many dropped.
  uint32_t  udp_audio_trim (uint32_t keep);

  // True if the socket is open.
  bool      udp_audio_is_open (void);

  // True if a packet arrived within the last timeout_ms.
  bool      udp_audio_peer_alive (uint32_t timeout_ms);

  void      udp_audio_get_stats (udp_audio_stats_t *out);

  uint32_t  udp_audio_queued (void);

#ifdef __cplusplus
}
#endif
