#include <arpa/inet.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "include/internet.h"
#include "include/udp_audio.h"

#define RX_TASK_PRIO     11 // above audio_task (10): wake promptly, do very little
#define TX_MUTEX_WAIT_MS 2  // never stall the audio loop on the tx mutex

_Static_assert (RXQ_DEPTH > MAX_FILL + 2, "RXQ_DEPTH must exceed MAX_FILL");

static const char       *TAG = "udp_audio";

static SemaphoreHandle_t s_mtx; // guards s_sock, s_tx_seq, s_tx; serialises senders
static QueueHandle_t     s_rxq;
static StaticQueue_t     s_rxq_struct;  // control block, internal RAM
static uint8_t          *s_rxq_storage; // item storage
static int               s_sock = -1;
static uint16_t          s_tx_seq;
static uint8_t           s_tx[HDR_BYTES + UDP_AUDIO_MAX_PAYLOAD];

// rx task only
static uint8_t           s_rx[HDR_BYTES + UDP_AUDIO_RX_MAX_PAYLOAD + 1];
static udp_audio_frame_t s_frame;

static portMUX_TYPE      s_ts_lock = portMUX_INITIALIZER_UNLOCKED;
static int64_t           s_last_rx_us; // 64-bit: read/written under s_ts_lock
static udp_audio_stats_t s_stats;

static int
open_socket (void)
{
  int s = socket (AF_INET, SOCK_DGRAM, IPPROTO_IP);
  if (s < 0)
    return -1;

  struct sockaddr_in dst = { .sin_family = AF_INET,
                             .sin_port   = htons (REMOTE_PORT) };
  inet_pton (AF_INET, REMOTE_HOST, &dst.sin_addr);

  // Sends nothing. Remembers the peer's address and ignores packets from
  // anyone else.
  if (connect (s, (struct sockaddr *) &dst, sizeof (dst)) < 0)
    {
      close (s);
      return -1;
    }

  struct timeval tv = { .tv_sec = 1 };
  setsockopt (s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof (tv));
  return s;
}

// Returns true if the frame should be kept.
static bool
track_sequence (uint16_t seq,
                bool     first)
{
  static uint16_t expected;
  if (first)
    {
      expected = seq + 1;
      return true;
    }

  int16_t diff = (int16_t) (seq - expected);

  // Sender restarted (its counter went back to ~0) or we missed a huge span.
  // Without this every frame after a PC restart looks "late" and is dropped.
  if (diff < -SEQ_RESYNC_WINDOW || diff > SEQ_RESYNC_WINDOW)
    {
      s_stats.rx_resync++;
      expected = seq + 1;
      xQueueReset (s_rxq); // stale audio from the old stream
      return true;
    }

  if (diff < 0)
    {
      s_stats.rx_late++; // old or duplicate: playing it would go backwards
      return false;
    }
  s_stats.rx_lost += diff; // diff == 0 means nothing missing
  expected = seq + 1;
  return true;
}

static void
push_frame (void)
{
  if (xQueueSend (s_rxq, &s_frame, 0) == pdTRUE)
    return;

  // Full: throw away the oldest so we keep the freshest audio.
  udp_audio_frame_t old;
  xQueueReceive (s_rxq, &old, 0);
  s_stats.rx_overrun++;
  xQueueSend (s_rxq, &s_frame, 0);
}

static void
rx_task (void *arg)
{
  while (1)
    {
      internet_wait_connected (portMAX_DELAY);
      int s = open_socket ();
      if (s < 0)
        {
          ESP_LOGE (TAG, "open failed: errno %d", errno);
          vTaskDelay (pdMS_TO_TICKS (RECONNECT_MS));
          continue;
        }

      xSemaphoreTake (s_mtx, portMAX_DELAY);
      s_sock = s;
      xSemaphoreGive (s_mtx);
      xQueueReset (s_rxq);
      ESP_LOGI (TAG, "socket up");

      bool first = true;
      while (1)
        {
          int n = recv (s, s_rx, sizeof (s_rx), 0);
          if (n < 0)
            {
              if (errno == EAGAIN || errno == EWOULDBLOCK)
                {
                  // 1 s of silence: make sure the network is still there.
                  if (! internet_wait_connected (0))
                    {
                      ESP_LOGW (TAG, "network down, closing socket");
                      break;
                    }
                  continue;
                }
              ESP_LOGE (TAG, "recv failed: errno %d", errno);
              break;
            }

          if (n <= HDR_BYTES)
            continue; // runt packet
          if (n > HDR_BYTES + UDP_AUDIO_RX_MAX_PAYLOAD)
            {
              s_stats.rx_oversize++;
              continue;
            }

          uint16_t seq = (uint16_t) (s_rx[0] | (s_rx[1] << 8)); // little-endian
          if (! track_sequence (seq, first))
            continue;
          first = false;

          s_frame.seq = seq;
          s_frame.len = n - HDR_BYTES;
          memcpy (s_frame.data, s_rx + HDR_BYTES, s_frame.len);

          portENTER_CRITICAL (&s_ts_lock);
          s_last_rx_us = esp_timer_get_time ();
          portEXIT_CRITICAL (&s_ts_lock);

          s_stats.rx_packets++;
          push_frame ();
        }

      xSemaphoreTake (s_mtx, portMAX_DELAY);
      s_sock = -1;
      close (s);
      xSemaphoreGive (s_mtx);
      vTaskDelay (pdMS_TO_TICKS (RECONNECT_MS));
    }
}

esp_err_t
udp_audio_send (const void *pcm,
                size_t      len)
{
  if (pcm == NULL || len == 0 || len > UDP_AUDIO_MAX_PAYLOAD)
    return ESP_ERR_INVALID_SIZE;

  // The caller is the real-time audio loop: if the mutex is busy (socket
  // being torn down) drop this packet instead of stalling the speaker.
  if (xSemaphoreTake (s_mtx, pdMS_TO_TICKS (TX_MUTEX_WAIT_MS)) != pdTRUE)
    {
      s_stats.tx_dropped++;
      return ESP_ERR_TIMEOUT;
    }

  esp_err_t err = ESP_OK;
  if (s_sock < 0)
    {
      s_stats.tx_dropped++;
      err = ESP_ERR_INVALID_STATE;
    }
  else
    {
      // Always advance the counter, even on failure, so the receiver can
      // see that a packet went missing.
      uint16_t seq = s_tx_seq++;
      s_tx[0]      = seq & 0xff;
      s_tx[1]      = seq >> 8;
      memcpy (s_tx + HDR_BYTES, pcm, len);

      // MSG_DONTWAIT: if lwIP/Wi-Fi has no buffer right now, drop, don't block.
      if (send (s_sock, s_tx, HDR_BYTES + len, MSG_DONTWAIT) < 0)
        {
          s_stats.tx_dropped++;
          err = (errno == ENOMEM || errno == EAGAIN) ? ESP_ERR_NO_MEM : ESP_FAIL;
        }
      else
        {
          s_stats.tx_packets++;
        }
    }
  xSemaphoreGive (s_mtx);
  return err;
}

esp_err_t
udp_audio_recv (udp_audio_frame_t *out,
                uint32_t           timeout_ms)
{
  return xQueueReceive (s_rxq, out,
                        timeout_ms == portMAX_DELAY ? portMAX_DELAY : pdMS_TO_TICKS (timeout_ms)) == pdTRUE ?
             ESP_OK :
             ESP_ERR_TIMEOUT;
}

uint32_t
udp_audio_trim (uint32_t keep)
{
  udp_audio_frame_t tmp;
  uint32_t          dropped = 0;
  while (uxQueueMessagesWaiting (s_rxq) > keep && xQueueReceive (s_rxq, &tmp, 0) == pdTRUE)
    dropped++;
  return dropped;
}

bool
udp_audio_is_open (void)
{
  return s_sock >= 0;
}

bool
udp_audio_peer_alive (uint32_t timeout_ms)
{
  portENTER_CRITICAL (&s_ts_lock);
  int64_t last = s_last_rx_us;
  portEXIT_CRITICAL (&s_ts_lock);
  return last != 0 && (esp_timer_get_time () - last) < (int64_t) timeout_ms * 1000;
}

void
udp_audio_get_stats (udp_audio_stats_t *out)
{
  *out = s_stats;
}

esp_err_t
udp_audio_start (void)
{
  s_mtx = xSemaphoreCreateMutex ();
  if (s_mtx == NULL)
    return ESP_ERR_NO_MEM;

  // Frames are small now (~260 B x 32 = ~8 KB): prefer internal RAM so the
  // audio path never waits on PSRAM; fall back to PSRAM if internal is tight.
  size_t bytes  = RXQ_DEPTH * sizeof (udp_audio_frame_t);
  s_rxq_storage = heap_caps_malloc (bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (s_rxq_storage == NULL)
    s_rxq_storage = heap_caps_malloc (bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (s_rxq_storage == NULL)
    {
      ESP_LOGE (TAG, "rx queue alloc failed");
      return ESP_ERR_NO_MEM;
    }
  s_rxq = xQueueCreateStatic (RXQ_DEPTH, sizeof (udp_audio_frame_t),
                              s_rxq_storage, &s_rxq_struct);
  if (s_rxq == NULL)
    return ESP_ERR_NO_MEM;

  return xTaskCreate (rx_task, "udp_audio_rx", 4096, NULL, RX_TASK_PRIO, NULL) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

uint32_t
udp_audio_queued (void)
{
  return uxQueueMessagesWaiting (s_rxq);
}
