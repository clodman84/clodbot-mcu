#include "bsp/m5stack_core_s3.h"
#include "esp_codec_dev.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "include/audio_io.h"
#include "include/udp_audio.h"

#define SAMPLE_RATE   16000
#define FRAME_MS      20
#define FRAME_SAMPLES (SAMPLE_RATE / 1000 * FRAME_MS) // 320
#define MONO_BYTES    (FRAME_SAMPLES * 2)             // 640, what goes on the wire
#define PREBUFFER     3
#define MAX_FILL      3
#define SPEAKER_VOL   100 // 100 was for the test; lower if it's loud
#define MIC_GAIN_DB   30.0f

static const char            *TAG = "audio_io";
static esp_codec_dev_handle_t s_mic;
static esp_codec_dev_handle_t s_spk;

// Mic -> network. The codec gives interleaved stereo; the wire is mono.
static void
mic_task (void *arg)
{
  static int16_t st[FRAME_SAMPLES * 2]; // 1280 bytes
  static int16_t mono[FRAME_SAMPLES];   // 640 bytes
  while (1)
    {
      if (esp_codec_dev_read (s_mic, st, sizeof (st)) != ESP_CODEC_DEV_OK)
        {
          ESP_LOGE (TAG, "mic read failed");
          vTaskDelay (pdMS_TO_TICKS (10));
          continue;
        }
      for (int i = 0; i < FRAME_SAMPLES; i++)
        mono[i] = (int16_t) (((int) st[2 * i] + (int) st[2 * i + 1]) / 2);
      udp_audio_send (mono, sizeof (mono));
    }
}

// Network -> speaker. Wire is mono; the codec wants interleaved stereo.
static void
speaker_task (void *arg)
{
  static int16_t    silence[FRAME_SAMPLES * 2]; // zero-initialised
  static int16_t    st[UDP_AUDIO_MAX_PAYLOAD];
  udp_audio_frame_t f;
  bool              playing  = false;
  uint16_t          last_seq = 0;

  while (1)
    {
      if (! playing)
        {
          if (udp_audio_queued () < PREBUFFER)
            {
              vTaskDelay (pdMS_TO_TICKS (FRAME_MS));
              continue;
            }
          playing = true;
        }

      if (udp_audio_recv (&f, 100) != ESP_OK)
        {
          playing = false;
          continue;
        }

      int16_t gap = (int16_t) (f.seq - last_seq) - 1;
      if (gap > 0 && last_seq != 0)
        for (int i = 0; i < gap && i < MAX_FILL; i++)
          esp_codec_dev_write (s_spk, silence, sizeof (silence));
      last_seq = f.seq;

      const int16_t *in = (const int16_t *) f.data;
      int            n  = f.len / 2; // mono samples
      for (int i = 0; i < n; i++)
        st[2 * i] = st[2 * i + 1] = in[i];
      esp_codec_dev_write (s_spk, st, n * 4);
    }
}

esp_err_t
audio_io_start (void)
{
  s_spk = bsp_audio_codec_speaker_init ();
  s_mic = bsp_audio_codec_microphone_init ();
  if (s_spk == NULL || s_mic == NULL)
    {
      ESP_LOGE (TAG, "codec init failed");
      return ESP_FAIL;
    }

  esp_codec_dev_sample_info_t fs = {
    .bits_per_sample = 16,
    .channel         = 2, // the fix
    .sample_rate     = SAMPLE_RATE,
  };
  if (esp_codec_dev_open (s_mic, &fs) != ESP_CODEC_DEV_OK || esp_codec_dev_open (s_spk, &fs) != ESP_CODEC_DEV_OK)
    {
      ESP_LOGE (TAG, "codec open failed");
      return ESP_FAIL;
    }

  bsp_feature_enable (BSP_FEATURE_SPEAKER, true);
  esp_codec_dev_set_out_vol (s_spk, SPEAKER_VOL);
  esp_codec_dev_set_in_gain (s_mic, MIC_GAIN_DB);

  xTaskCreate (mic_task, "mic", 4096, NULL, 10, NULL);
  xTaskCreate (speaker_task, "spk", 4096, NULL, 11, NULL);
  return ESP_OK;
}
