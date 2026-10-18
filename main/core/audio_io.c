#include "bsp/m5stack_core_s3.h"
#include "esp_codec_dev.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "include/aec.h"
#include "include/audio_io.h"
#include "include/udp_audio.h"

#include <stdint.h>
#include <string.h>

static const char            *TAG = "audio_io";
static esp_codec_dev_handle_t s_mic;
static esp_codec_dev_handle_t s_spk;

static void
audio_task (void *arg)
{
  static int16_t    spk_st[FRAME_SAMPLES * 2];
  static int16_t    mic_st[FRAME_SAMPLES * 2];
  static int16_t    ref[FRAME_SAMPLES]; // what we play (mono)
  static int16_t    mic[FRAME_SAMPLES]; // what we captured (mono)
  static int16_t    out[FRAME_SAMPLES]; // AEC output
  udp_audio_frame_t f;
  bool              playing = false;

  while (1)
    {
      // Decide what to play this tick. ALWAYS produce a full frame.
      memset (ref, 0, sizeof (ref));
      if (! playing && udp_audio_queued () >= PREBUFFER)
        playing = true;
      if (playing)
        {
          if (udp_audio_recv (&f, 0) == ESP_OK) // timeout 0: never stall the mic
            memcpy (ref, f.data, MIN (f.len, MONO_BYTES));
          else
            playing = false; // underrun: rebuffer, play silence
        }

      // Play it
      for (int i = 0; i < FRAME_SAMPLES; i++)
        spk_st[2 * i] = spk_st[2 * i + 1] = ref[i];
      esp_codec_dev_write (s_spk, spk_st, sizeof (spk_st));

      // Capture
      if (esp_codec_dev_read (s_mic, mic_st, sizeof (mic_st)) != ESP_CODEC_DEV_OK)
        continue;
      for (int i = 0; i < FRAME_SAMPLES; i++)
        mic[i] = (int16_t) (((int) mic_st[2 * i] + mic_st[2 * i + 1]) / 2);

      // AEC: mic + the exact frame we just played
      aec_process (mic, ref, out);
      udp_audio_send (out, sizeof (out));
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

  aec_init();
  bsp_feature_enable (BSP_FEATURE_SPEAKER, true);
  esp_codec_dev_set_out_vol (s_spk, SPEAKER_VOL);
  esp_codec_dev_set_in_gain (s_mic, MIC_GAIN_DB);

  xTaskCreate (audio_task, "mic", 8192, NULL, 10, NULL);
  return ESP_OK;
}
