#include "bsp/m5stack_core_s3.h"
#include "esp_codec_dev.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "include/aec.h"
#include "include/audio_io.h"
#include "include/udp_audio.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

static const char            *TAG = "audio_io";
static esp_codec_dev_handle_t s_mic;
static esp_codec_dev_handle_t s_spk;

// file scope so measure_delay and audio_task share them
static int16_t spk_st[FRAME_SAMPLES * 2];
static int16_t mic_st[FRAME_SAMPLES * 2];

#define RING_SZ     4096 // power of two, > ref_delay + FRAME_SAMPLES
#define RING_MASK   (RING_SZ - 1)
#define DELAY_MARGIN 24  // click-onset threshold error + pre-ringing
#define DELAY_FALLBACK 1850

static int16_t  s_ring[RING_SZ]; // every sample sent to the speaker
static int      s_ref_delay = DELAY_FALLBACK - DELAY_MARGIN;

// Click into silence, find where it first shows up in the mic.
// Same write-then-read ordering as audio_task, so frame-level effects are included.
static int
measure_delay (void)
{
  const int N_TICKS = 16;
  int       onset = -1, noise = 0;

  for (int k = 0; k < N_TICKS; k++)
    {
      memset (spk_st, 0, sizeof (spk_st));
      if (k == 4)
        for (int i = 0; i < 16; i++)
          spk_st[2 * i] = spk_st[2 * i + 1] = (i & 1) ? 16000 : -16000;

      esp_codec_dev_write (s_spk, spk_st, sizeof (spk_st));
      if (esp_codec_dev_read (s_mic, mic_st, sizeof (mic_st)) != ESP_CODEC_DEV_OK)
        return -1;

      for (int i = 0; i < FRAME_SAMPLES; i++)
        {
          int m = abs (((int) mic_st[2 * i] + mic_st[2 * i + 1]) / 2);
          if (k < 4)
            noise = m > noise ? m : noise;
          else if (onset < 0 && m > 8 * noise + 200)
            onset = (k - 4) * FRAME_SAMPLES + i;
        }
    }
  return onset;
}

static void
calibrate_delay (void)
{
  for (int i = 0; i < 3; i++)
    measure_delay (); // warm-up: first readings are a startup transient

  // take the median of 5 so one bad run can't set the delay
  int r[5], n = 0;
  for (int i = 0; i < 5; i++)
    {
      int d = measure_delay ();
      if (d > 0)
        r[n++] = d;
    }
  int d = DELAY_FALLBACK;
  if (n >= 3)
    {
      for (int i = 1; i < n; i++) // insertion sort
        for (int j = i; j > 0 && r[j - 1] > r[j]; j--)
          {
            int t = r[j]; r[j] = r[j - 1]; r[j - 1] = t;
          }
      d = r[n / 2];
    }
  else
    ESP_LOGE (TAG, "delay measurement failed (%d good runs), using fallback", n);

  s_ref_delay = d - DELAY_MARGIN;
  if (s_ref_delay < 0)
    s_ref_delay = 0;
  if (s_ref_delay > RING_SZ - FRAME_SAMPLES)
    s_ref_delay = RING_SZ - FRAME_SAMPLES;
  ESP_LOGI (TAG, "measured delay %d, ref_delay %d", d, s_ref_delay);
}

static void
audio_task (void *arg)
{
  static int16_t    ref[FRAME_SAMPLES];  // what we play (mono)
  static int16_t    dref[FRAME_SAMPLES]; // ref delayed to line up with the echo in mic
  static int16_t    mic[FRAME_SAMPLES];  // what we captured (mono)
  static int16_t    out[FRAME_SAMPLES];  // AEC output
  udp_audio_frame_t f;
  bool              playing = false;

  uint32_t spk_w = 0; // samples written to the speaker
  uint32_t mic_w = 0; // samples successfully read from the mic

  while (1)
    {
      memset (ref, 0, sizeof (ref));
      if (! playing && udp_audio_queued () >= PREBUFFER)
        playing = true;
      if (playing)
        {
          if (udp_audio_recv (&f, 0) == ESP_OK)
            memcpy (ref, f.data, MIN (f.len, MONO_BYTES));
          else
            playing = false;
        }

      // Log exactly what goes to the speaker, then play it.
      for (int i = 0; i < FRAME_SAMPLES; i++)
        {
          s_ring[(spk_w + i) & RING_MASK] = ref[i];
          spk_st[2 * i] = spk_st[2 * i + 1] = ref[i];
        }
      spk_w += FRAME_SAMPLES;
      esp_codec_dev_write (s_spk, spk_st, sizeof (spk_st));

      if (esp_codec_dev_read (s_mic, mic_st, sizeof (mic_st)) != ESP_CODEC_DEV_OK)
        {
          ESP_LOGW (TAG, "mic read failed, skipping frame");
          continue; // mic_w not advanced, see note below
        }
      for (int i = 0; i < FRAME_SAMPLES; i++)
        mic[i] = (int16_t) (((int) mic_st[2 * i] + mic_st[2 * i + 1]) / 2);

      // The echo in this mic frame was played s_ref_delay samples before
      // the mic stream position, so index the ring by mic position.
      for (int i = 0; i < FRAME_SAMPLES; i++)
        dref[i] = s_ring[(mic_w + i - s_ref_delay) & RING_MASK];
      mic_w += FRAME_SAMPLES;

      aec_run (mic, dref, out);
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
    .channel         = 2,
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

  calibrate_delay (); // needs a quiet room, with volume and gain at their final settings
  aec_init ();

  if (xTaskCreate (audio_task, "mic", 8192, NULL, 10, NULL) != pdPASS)
    {
      ESP_LOGE (TAG, "audio_task creation failed (stack alloc)");
      return ESP_ERR_NO_MEM;
    }
  return ESP_OK;
}
