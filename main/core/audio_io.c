#include "bsp/m5stack_core_s3.h"
#include "esp_codec_dev.h"
#include "esp_log.h"
#include "esp_timer.h"
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
#include <sys/param.h>

#define AUDIO_TASK_PRIO 10
#define STARVE_FRAMES   4 // consecutive empty polls before we re-buffer

static const char            *TAG = "audio_io";
static esp_codec_dev_handle_t s_mic;
static esp_codec_dev_handle_t s_spk;

// file scope so measure_delay and audio_task share them
static int16_t                spk_st[FRAME_SAMPLES * 2];
static int16_t                mic_st[FRAME_SAMPLES * 2];

#define RING_SZ        4096 // power of two, > ref_delay + FRAME_SAMPLES
#define RING_MASK      (RING_SZ - 1)
#define DELAY_MARGIN   24 // click-onset threshold error + pre-ringing
#define DELAY_FALLBACK 1850

static int16_t s_ring[RING_SZ]; // every sample sent to the speaker
static int     s_ref_delay = DELAY_FALLBACK - DELAY_MARGIN;

// Click into silence, find where it first shows up in the mic.
// Same write-then-read ordering as audio_task, so frame-level effects are included.
static int
measure_delay (void)
{
  // Click at tick 4; must listen long enough to hear an echo ~DELAY_FALLBACK
  // samples later regardless of FRAME_MS (a fixed 16 ticks only covers
  // 1536 samples at 8 ms, which misses the echo entirely).
  const int N_TICKS = 6 + 3000 / FRAME_SAMPLES;
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
            int t    = r[j];
            r[j]     = r[j - 1];
            r[j - 1] = t;
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
  static int16_t  audio_scratch[FRAME_SAMPLES * 4];
  static int16_t *out  = &audio_scratch[FRAME_SAMPLES * 0];
  static int16_t *ref  = &audio_scratch[FRAME_SAMPLES * 1];
  static int16_t *mic  = &audio_scratch[FRAME_SAMPLES * 2];
  static int16_t *dref = &audio_scratch[FRAME_SAMPLES * 3];

  udp_audio_frame_t f;
  bool              playing = false;

  uint32_t          spk_w = 0; // samples written to the speaker
  uint32_t          mic_w = 0; // samples successfully read from the mic

  // diagnostics, logged once a second
  uint32_t miss = 0, underruns = 0, rebuffers = 0, trimmed = 0, frames = 0;
  int64_t  worst_us = 0, worst_loop_us = 0, last_loop = esp_timer_get_time ();

  while (1)
    {
      // ---- playout: jitter buffer ------------------------------------
      memset (ref, 0, MONO_BYTES);
      uint32_t q = udp_audio_queued ();
      if (! playing && q >= PREBUFFER)
        {
          playing = true;
          miss    = 0;
          rebuffers++;
        }
      if (playing)
        {
          // A Wi-Fi burst can pile up many frames; use the newest, don't let
          // latency grow without bound.
          if (q > MAX_FILL)
            trimmed += udp_audio_trim (MAX_FILL);

          if (udp_audio_recv (&f, 0) == ESP_OK)
            {
              memcpy (ref, f.data, MIN (f.len, MONO_BYTES));
              miss = 0;
            }
          else
            {
              // One missing frame is a one-frame hole, not a reason to
              // throw away the buffer. Only re-buffer on sustained starvation.
              underruns++;
              if (++miss >= STARVE_FRAMES)
                playing = false;
            }
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
          continue; // mic_w not advanced: the ring is indexed by mic position
        }
      for (int i = 0; i < FRAME_SAMPLES; i++)
        mic[i] = (int16_t) (((int) mic_st[2 * i] + mic_st[2 * i + 1]) / 2);

      // The echo in this mic frame was played s_ref_delay samples before
      // the mic stream position, so index the ring by mic position.
      for (int i = 0; i < FRAME_SAMPLES; i++)
        dref[i] = s_ring[(mic_w + i - s_ref_delay) & RING_MASK];
      mic_w += FRAME_SAMPLES;

      // ---- processing + telemetry (time it: must fit well inside FRAME_MS)
      int64_t t0 = esp_timer_get_time ();
      aec_run (mic, dref, out);
      udp_audio_send (audio_scratch, MONO_BYTES * 4);
      int64_t now = esp_timer_get_time ();

      if (now - t0 > worst_us)
        worst_us = now - t0;
      if (now - last_loop > worst_loop_us)
        worst_loop_us = now - last_loop;
      last_loop = now;

      if (++frames >= 1000 / FRAME_MS)
        {
          udp_audio_stats_t st;
          udp_audio_get_stats (&st);
          ESP_LOGI (TAG,
                    "q=%u under=%u rebuf=%u trim=%u | proc worst %lldus loop worst %lldus (budget %dus) | "
                    "rx lost=%u late=%u over=%u resync=%u tx drop=%u",
                    (unsigned) udp_audio_queued (), (unsigned) underruns, (unsigned) rebuffers, (unsigned) trimmed,
                    (long long) worst_us, (long long) worst_loop_us, FRAME_MS * 1000,
                    (unsigned) st.rx_lost, (unsigned) st.rx_late, (unsigned) st.rx_overrun, (unsigned) st.rx_resync,
                    (unsigned) st.tx_dropped);
          frames = 0;
          worst_us = worst_loop_us = 0;
        }
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

  // Frames received during calibration are stale; start from an empty buffer.
  udp_audio_trim (0);

  if (xTaskCreate (audio_task, "mic", 8192, NULL, AUDIO_TASK_PRIO, NULL) != pdPASS)
    {
      ESP_LOGE (TAG, "audio_task creation failed (stack alloc)");
      return ESP_ERR_NO_MEM;
    }
  return ESP_OK;
}
