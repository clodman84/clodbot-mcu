#include "include/aec.h"
#include "esp_dsp.h"
#include "esp_err.h"
#include "include/udp_audio.h"
#include "math.h"
#include <stdint.h>
#include <string.h>

#define filter_length_M      (FRAME_SAMPLES + 1)
#define input_block_length_L (FRAME_SAMPLES)
#define fft_len_N            (filter_length_M + input_block_length_L - 1)
#define TAG                  "aec"

// this is the filter
__attribute__ ((aligned (16))) static float H_cf[fft_len_N * 2];
// Working array of input_signal
__attribute__ ((aligned (16))) static float X_cf[fft_len_N * 2];
static float                                previous_x[filter_length_M - 1];

#define INV_Q15            (1.0f / 32768.0f)
#define NORMALISATION_TERM (1.0f / 512.f) // I do not know how to float this ideally it should be 1/fft_len_N but my lsp is yelling at me

void
aec_init ()
{
  // Initialise h_cf  let the filter be just 1s for now
  for (int i = 0; i < fft_len_N; i++)
    {
      H_cf[i * 2]     = 1.f;
      H_cf[i * 2 + 1] = 0.f;
    }
  memset(previous_x, 0,(filter_length_M - 1) * sizeof(float));
  esp_err_t ret = dsps_fft2r_init_fc32 (NULL, fft_len_N);
  if (ret != ESP_OK)
    {
      ESP_LOGE (TAG, "Not possible to initialize FFT. Error = %i", ret);
      return;
    }
}

static void
apply_filter (int16_t *ref)
{
  // Write the history into the first M - 1 terms of X_cf
  for (int i = 0; i < filter_length_M - 1; i++)
    {
      X_cf[i * 2]     = previous_x[i];
      X_cf[i * 2 + 1] = 0.f;
    }
  for (int i = 0; i < input_block_length_L; i++)
    {
      X_cf[(filter_length_M - 1 + i) * 2]     = ref[i] * INV_Q15;
      X_cf[(filter_length_M - 1 + i) * 2 + 1] = 0.f;
    }
  // Save the last M-1 terms for the next iteration
  for (int i = 0; i < filter_length_M - 1; i++)
    {
      previous_x[i] = X_cf[(input_block_length_L + i) * 2];
    }

  // FFT the input
  dsps_fft2r_fc32_aes3 (X_cf, fft_len_N);
  dsps_bit_rev_fc32 (X_cf, fft_len_N);

  // Y = X * H, stored conjugated: IFFT(Y) = conj(FFT(conj(Y)))/N, so a
  // forward FFT on conj(Y) gives the time signal (real part) without a
  // dedicated inverse routine.
  for (int i = 0; i < fft_len_N; i++)
    {
      float xr = X_cf[i * 2], xi = X_cf[i * 2 + 1];
      float hr = H_cf[i * 2], hi = H_cf[i * 2 + 1];
      X_cf[i * 2]     = xr * hr - xi * hi;
      X_cf[i * 2 + 1] = -(xr * hi + xi * hr);
    }

  // FFT calculated here
  dsps_fft2r_fc32_aes3 (X_cf, fft_len_N);
  // Bit reverse
  dsps_bit_rev_fc32 (X_cf, fft_len_N);
  // Here you have your original complex signal in time domain.
  // Y_cf[i*2 + 0] == x[i]*h(i)*N;
  // Y_cf[i*2 + 1] == 0;
}

void
aec_process (int16_t *mic,
             int16_t *ref,
             int16_t *out)
{

  static int   iterations = 0;
  iterations ++;
  unsigned int start_b    = dsp_get_cpu_cycle_count ();
  apply_filter (mic);
  for (int i = 0; i < input_block_length_L; i++)
    {
      // this could probably be skipped if udp_audio_send and receive used floats instead
      // especially audio_send
      float v = X_cf[(filter_length_M + i - 1) * 2] * 32768.0f * NORMALISATION_TERM;
      v       = fmaxf (-32768.0f, fminf (32767.0f, v));
      out[i]  = (int16_t) lrintf (v);
    }
  unsigned int end_b = dsp_get_cpu_cycle_count ();

  if (iterations % 200 == 0)
    ESP_LOGI (TAG, "AEC Process for %i n point fft and all that takes %i cycles", fft_len_N, end_b - start_b);
}
