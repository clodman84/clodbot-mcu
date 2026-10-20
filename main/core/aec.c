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
// Working array of output_signal
__attribute__ ((aligned (16))) static float Y_cf[fft_len_N * 2];
// Working array of error_signal for the weight adaptation
__attribute__ ((aligned (16))) static float E_cf[fft_len_N * 2];
static float                                signal_power_estimate[fft_len_N];
static float                                x_hist[filter_length_M - 1];

#define INV_Q15                          (1.0f / 32768.0f)
#define NORMALISATION_TERM               (1.0f / fft_len_N)
#define STEP_SIZE_MU                     0.02f
#define STEP_NORMALISATION_CONSTANT_BETA 0.9f
#define EPSILON                          1e-6f // to prevent division by 0

void
aec_init ()
{
  for (int i = 0; i < fft_len_N; i++)
    {
      H_cf[i * 2]     = 0.f;
      H_cf[i * 2 + 1] = 0.f;
    }
  memset (signal_power_estimate, 0, fft_len_N * sizeof (float));
  esp_err_t ret = dsps_fft2r_init_fc32 (NULL, fft_len_N);
  if (ret != ESP_OK)
    {
      ESP_LOGE (TAG, "Not possible to initialize FFT. Error = %i", ret);
      return;
    }
}

static bool
double_talk (int16_t *ref,
             int16_t *mic)
{
  return false;
}

static void
calculate_power_estimate ()
{
  for (int i = 0; i < fft_len_N; i++)
    {
      float signal_power       = X_cf[i * 2] * X_cf[i * 2] + X_cf[i * 2 + 1] * X_cf[i * 2 + 1];
      float power_estimate     = STEP_NORMALISATION_CONSTANT_BETA * signal_power_estimate[i] + (1 - STEP_NORMALISATION_CONSTANT_BETA) * signal_power;
      signal_power_estimate[i] = power_estimate;
    }
}

static void
update_filter (int16_t *mic)
{
  // Preparing the Error signal
  for (int i = 0; i < input_block_length_L; i++)
    {
      E_cf[(filter_length_M + i - 1) * 2]     = mic[i] * INV_Q15 - Y_cf[(filter_length_M + i - 1) * 2] * NORMALISATION_TERM;
      E_cf[(filter_length_M + i - 1) * 2 + 1] = 0;
    }
  memset (E_cf, 0, (filter_length_M - 1) * 2 * sizeof (float));

  // FFT the error signal
  dsps_fft2r_fc32_aes3 (E_cf, fft_len_N);
  dsps_bit_rev_fc32 (E_cf, fft_len_N);

  // multiply with the conjugate of X_cf and store the result within E_cf
  for (int i = 0; i < fft_len_N; i++)
    {
      float xr = X_cf[i * 2], xi = X_cf[i * 2 + 1];
      float er = E_cf[i * 2], ei = E_cf[i * 2 + 1];
      E_cf[i * 2]     = (xr * er + xi * ei);
      E_cf[i * 2 + 1] = (xr * ei - xi * er);
    }

  calculate_power_estimate ();

  // add the error to the weights, we are doing unconstrained FBLMS
  for (int i = 0; i < fft_len_N; i++)
    {
      H_cf[i * 2] += 2 * (STEP_SIZE_MU / (signal_power_estimate[i] + EPSILON)) * E_cf[i * 2];
      H_cf[i * 2 + 1] += 2 * (STEP_SIZE_MU / (signal_power_estimate[i] + EPSILON)) * E_cf[i * 2 + 1];
    }
}

static void
apply_filter (int16_t *ref)
{
  for (int i = 0; i < filter_length_M - 1; i++)
    {
      X_cf[i * 2]     = x_hist[i];
      X_cf[i * 2 + 1] = 0.f;
    }
  for (int i = 0; i < input_block_length_L; i++)
    {
      X_cf[(filter_length_M - 1 + i) * 2]     = ref[i] * INV_Q15;
      X_cf[(filter_length_M - 1 + i) * 2 + 1] = 0.f;
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
      Y_cf[i * 2]     = xr * hr - xi * hi;
      Y_cf[i * 2 + 1] = -(xr * hi + xi * hr);
    }

  // FFT calculated here
  dsps_fft2r_fc32_aes3 (Y_cf, fft_len_N);
  // Bit reverse
  dsps_bit_rev_fc32 (Y_cf, fft_len_N);
  // Here you have your signal in time domain.
  // Y_cf[i*2 + 0] == x[i]*h(i)*N;
  // Y_cf[i*2 + 1] == 0;
}

static void
prepare_X_cf_for_next_convolution (int16_t *ref)
{
  const int H = filter_length_M - 1;
  if (H >= input_block_length_L)
    {
      // slide history left by L, append the new block
      memmove (x_hist, x_hist + input_block_length_L,
               (H - input_block_length_L) * sizeof (float));
      for (int i = 0; i < input_block_length_L; i++)
        x_hist[H - input_block_length_L + i] = ref[i] * INV_Q15;
    }
  else
    {
      // history is the tail of the block
      for (int i = 0; i < H; i++)
        x_hist[i] = ref[input_block_length_L - H + i] * INV_Q15;
    }
}

void
aec_process (int16_t *mic,
             int16_t *ref,
             int16_t *out)
{

  static int iterations = 0;
  iterations++;
  unsigned int start_b = dsp_get_cpu_cycle_count ();

  apply_filter (ref);
  if (! double_talk (ref, mic))
    update_filter (mic);
  prepare_X_cf_for_next_convolution (ref);

  for (int i = 0; i < input_block_length_L; i++)
    {
      // this could probably be skipped if udp_audio_send and receive used floats instead
      // especially audio_send
      float v = mic[i] * INV_Q15 - Y_cf[(filter_length_M + i - 1) * 2] * NORMALISATION_TERM;
      v       = v * 32768.0f;
      v       = fmaxf (-32768.0f, fminf (32767.0f, v));
      // The output is the mic signal - filter of the reference signal
      out[i]  = (int16_t) lrintf (v);
    }
  unsigned int end_b = dsp_get_cpu_cycle_count ();

  if (iterations % 200 == 0)
    ESP_LOGI (TAG, "AEC Process for %i takes %i cycles", fft_len_N, end_b - start_b);
}
