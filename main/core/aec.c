#include "include/aec.h"
#include "esp_dsp.h"
#include "esp_err.h"
#include "include/udp_audio.h"
#include "math.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define filter_length_M                  (FRAME_SAMPLES + 1)
#define input_block_length_L             (FRAME_SAMPLES)
#define fft_len_N                        (filter_length_M + input_block_length_L - 1)
#define TAG                              "aec"
#define INV_Q15                          (1.0f / 32768.0f)
#define NORMALISATION_TERM               (1.0f / fft_len_N)
#define STEP_SIZE_MU                     0.02f
#define STEP_NORMALISATION_CONSTANT_BETA 0.9f
#define EPSILON                          1e-2f               // to prevent division by 0
#define HD_NB                            (fft_len_N / 2 + 1) // unique bins of the real N-point FFT
#define HD_H                             10                  // harmonics per fundamental
#define HD_K                             1                   // search window is 2K+1 bins
#define HD_WIN                           (2 * HD_K + 1)
#define HD_MU                            0.05f // NLMS step, (0, 2)
#define HD_BETA                          1.0f  // over-subtraction
#define HD_X_REL                         1.0f  // fundamental is active if |X| > HD_X_REL * mean|X|
#define HD_MAG_FLOOR                     0.02f // ... and above this absolute |X|
#define HD_RHO                           0.1f  // smoothing of mean|X|
#define HD_NF_SMOOTH                     0.1f  // smoothing of |E| before the min tracker
#define HD_NF_RISE                       1.01f // per-frame rise of the noise-floor tracker (bins with no active speaker band)
#define HD_NF_BIAS                       1.5f  // min of a smoothed magnitude underestimates the floor
#define HD_G_MIN                         0.1f  // hard gain floor, -20 dB
#define HD_ADAPT_SNR                     1.5f  // adapt bin only if |E| > this * noise floor
#define HD_EPS                           1e-6f
#define HD_T1                            8     // rollback: commit lags the weights by T1..2*T1 steps
#define HD_T2                            10    // hysteresis: single-talk frames before adapting
#define DTD_X_MIN                        1e-6f // far end active if mean(ref^2) > this (~ -60 dBFS)
#define DTD_RATIO                        0.8f  // dt if P_err / P_mic > this
#define DTD_INIT                         100   // far-end-active frames of forced adaptation at startup
#define DTD_HANG                         6     // frames dt persists after the statistic drops
#define DTD_MAX_HOLD                     150   // watchdog: dt held this long -> force adaptation
#define DTD_RELEASE                      20    // frames of forced adaptation after the watchdog
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
static float                               *hd_W, *hd_Ws, *hd_Wc; // working / snapshot / committed weights
static int                                  hd_nw;
static float                                hd_win[fft_len_N];
static float                                hd_hist[input_block_length_L]; // previous AEC error block
static float                                hd_ola[input_block_length_L];  // tail of previous synthesised frame
static float                                hd_Es[HD_NB], hd_Nf[HD_NB];
static float                                hd_xavg;
static int                                  hd_st_run, hd_since_snap;
static bool                                 hd_started, hd_dirty;
__attribute__ ((aligned (16))) static float R_cf[fft_len_N * 2];

// largest fundamental bin that can reach a bin <= HD_NB-1 as harmonic j
static inline int
hd_imax (int j)
{
  int im = (HD_NB - 1 + HD_K) / j;
  return im < HD_NB - 1 ? im : HD_NB - 1;
}

static void
hdres_init ()
{
  hd_nw = 0;
  for (int j = 1; j <= HD_H; j++)
    hd_nw += hd_imax (j) * HD_WIN;
  free (hd_W);
  hd_W = calloc (3 * hd_nw, sizeof (float));
  if (! hd_W)
    {
      ESP_LOGE (TAG, "HDRES: no memory for %i weights, passing through", 3 * hd_nw);
      return;
    }
  hd_Ws = hd_W + hd_nw;
  hd_Wc = hd_W + 2 * hd_nw;
  for (int n = 0; n < fft_len_N; n++)
    hd_win[n] = sinf (3.14159265f * (n + 0.5f) / fft_len_N); // w^2[n] + w^2[n+L] = 1
  memset (hd_hist, 0, sizeof (hd_hist));
  memset (hd_ola, 0, sizeof (hd_ola));
  hd_started = hd_dirty = false;
  hd_st_run = hd_since_snap = 0;
}

// Adaptation control: hysteresis + rollback. Returns true if weights may adapt.
static bool
hd_adapt_enabled (bool dt)
{
  if (dt)
    {
      if (hd_dirty) // discard the steps taken just before the DTD fired
        {
          memcpy (hd_W, hd_Wc, hd_nw * sizeof (float));
          memcpy (hd_Ws, hd_Wc, hd_nw * sizeof (float));
          hd_dirty = false;
        }
      hd_st_run = hd_since_snap = 0;
      return false;
    }
  if (hd_st_run < HD_T2)
    {
      hd_st_run++;
      return false;
    }
  hd_dirty = true;
  if (++hd_since_snap >= HD_T1)
    {
      memcpy (hd_Wc, hd_Ws, hd_nw * sizeof (float)); // commit the snapshot taken T1 steps ago
      memcpy (hd_Ws, hd_W, hd_nw * sizeof (float));
      hd_since_snap = 0;
    }
  return true;
}

// Dr[m] = sum W*|X(i)|, Pn[m] = sum |X(i)|^2 over the same terms (NLMS norm per target bin)
static void
hd_predict (const float *Xp,
            float        thr_x,
            float       *Dr,
            float       *Pn)
{
  memset (Dr, 0, HD_NB * sizeof (float));
  memset (Pn, 0, HD_NB * sizeof (float));
  const float *w = hd_W;
  for (int j = 1; j <= HD_H; j++)
    for (int i = 1, im = hd_imax (j); i <= im; i++, w += HD_WIN)
      {
        float xi = Xp[i];
        if (xi < thr_x)
          continue;
        for (int k = -HD_K; k <= HD_K; k++)
          {
            int m = i * j + k;
            if (m < 1 || m >= HD_NB)
              continue;
            Dr[m] += w[k + HD_K] * xi;
            Pn[m] += xi * xi;
          }
      }
}

// W(i,j,k) += mu * |X(i)| * xi(m) / Pn(m), stp[m] holds mu*xi/Pn (0 = no adaptation)
static void
hd_update (const float *Xp,
           float        thr_x,
           const float *stp)
{
  float *w = hd_W;
  for (int j = 1; j <= HD_H; j++)
    for (int i = 1, im = hd_imax (j); i <= im; i++, w += HD_WIN)
      {
        float xi = Xp[i];
        if (xi < thr_x)
          continue;
        for (int k = -HD_K; k <= HD_K; k++)
          {
            int m = i * j + k;
            if (m < 1 || m >= HD_NB || stp[m] == 0.f)
              continue;
            float v     = w[k + HD_K] + xi * stp[m];
            w[k + HD_K] = v > 0.f ? v : 0.f; // magnitudes add, weights stay >= 0
          }
      }
}

// e: AEC error block (float, +-1). out: suppressed block, delayed by one block.
// Must be called after apply_filter(): it reads |X_cf| of the current frame.
static void
hdres_run (const float *e,
           bool         dt,
           float       *out)
{
  static float Emag[HD_NB], Xp[HD_NB], Dr[HD_NB], Pn[HD_NB], G[HD_NB], stp[HD_NB];
  const int    L = input_block_length_L, N = fft_len_N;

  if (! hd_W)
    {
      memcpy (out, e, L * sizeof (float));
      return;
    }

  // analysis: [previous block, current block] * sine window
  for (int n = 0; n < L; n++)
    {
      R_cf[n * 2]           = hd_win[n] * hd_hist[n];
      R_cf[n * 2 + 1]       = 0.f;
      R_cf[(L + n) * 2]     = hd_win[L + n] * e[n];
      R_cf[(L + n) * 2 + 1] = 0.f;
      hd_hist[n]            = e[n];
    }
  dsps_fft2r_fc32_aes3 (R_cf, N);
  dsps_bit_rev_fc32 (R_cf, N);

  float xsum = 0.f;
  for (int m = 0; m < HD_NB; m++)
    {
      Emag[m] = sqrtf (R_cf[m * 2] * R_cf[m * 2] + R_cf[m * 2 + 1] * R_cf[m * 2 + 1]);
      Xp[m]   = sqrtf (X_cf[m * 2] * X_cf[m * 2] + X_cf[m * 2 + 1] * X_cf[m * 2 + 1]);
      if (m > 0)
        xsum += Xp[m];
    }
  float xmean = xsum / (HD_NB - 1);

  if (! hd_started)
    {
      hd_xavg = xmean;
      for (int m = 0; m < HD_NB; m++)
        {
          hd_Es[m] = Emag[m];
          hd_Nf[m] = HD_MAG_FLOOR; // start low: only echo-free frames can raise it
        }
      hd_started = true;
    }
  else
    hd_xavg += HD_RHO * (xmean - hd_xavg);

  float thr_x = fmaxf (HD_X_REL * hd_xavg, HD_MAG_FLOOR);
  bool  adapt = hd_adapt_enabled (dt);

  hd_predict (Xp, thr_x, Dr, Pn);

  // Noise floor by minimum statistics. It may only rise in bins that no active
  // speaker band feeds; otherwise a sustained tone drags the floor up to the echo.
  for (int m = 1; m < HD_NB; m++)
    {
      hd_Es[m] += HD_NF_SMOOTH * (Emag[m] - hd_Es[m]);
      hd_Nf[m] = fminf (hd_Es[m], hd_Nf[m] * (Pn[m] > 0.f ? 1.f : HD_NF_RISE));
    }

  G[0] = 1.f;
  for (int m = 1; m < HD_NB; m++)
    {
      float nf = HD_NF_BIAS * hd_Nf[m];
      float g  = 1.f;
      stp[m]   = 0.f;
      if (Dr[m] > 0.f)
        {
          g = fmaxf (Emag[m] - HD_BETA * Dr[m], nf) / (Emag[m] + HD_EPS);
          g = fminf (fmaxf (g, HD_G_MIN), 1.f);
        }
      if (adapt && Pn[m] > 0.f && Emag[m] > HD_ADAPT_SNR * nf)
        stp[m] = HD_MU * (Emag[m] - Dr[m]) / (Pn[m] + HD_EPS);
      G[m] = g;
    }
  if (adapt)
    hd_update (Xp, thr_x, stp);

  // apply the real gain to both halves of the spectrum
  for (int m = 1; m < HD_NB; m++)
    {
      R_cf[m * 2] *= G[m];
      R_cf[m * 2 + 1] *= G[m];
      if (N - m != m)
        {
          R_cf[(N - m) * 2] *= G[m];
          R_cf[(N - m) * 2 + 1] *= G[m];
        }
    }

  // inverse FFT via conj(FFT(conj(Z)))/N, same trick as in apply_filter
  for (int n = 0; n < N; n++)
    R_cf[n * 2 + 1] = -R_cf[n * 2 + 1];
  dsps_fft2r_fc32_aes3 (R_cf, N);
  dsps_bit_rev_fc32 (R_cf, N);

  // synthesis window + overlap-add
  const float s = 1.f / N;
  for (int n = 0; n < L; n++)
    {
      out[n]    = hd_ola[n] + R_cf[n * 2] * s * hd_win[n];
      hd_ola[n] = R_cf[(L + n) * 2] * s * hd_win[L + n];
    }
}

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
  hdres_init ();
}

static bool
double_talk (const float   *err,
             const int16_t *ref,
             const int16_t *mic,
             bool          *far_active)
{
  static int init, hang, held, release;

  float      x = 0.f, d = 0.f, e = 0.f;
  for (int i = 0; i < input_block_length_L; i++)
    {
      float r = ref[i] * INV_Q15, m = mic[i] * INV_Q15;
      x += r * r;
      d += m * m;
      e += err[i] * err[i];
    }
  const float inv = 1.f / input_block_length_L;
  *far_active     = x * inv > DTD_X_MIN;
  if (! *far_active)
    {
      held = 0;
      return false;
    }
  if (init < DTD_INIT)
    {
      init++;
      return false;
    }
  if (release > 0)
    {
      release--;
      return false;
    }

  if (e > DTD_RATIO * d)
    hang = DTD_HANG;
  else if (hang > 0)
    hang--;

  bool dt = hang > 0;
  held    = dt ? held + 1 : 0;
  if (held > DTD_MAX_HOLD)
    {
      release = DTD_RELEASE;
      held = hang = 0;
    }
  return dt;
}

static void
calculate_power_estimate ()
{
  for (int i = 0; i < fft_len_N; i++)
    {
      float signal_power       = X_cf[i * 2] * X_cf[i * 2] + X_cf[i * 2 + 1] * X_cf[i * 2 + 1];
      float power_estimate     = STEP_NORMALISATION_CONSTANT_BETA * signal_power_estimate[i] + (1 - STEP_NORMALISATION_CONSTANT_BETA) * signal_power;
      signal_power_estimate[i] = fmaxf (power_estimate, signal_power); // never under-estimate on onsets
    }
}

static void
update_filter (int16_t *mic)
{
  // FBLMS
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
  // to save on the number of FFTs we have to compute
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
  // forward FFT on conj(Y) gives the time signal (real part)
  // This is because ESP-DSP has no dedicated inverse routine
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
  // overlap save shenanigans
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

static inline int16_t
to_q15 (float v)
{
  v = v * 32768.0f;
  v = fmaxf (-32768.0f, fminf (32767.0f, v));
  return (int16_t) lrintf (v);
}

void
aec_run (int16_t *mic,
         int16_t *ref,
         int16_t *out,
         int16_t *nohdres)
{
  static float err[input_block_length_L], res[input_block_length_L];
  static int   iterations = 0;
  static int   dt_cnt     = 0;
  iterations++;

  unsigned int start_b = dsp_get_cpu_cycle_count ();
  apply_filter (ref);
  for (int i = 0; i < input_block_length_L; i++)
    {
      err[i] = mic[i] * INV_Q15 - Y_cf[(filter_length_M + i - 1) * 2] * NORMALISATION_TERM;
      nohdres[i] = to_q15(err[i]);
    }
  bool far;
  bool dt = double_talk (err, ref, mic, &far);
  dt_cnt += dt;
  if (far && ! dt)
    update_filter (mic);
  prepare_X_cf_for_next_convolution (ref);

  unsigned int mid_b = dsp_get_cpu_cycle_count ();
  hdres_run (err, dt, res);
  unsigned int end_b = dsp_get_cpu_cycle_count ();

  for (int i = 0; i < input_block_length_L; i++)
    out[i] = to_q15 (res[i]);

  static float sum_mic = 0, sum_aec = 0, sum_out = 0;
  for (int i = 0; i < input_block_length_L; i++)
    {
      float a = err[i] * 32768.0f;
      sum_mic += (float) mic[i] * mic[i];
      sum_aec += a * a;
      sum_out += (float) out[i] * out[i];
    }

  if (iterations % 200 == 0)
    {
      ESP_LOGI (TAG, "dt %i/200", dt_cnt);
      dt_cnt = 0; // inside the %200 block
      ESP_LOGI (TAG, "AEC %i cycles, HDRES %i cycles", mid_b - start_b, end_b - mid_b);
      ESP_LOGI (TAG, "ERLE AEC %.1f dB, AEC+HDRES %.1f dB",
                10.f * log10f ((sum_mic + 1.f) / (sum_aec + 1.f)),
                10.f * log10f ((sum_mic + 1.f) / (sum_out + 1.f)));
      sum_mic = sum_aec = sum_out = 0;
    }
}
