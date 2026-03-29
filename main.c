/**
 * ============================================================================
 *                         FM RADIO RECEIVER
 *              A Literate Programming Tutorial in C
 * ============================================================================
 *
 * This program turns a HackRF One (a USB software-defined radio device) into
 * an FM radio. It captures raw electromagnetic waves and converts them into
 * audible sound using digital signal processing (DSP).
 *
 *
 * WHAT IS SOFTWARE-DEFINED RADIO (SDR)?
 * --------------------------------------
 * A traditional radio uses analog circuits (coils, capacitors, crystals) to
 * select and decode stations. An SDR replaces almost all of that hardware with
 * software. The HackRF simply digitizes a wide chunk of the radio spectrum,
 * and our code does the rest with math.
 *
 *
 * THE SIGNAL PROCESSING PIPELINE
 * --------------------------------
 * Raw radio waves go through six stages to become audio:
 *
 *   HackRF Antenna
 *        |
 *        v
 *   [1. CAPTURE]  The HackRF digitizes radio waves at 1.92 million
 *        |        samples per second, producing I/Q pairs (explained below).
 *        v
 *   [2. MIX]      We shift our target station's frequency down to zero
 *        |        ("baseband") so subsequent stages don't need to know
 *        |        which frequency we tuned to.
 *        v
 *   [3. DECIMATE]  We reduce the sample rate from 1.92 MHz to 480 kHz
 *        |         by averaging groups of 4 samples. This discards
 *        |         frequencies we don't need and reduces CPU work.
 *        v
 *   [4. DEMODULATE]  FM encodes audio in the *rate of phase change*
 *        |           of the carrier wave. We extract the audio by
 *        |           measuring how fast the phase is spinning.
 *        v
 *   [5. CLEAN]  Three sub-steps:
 *        |      a) Low-pass filter: remove frequencies above human hearing
 *        |      b) De-emphasis: undo a broadcast treble boost (explained later)
 *        |      c) AGC: automatically adjust volume for comfort
 *        v
 *   [6. PLAY]  Send 48,000 audio samples/second to macOS speakers.
 *
 *
 * WHAT ARE I/Q SAMPLES?
 * ----------------------
 * When the HackRF digitizes radio waves, it doesn't just measure the wave's
 * height at each instant (like a microphone does with sound). Instead, it
 * captures TWO numbers per sample:
 *
 *   I (In-phase):     "How much of the wave is aligned with a cosine?"
 *   Q (Quadrature):   "How much of the wave is aligned with a sine?"
 *
 * Together, I and Q describe both the amplitude AND the phase of the wave
 * at that instant. You can think of (I, Q) as a point on a 2D plane that
 * spins around the origin as the wave oscillates. This representation lets
 * us do things that would be impossible with a single number -- like
 * distinguishing between frequencies above and below our tuning frequency.
 *
 * Mathematically, we treat each (I, Q) pair as a complex number: I + jQ,
 * where j = sqrt(-1). Complex arithmetic makes frequency-shifting trivial
 * (it's just multiplication), which is why SDR uses this representation.
 *
 *
 * THREADING MODEL
 * ----------------
 * This program has two concurrent threads that share data through a ring
 * buffer (a circular queue):
 *
 *   [HackRF driver thread]  ---> ring buffer ---> [macOS audio thread]
 *     (produces samples)                           (consumes samples)
 *
 * The HackRF pushes raw I/Q data at its own pace. Our callback processes
 * each batch through the DSP pipeline and writes finished audio samples
 * into the ring buffer. Meanwhile, macOS pulls audio samples out of the
 * ring buffer whenever the speakers need more data. The ring buffer
 * decouples these two timing domains so neither thread blocks the other.
 *
 *
 * HOW THIS FILE IS ORGANIZED
 * ----------------------------
 * This file is meant to be read top-to-bottom. Each section introduces one
 * stage of the pipeline completely -- its purpose, its data structure, its
 * constants, and its implementation -- before moving to the next stage. After
 * all stages are defined, we tie them together in the callbacks (where the
 * full pipeline runs) and the main function.
 */

#include <AudioToolbox/AudioQueue.h> /* macOS Audio System */
#include <libhackrf/hackrf.h>        /* HackRF Device Drivers */
#include <math.h>                    /* Trig and Math (sin, cos, atan2) */
#include <signal.h>                  /* Handling Ctrl+C */
#include <stdatomic.h>               /* Thread-safe flags and pointers */
#include <stdbool.h>                 /* True/False logic */
#include <stdint.h>                  /* Standard integer types (int16_t, etc) */
#include <stdio.h>                   /* Console output */
#include <stdlib.h>                  /* Memory management */
#include <string.h>                  /* Buffer operations */
#include <unistd.h>                  /* System sleep and timing */

/* ============================================================================
 * GLOBAL CONSTANTS
 * ============================================================================
 * These sample rates and decimation factors define the overall data flow
 * through the pipeline. They are grouped here because multiple stages
 * reference them. Stage-specific constants are defined alongside their
 * respective stages further below.
 */

/* The HackRF captures 1,920,000 I/Q sample pairs every second.
 * This wide bandwidth lets us "see" roughly +/- 960 kHz around our
 * center frequency -- enough to contain one FM station (which uses
 * about 200 kHz of bandwidth).
 */
#define HACKRF_SAMPLE_RATE 1920000

/* Your speakers expect 48,000 audio samples every second (48 kHz).
 * This is a standard rate that covers the full range of human hearing
 * (we can hear up to ~20 kHz, and you need at least 2x that rate by
 * the Nyquist theorem).
 */
#define AUDIO_SAMPLE_RATE 48000

/* We reduce the sample rate in two stages:
 *   1.92 MHz / 4  = 480 kHz  (after IQ decimation -- still wide enough for FM)
 *   480 kHz  / 10 = 48 kHz   (after audio decimation -- matches speaker rate)
 *
 * Why two stages instead of one big /40? Each stage can use a simpler,
 * cheaper filter, and the intermediate rate (480 kHz) is useful for FM
 * demodulation which needs some bandwidth headroom.
 */
#define IQ_DECIMATION_FACTOR 4
#define AUDIO_DECIMATION_FACTOR 10

/* Default FM station: 100.0 MHz. Override with the -f flag. */
#define DEFAULT_FREQUENCY_HZ 100000000

/* ============================================================================
 * STAGE 1: CAPTURE
 * ============================================================================
 * The HackRF One is a USB device that digitizes a slice of the radio spectrum.
 * It produces pairs of signed 8-bit integers (I, Q) at HACKRF_SAMPLE_RATE.
 * Before we can do any math, we normalize these 8-bit values (-128..+127)
 * to floating point (-1.0..+1.0) by dividing by 128.
 *
 * We also configure the HackRF's analog gain stages here. The signal path
 * inside the HackRF has two amplifiers:
 *   LNA (Low Noise Amplifier): first stage, near the antenna.
 *   VGA (Variable Gain Amplifier): second stage, after initial filtering.
 * Higher values = more sensitivity but also more noise. These are reasonable
 * defaults for typical indoor FM reception.
 */

#define HACKRF_RESAMPLING_DIVISOR 128.0f
#define LNA_GAIN 32
#define VGA_GAIN 30

/* ============================================================================
 * STAGE 2: FREQUENCY MIXING (THE TUNING DIAL)
 * ============================================================================
 * The HackRF gives us a wide chunk of spectrum centered on some frequency.
 * Our target FM station sits at a specific position within that chunk. To
 * process it, we need to shift it down to "baseband" (centered at 0 Hz).
 *
 * We do this by multiplying the signal by a spinning complex exponential:
 *
 *   shifted = signal * e^(j*w*t) = signal * (cos(wt) + j*sin(wt))
 *
 * This is called "mixing" or "heterodyning". It shifts all frequencies in
 * the signal by w Hz. Multiplying complex numbers is what makes I/Q
 * representation so powerful -- frequency shifting is just multiplication.
 *
 * DC OFFSET TRICK
 * ----------------
 * Every radio receiver generates a small amount of noise right at its center
 * frequency (called a "DC offset" or "DC spike"). If we tuned the HackRF
 * directly to our station, this noise would land right on top of our signal.
 * Instead, we tune the hardware IF_OFFSET Hz *below* our target, then shift
 * the signal up by IF_OFFSET in software. The DC spike ends up IF_OFFSET Hz
 * away from our station, where it can't interfere.
 */

#define IF_OFFSET 200000

/**
 * NCO -- Numerically Controlled Oscillator.
 *
 * The NCO generates the spinning complex exponential sample-by-sample.
 * Rather than calling sin() and cos() for every sample (expensive), it
 * uses a 2x2 rotation matrix to advance the phasor by one step:
 *
 *   [cos(n+1)]   [step_cos  -step_sin] [cos(n)]
 *   [sin(n+1)] = [step_sin   step_cos] [sin(n)]
 *
 * This costs only two multiplies and two additions per sample. Over
 * millions of iterations, floating-point rounding would make the phasor
 * grow or shrink, so we periodically re-normalize it to unit length.
 */
typedef struct {
  float cos;      /* Current cosine value of the phasor */
  float sin;      /* Current sine value of the phasor */
  float step_cos; /* Cosine of the per-sample phase increment */
  float step_sin; /* Sine of the per-sample phase increment */
  int norm_count; /* Samples since last re-normalization */
} NcoState;

/**
 * Precompute the rotation step from the desired frequency offset.
 */
void dsp_nco_init(NcoState *nco, float freq_offset, float sample_rate) {
  nco->cos = 1.0f;
  nco->sin = 0.0f;
  float w = 2.0f * (float)M_PI * freq_offset / sample_rate;
  nco->step_cos = cosf(w);
  nco->step_sin = sinf(w);
  nco->norm_count = 0;
}

/**
 * Mix one I/Q sample: multiply by the NCO phasor, then advance the NCO.
 *
 * The math:  (I + jQ) * (cos + jsin)
 *          = (I*cos - Q*sin) + j(I*sin + Q*cos)
 */
void dsp_mix_down(NcoState *nco, float raw_i, float raw_q, float *i, float *q) {
  *i = raw_i * nco->cos - raw_q * nco->sin;
  *q = raw_i * nco->sin + raw_q * nco->cos;

  /* Advance the NCO using the rotation matrix identity:
   *   cos(a+b) = cos(a)*cos(b) - sin(a)*sin(b)
   *   sin(a+b) = sin(a)*cos(b) + cos(a)*sin(b)
   */
  float next_cos = nco->cos * nco->step_cos - nco->sin * nco->step_sin;
  float next_sin = nco->sin * nco->step_cos + nco->cos * nco->step_sin;
  nco->cos = next_cos;
  nco->sin = next_sin;

  /* Re-normalize every 1000 samples to correct accumulated rounding error.
   * Without this, |cos^2 + sin^2| would slowly drift away from 1.0,
   * causing the signal amplitude to grow or decay over time.
   */
  if (++nco->norm_count > 1000) {
    float mag = sqrtf(nco->cos * nco->cos + nco->sin * nco->sin);
    nco->cos /= mag;
    nco->sin /= mag;
    nco->norm_count = 0;
  }
}

/* ============================================================================
 * STAGE 3: IQ DECIMATION (1.92 MHz -> 480 kHz)
 * ============================================================================
 * After mixing, our signal is at baseband but still sampled at 1.92 MHz.
 * That's far more data than we need -- an FM station only occupies about
 * 200 kHz of bandwidth. Decimation reduces the sample rate by averaging
 * groups of samples, discarding the excess detail.
 *
 * Our decimator is the simplest kind: a "box-car" average. Accumulate
 * IQ_DECIMATION_FACTOR (4) input samples, divide by 4, emit one output.
 * The output rate is 1.92 MHz / 4 = 480 kHz.
 *
 * We decimate the I and Q channels independently but in lockstep --
 * they must stay paired for FM demodulation to work correctly.
 */

typedef struct {
  float i_acc; /* Running sum of I samples */
  float q_acc; /* Running sum of Q samples */
  int count;   /* How many samples accumulated so far */
} IqDecimState;

/**
 * Feed one I/Q sample into the decimator.
 * Returns true (and writes *i_out, *q_out) when a decimated output is ready.
 * Returns false when still accumulating.
 */
bool dsp_decimate_iq(IqDecimState *iq, float i_in, float q_in, float *i_out,
                     float *q_out) {
  iq->i_acc += i_in;
  iq->q_acc += q_in;
  if (++iq->count >= IQ_DECIMATION_FACTOR) {
    *i_out = iq->i_acc / (float)IQ_DECIMATION_FACTOR;
    *q_out = iq->q_acc / (float)IQ_DECIMATION_FACTOR;
    iq->i_acc = 0;
    iq->q_acc = 0;
    iq->count = 0;
    return true;
  }
  return false;
}

/* ============================================================================
 * STAGE 4: FM DEMODULATION
 * ============================================================================
 * This is the heart of the FM receiver. FM (Frequency Modulation) encodes
 * audio in the *instantaneous frequency* of the carrier wave. When the DJ's
 * voice goes up, the carrier frequency increases slightly; when it goes down,
 * the frequency decreases. Our job is to measure that frequency variation
 * and output it as an audio waveform.
 *
 * Since frequency is the rate of change of phase, we can recover the audio
 * by measuring how the phase changes between consecutive I/Q samples.
 *
 * THE CONJUGATE PRODUCT TRICK
 * ----------------------------
 * The naive approach would be:
 *   1. Compute phase of current sample:   angle1 = atan2(Q, I)
 *   2. Compute phase of previous sample:  angle0 = atan2(prev_Q, prev_I)
 *   3. Phase difference = angle1 - angle0
 *
 * But there's a cheaper way. Multiplying the current sample by the
 * *conjugate* of the previous sample gives us a complex number whose
 * angle IS the phase difference:
 *
 *   diff = current * conjugate(previous)
 *        = (I + jQ) * (prev_I - j*prev_Q)
 *        = (I*prev_I + Q*prev_Q) + j(Q*prev_I - I*prev_Q)
 *
 * Then a single atan2(imag, real) gives the phase difference directly.
 * This saves one atan2 call and avoids phase-wrapping headaches.
 */

typedef struct {
  float prev_i; /* I component of the previous sample */
  float prev_q; /* Q component of the previous sample */
} FmDemodState;

float dsp_demodulate_fm(FmDemodState *fm, float i, float q) {
  float real = i * fm->prev_i + q * fm->prev_q;
  float imag = q * fm->prev_i - i * fm->prev_q;
  float demod = atan2f(imag, real);
  fm->prev_i = i;
  fm->prev_q = q;
  return demod;
}

/* ============================================================================
 * STAGE 5a: AUDIO DECIMATION (480 kHz -> 48 kHz)
 * ============================================================================
 * The demodulated audio is still at 480 kHz -- ten times higher than our
 * speaker needs. We decimate again, this time by a factor of 10, to reach
 * the final 48 kHz audio rate. Same box-car averaging, but on a single
 * real-valued signal (no I/Q pairs at this point).
 */

typedef struct {
  float acc; /* Running sum of samples */
  int count; /* How many samples accumulated so far */
} AudioDecimState;

bool dsp_decimate_audio(AudioDecimState *ad, float in, float *out) {
  ad->acc += in;
  if (++ad->count >= AUDIO_DECIMATION_FACTOR) {
    *out = ad->acc / (float)AUDIO_DECIMATION_FACTOR;
    ad->acc = 0;
    ad->count = 0;
    return true;
  }
  return false;
}

/* ============================================================================
 * STAGE 5b: LOW-PASS FIR FILTER
 * ============================================================================
 * After demodulation and decimation, our audio still contains unwanted
 * high-frequency content: noise, adjacent-channel interference, the stereo
 * pilot tone at 19 kHz, etc. A low-pass filter keeps only the frequencies
 * within the audible range (<15 kHz) and removes everything above.
 *
 * WHAT IS A FIR FILTER?
 * ----------------------
 * A FIR (Finite Impulse Response) filter computes each output sample as a
 * weighted sum of the last N input samples:
 *
 *   y[n] = coeffs[0]*x[n] + coeffs[1]*x[n-1] + ... + coeffs[N-1]*x[n-N+1]
 *
 * The coefficients (weights) determine the filter's frequency response --
 * which frequencies pass through and which are suppressed. We design them
 * using the "windowed sinc" method (see dsp_design_lowpass_filter below).
 *
 * HOW THE CIRCULAR BUFFER WORKS
 * ------------------------------
 * A naive implementation would shift the entire history array left by one
 * position on every new sample (expensive for 63 taps). Instead, we use a
 * circular buffer: we overwrite the oldest entry at position `index` and
 * advance the index. When computing the weighted sum, we wrap around the
 * array boundaries. This is O(1) for insertion instead of O(N).
 */

#define FIR_TAPS 63

typedef struct {
  float coeffs[FIR_TAPS];  /* Filter weights (designed once at startup) */
  float history[FIR_TAPS]; /* Circular buffer of recent input samples */
  int index;               /* Write position in the circular buffer */
} FirState;

float dsp_filter_audio(FirState *fir, float in) {
  fir->history[fir->index] = in;
  float filtered = 0;
  for (int k = 0; k < FIR_TAPS; k++) {
    int idx = (fir->index + k);
    if (idx >= FIR_TAPS)
      idx -= FIR_TAPS;
    filtered += fir->history[idx] * fir->coeffs[k];
  }
  fir->index = (fir->index + 1);
  if (fir->index >= FIR_TAPS)
    fir->index = 0;
  return filtered;
}

/**
 * Design a low-pass FIR filter using the windowed-sinc method.
 *
 * The ideal low-pass filter in the frequency domain is a rectangle: pass
 * everything below the cutoff, block everything above. In the time domain,
 * this rectangle becomes a "sinc" function: sin(x)/x. But a sinc extends
 * infinitely in both directions, so we must truncate it to `taps` samples.
 *
 * Truncation alone would cause ripples in the frequency response (called
 * "Gibbs phenomenon"). To suppress them, we multiply the truncated sinc
 * by a smooth "window" function that tapers to zero at both ends. We use
 * the Blackman window, which gives excellent sidelobe suppression at the
 * cost of a slightly wider transition band.
 *
 * Finally, we normalize the coefficients so they sum to 1.0, ensuring the
 * filter doesn't change the signal's overall volume.
 */
void dsp_design_lowpass_filter(float *coeffs, int taps, float cutoff_freq,
                               float fs) {
  float f_c = cutoff_freq / fs;
  float sum = 0;
  for (int i = 0; i < taps; i++) {
    int n = i - (taps - 1) / 2;
    if (n == 0)
      coeffs[i] = 2.0f * f_c;
    else
      coeffs[i] = sinf(2.0f * (float)M_PI * f_c * n) / ((float)M_PI * n);

    /* Blackman window */
    float w = 0.42f - 0.5f * cosf(2.0f * (float)M_PI * i / (taps - 1)) +
              0.08f * cosf(4.0f * (float)M_PI * i / (taps - 1));
    coeffs[i] *= w;
    sum += coeffs[i];
  }
  for (int i = 0; i < taps; i++)
    coeffs[i] /= sum;
}

/* ============================================================================
 * STAGE 5c: DE-EMPHASIS
 * ============================================================================
 * FM radio has an inherent problem: high-frequency noise is stronger than
 * low-frequency noise (noise power increases with frequency). To improve
 * the signal-to-noise ratio, FM broadcasters "pre-emphasize" the signal
 * before transmission -- they boost the treble. The receiver must apply
 * the exact inverse, "de-emphasis", to restore the original audio balance.
 * Without this step, FM radio would sound harsh and overly bright.
 *
 * The pre-emphasis / de-emphasis curve is defined by a time constant (tau).
 * This is standardized: 50 microseconds in most of the world, 75 us in the
 * Americas. Our filter implements the de-emphasis as a 1st-order IIR
 * (Infinite Impulse Response) low-pass filter:
 *
 *   y[n] = b0*x[n] + b1*x[n-1] - a1*y[n-1]
 *
 * FIR VS IIR
 * -----------
 * Unlike the FIR filter above (which only uses past *inputs*), an IIR filter
 * also feeds back its own past *outputs*, creating a recursive structure.
 * This makes IIR filters much more efficient for simple frequency shaping --
 * what takes 63 FIR taps can often be done with 2-3 IIR coefficients.
 * The tradeoff is that IIR filters can be unstable if poorly designed, but
 * a 1st-order de-emphasis filter is inherently safe.
 */

#define DEEMPH_TAU 50e-6f

typedef struct {
  float b0, b1, a1; /* Filter coefficients (derived from tau at startup) */
  float x1, y1;     /* Previous input and output (for the feedback loop) */
} DeemphState;

float dsp_deemphasize(DeemphState *d, float in) {
  float out = d->b0 * in + d->b1 * d->x1 - d->a1 * d->y1;
  d->x1 = in;
  d->y1 = out;
  return out;
}

/* ============================================================================
 * STAGE 5d: AUTOMATIC GAIN CONTROL (AGC)
 * ============================================================================
 * Different stations have different signal strengths, and even the same
 * station varies as you move the antenna. The AGC continuously adjusts a
 * gain multiplier to keep the output at a comfortable, consistent volume.
 *
 * How it works:
 *   1. PEAK TRACKING: We maintain a running estimate of the signal's peak
 *      amplitude. New peaks update instantly (so we react immediately to
 *      loud bursts, preventing distortion). When the signal is below the
 *      current peak, the tracker decays slowly (ATTACK_RATE ≈ 0.99995).
 *      This asymmetry prevents the gain from spiking during brief pauses.
 *
 *   2. TARGET GAIN: We compute the gain that would bring the current peak
 *      to TARGET_LEVEL (0.7 = 70% of full scale, leaving headroom).
 *
 *   3. SMOOTHING: The actual gain moves toward the target gain gradually
 *      (controlled by RELEASE_RATE and SMOOTH_RATE) to avoid audible
 *      "pumping" -- sudden volume jumps that would be distracting.
 */

#define AGC_ATTACK_RATE 0.99995f
#define AGC_RELEASE_RATE 0.998f
#define AGC_SMOOTH_RATE 0.002f
#define AGC_TARGET_LEVEL 0.7f

typedef struct {
  float gain; /* Current gain multiplier */
  float peak; /* Tracked peak signal level */
} AgcState;

float dsp_apply_agc(AgcState *agc, float in) {
  float abs_val = fabsf(in);
  if (abs_val > agc->peak)
    agc->peak = abs_val;
  else
    agc->peak *= AGC_ATTACK_RATE;

  if (agc->peak < 0.01f)
    agc->peak = 0.01f;
  float target_gain = AGC_TARGET_LEVEL / agc->peak;
  agc->gain = agc->gain * AGC_RELEASE_RATE + target_gain * AGC_SMOOTH_RATE;
  return in * agc->gain;
}

/* ============================================================================
 * STAGE 6: PCM CONVERSION
 * ============================================================================
 * At this point we have a clean audio signal as floating-point numbers in
 * roughly the range -1.0 to +1.0. Speakers and audio hardware expect
 * samples as 16-bit signed integers (-32768 to +32767). We scale by 32767
 * and clamp ("saturate") to prevent overflow, which would cause a loud pop.
 */

int16_t dsp_saturate_pcm(float in) {
  float pcm_f = in * 32767.0f;
  if (pcm_f > 32767.0f)
    pcm_f = 32767.0f;
  if (pcm_f < -32768.0f)
    pcm_f = -32768.0f;
  return (int16_t)pcm_f;
}

/* ============================================================================
 * THE RING BUFFER (CONNECTING THE TWO THREADS)
 * ============================================================================
 * The HackRF driver thread produces audio samples; the macOS audio thread
 * consumes them. They run at different rates and on different schedules.
 * The ring buffer is a lock-free, single-producer / single-consumer queue
 * that decouples them.
 *
 * HOW IT WORKS
 * -------------
 * The buffer is a fixed-size array with two position counters:
 *   - write_pos: advanced by the producer after storing a sample
 *   - read_pos:  advanced by the consumer after reading a sample
 *
 * Both positions wrap around using a bitmask (RING_MASK). Because the
 * capacity is a power of 2, (pos & RING_MASK) is equivalent to
 * (pos % RING_CAPACITY) but much faster (a single AND instruction vs.
 * an integer division).
 *
 * The buffer is full when (write_pos + 1) == read_pos. This means we
 * "waste" one slot to distinguish full from empty, but that's a trivial
 * cost for 262,144 slots.
 *
 * THREAD SAFETY WITHOUT LOCKS
 * ----------------------------
 * We use atomic operations with acquire/release memory ordering instead of
 * mutexes. This is critical because both callbacks run in real-time contexts
 * where blocking on a lock could cause audio dropouts. The ordering
 * guarantees that when the consumer sees an updated write_pos, the sample
 * data at that position is guaranteed to be visible.
 */

#define RING_CAPACITY (1 << 18) /* 262,144 samples ≈ 5.5 seconds at 48 kHz */
#define RING_MASK (RING_CAPACITY - 1)

/**
 * RadioContext -- the master state for the entire radio.
 *
 * This single structure holds the ring buffer, all DSP stage states, and
 * control flags. It is allocated once at startup and passed to both the
 * HackRF callback and the audio callback via their respective user-data
 * pointers.
 */
typedef struct {
  /* Ring buffer */
  int16_t ring_buffer[RING_CAPACITY];
  atomic_uint_fast32_t ring_write_pos;
  atomic_uint_fast32_t ring_read_pos;

  /* DSP pipeline state (one struct per processing stage) */
  NcoState nco;
  IqDecimState iq_decim;
  FmDemodState fm_demod;
  AudioDecimState audio_decim;
  FirState fir;
  DeemphState deemph;
  AgcState agc;

  /* If the audio thread drains the ring buffer faster than the HackRF
   * fills it, we repeat the last sample instead of outputting silence
   * (which would cause an audible click). */
  int16_t last_audio_sample;

  /* Control flags shared between threads */
  atomic_bool running;       /* false when Ctrl+C is pressed */
  atomic_bool pre_roll_done; /* true once ring buffer has enough data */
} RadioContext;

void radio_push_sample(RadioContext *ctx, int16_t sample) {
  uint32_t write_pos =
      atomic_load_explicit(&ctx->ring_write_pos, memory_order_relaxed);
  uint32_t read_pos =
      atomic_load_explicit(&ctx->ring_read_pos, memory_order_acquire);
  uint32_t next = (write_pos + 1) & RING_MASK;
  if (next != read_pos) {
    ctx->ring_buffer[write_pos] = sample;
    atomic_store_explicit(&ctx->ring_write_pos, next, memory_order_release);
  }
  /* If full, silently drop the sample (better than blocking the
   * HackRF driver thread, which would cause it to lose radio data). */
}

int16_t ring_buffer_pop(RadioContext *ctx) {
  uint32_t read_pos =
      atomic_load_explicit(&ctx->ring_read_pos, memory_order_relaxed);
  uint32_t write_pos =
      atomic_load_explicit(&ctx->ring_write_pos, memory_order_acquire);
  if (read_pos != write_pos) {
    int16_t sample = ctx->ring_buffer[read_pos];
    atomic_store_explicit(&ctx->ring_read_pos, (read_pos + 1) & RING_MASK,
                          memory_order_release);
    return sample;
  }
  /* Buffer empty: repeat the last sample to avoid a click/pop. */
  return ctx->last_audio_sample;
}

/* ============================================================================
 * PUTTING IT ALL TOGETHER: THE CALLBACKS
 * ============================================================================
 * Now that every pipeline stage is defined, we can wire them together.
 * The HackRF callback runs the full DSP pipeline (stages 1-6) and the
 * audio callback pulls the finished samples out of the ring buffer.
 */

/* macOS audio playback uses a triple-buffer scheme: while one buffer is
 * playing, another is being filled, and a third is queued. This prevents
 * gaps in the audio even if our processing momentarily falls behind.
 */
#define AUDIO_BUFFER_COUNT 3
#define AUDIO_BUFFER_SAMPLES 4096

/**
 * HackRF Receive Callback.
 *
 * The HackRF driver calls this on its own thread every time a new batch of
 * raw I/Q samples is ready. Each byte pair in the buffer is one I/Q sample:
 * [I0, Q0, I1, Q1, ...].
 *
 * This is where the entire DSP pipeline runs. For each I/Q pair:
 *   1. Normalize the 8-bit integers to floats
 *   2. Mix down to baseband (frequency shift)
 *   3. Decimate from 1.92 MHz to 480 kHz
 *   4. Demodulate FM (extract audio from phase changes)
 *   5. Decimate from 480 kHz to 48 kHz
 *   6. Filter, de-emphasize, and apply AGC
 *   7. Convert to 16-bit PCM and push into the ring buffer
 *
 * Steps 3 and 5 use "continue" to skip to the next input sample when
 * the decimator hasn't accumulated enough samples for an output yet.
 */
int hackrf_receive_cb(hackrf_transfer *transfer) {
  RadioContext *ctx = (RadioContext *)transfer->rx_ctx;
  if (!ctx || !atomic_load(&ctx->running))
    return -1;

  int8_t *iq_samples = (int8_t *)transfer->buffer;
  int byte_count = transfer->valid_length;

  for (int n = 0; n < byte_count - 1; n += 2) {
    float i_base, q_base, i_480, q_480, audio_raw, audio_final;

    /* Stage 1: Normalize signed 8-bit samples to floats (-1.0..+1.0) */
    float norm_i = (float)iq_samples[n] / HACKRF_RESAMPLING_DIVISOR;
    float norm_q = (float)iq_samples[n + 1] / HACKRF_RESAMPLING_DIVISOR;

    /* Stage 2: Mix to baseband */
    dsp_mix_down(&ctx->nco, norm_i, norm_q, &i_base, &q_base);

    /* Stage 3: Decimate to 480 kHz (wait until 4 samples accumulated) */
    if (!dsp_decimate_iq(&ctx->iq_decim, i_base, q_base, &i_480, &q_480))
      continue;

    /* Stage 4: FM demodulation */
    float demod = dsp_demodulate_fm(&ctx->fm_demod, i_480, q_480);

    /* Stage 5a: Decimate to 48 kHz (wait until 10 samples accumulated) */
    if (!dsp_decimate_audio(&ctx->audio_decim, demod, &audio_raw))
      continue;

    /* Stage 5b-d: Clean up the audio */
    audio_final = dsp_filter_audio(&ctx->fir, audio_raw);
    audio_final = dsp_deemphasize(&ctx->deemph, audio_final);
    audio_final = dsp_apply_agc(&ctx->agc, audio_final);

    /* Stage 6: Convert to PCM and push to ring buffer */
    radio_push_sample(ctx, dsp_saturate_pcm(audio_final));
  }
  return 0;
}

/**
 * macOS Audio Playback Callback.
 *
 * macOS calls this on its audio thread whenever a speaker buffer needs to be
 * refilled (roughly every AUDIO_BUFFER_SAMPLES / 48000 ≈ 85 ms).
 *
 * On the first few calls, the ring buffer may not have enough data yet
 * (the HackRF is still warming up). We output silence until at least
 * 2 * AUDIO_BUFFER_SAMPLES are available -- this "pre-roll" prevents
 * initial stuttering.
 */
void audio_playback_cb(void *userdata, AudioQueueRef queue,
                       AudioQueueBufferRef buffer) {
  RadioContext *ctx = (RadioContext *)userdata;
  int16_t *output = (int16_t *)buffer->mAudioData;

  /* Pre-roll: wait for the ring buffer to accumulate enough data */
  if (!atomic_load(&ctx->pre_roll_done)) {
    uint32_t count =
        (atomic_load(&ctx->ring_write_pos) - atomic_load(&ctx->ring_read_pos)) &
        RING_MASK;
    if (count < AUDIO_BUFFER_SAMPLES * 2) {
      memset(output, 0, AUDIO_BUFFER_SAMPLES * sizeof(int16_t));
      buffer->mAudioDataByteSize = AUDIO_BUFFER_SAMPLES * sizeof(int16_t);
      AudioQueueEnqueueBuffer(queue, buffer, 0, NULL);
      return;
    }
    atomic_store(&ctx->pre_roll_done, true);
  }

  /* Normal operation: pull samples from the ring buffer */
  for (int i = 0; i < AUDIO_BUFFER_SAMPLES; i++) {
    ctx->last_audio_sample = ring_buffer_pop(ctx);
    output[i] = ctx->last_audio_sample;
  }

  buffer->mAudioDataByteSize = AUDIO_BUFFER_SAMPLES * sizeof(int16_t);
  AudioQueueEnqueueBuffer(queue, buffer, 0, NULL);
}

/* ============================================================================
 * INITIALIZATION
 * ============================================================================
 * Create and configure the RadioContext with all DSP stages ready to go.
 */

RadioContext *radio_context_create(void) {
  RadioContext *ctx = calloc(1, sizeof(RadioContext));
  if (!ctx)
    return NULL;

  atomic_init(&ctx->running, true);
  atomic_init(&ctx->pre_roll_done, false);

  /* NCO: shift by +IF_OFFSET Hz to compensate for the hardware being
   * tuned IF_OFFSET Hz below our target frequency. */
  dsp_nco_init(&ctx->nco, IF_OFFSET, HACKRF_SAMPLE_RATE);

  /* FIR filter: 15 kHz cutoff keeps audio, removes everything above
   * (including the 19 kHz stereo pilot tone). */
  dsp_design_lowpass_filter(ctx->fir.coeffs, FIR_TAPS, 15000.0f,
                            (float)AUDIO_SAMPLE_RATE);

  /* De-emphasis: convert the analog RC time constant (tau) into digital
   * IIR coefficients using the bilinear transform:
   *   k = 2 * sample_rate * tau
   *   b0 = b1 = 1 / (1 + k)
   *   a1 = (1 - k) / (1 + k)
   */
  float k = 2.0f * (float)AUDIO_SAMPLE_RATE * DEEMPH_TAU;
  ctx->deemph.b0 = 1.0f / (1.0f + k);
  ctx->deemph.b1 = ctx->deemph.b0;
  ctx->deemph.a1 = (1.0f - k) / (1.0f + k);

  /* AGC: start with unity gain and a moderate peak estimate. */
  ctx->agc.gain = 1.0f;
  ctx->agc.peak = 0.5f;

  /* FM demod: initialize prev_i to 1.0 (not 0.0) so the first call to
   * atan2 doesn't get atan2(0, 0) which is undefined. */
  ctx->fm_demod.prev_i = 1.0f;

  return ctx;
}

/* ============================================================================
 * MAIN ENTRY POINT
 * ============================================================================
 * Parse arguments, open devices, start streaming, and wait for Ctrl+C.
 */

static RadioContext *g_ctx_ptr = NULL;
void handle_stop(int sig) {
  (void)sig;
  if (g_ctx_ptr)
    atomic_store(&g_ctx_ptr->running, false);
}

int main(int argc, char *argv[]) {
  uint64_t frequency = DEFAULT_FREQUENCY_HZ;
  int opt;
  while ((opt = getopt(argc, argv, "f:h")) != -1) {
    if (opt == 'f')
      frequency = strtoull(optarg, NULL, 10);
    else {
      fprintf(stderr, "Usage: %s [-f freq_hz]\n", argv[0]);
      return 1;
    }
  }

  RadioContext *ctx = radio_context_create();
  g_ctx_ptr = ctx;
  signal(SIGINT, handle_stop);

  printf("FM Radio Receiver\n");
  printf("Tuning to: %.1f MHz\n", (double)frequency / 1e6);

  /* --- Set up macOS audio output ---
   * Configure a mono, 16-bit, 48 kHz audio stream. AudioQueue manages the
   * speaker hardware and calls audio_playback_cb whenever it needs samples.
   */
  AudioStreamBasicDescription format = {
      .mSampleRate = AUDIO_SAMPLE_RATE,
      .mFormatID = kAudioFormatLinearPCM,
      .mFormatFlags =
          kLinearPCMFormatFlagIsSignedInteger | kLinearPCMFormatFlagIsPacked,
      .mBytesPerPacket = 2,
      .mFramesPerPacket = 1,
      .mBytesPerFrame = 2,
      .mChannelsPerFrame = 1,
      .mBitsPerChannel = 16,
  };
  AudioQueueRef queue;
  AudioQueueNewOutput(&format, audio_playback_cb, ctx, NULL, NULL, 0, &queue);
  for (int i = 0; i < AUDIO_BUFFER_COUNT; i++) {
    AudioQueueBufferRef buf;
    AudioQueueAllocateBuffer(queue, AUDIO_BUFFER_SAMPLES * 2, &buf);
    buf->mAudioDataByteSize = AUDIO_BUFFER_SAMPLES * 2;
    memset(buf->mAudioData, 0, buf->mAudioDataByteSize);
    AudioQueueEnqueueBuffer(queue, buf, 0, NULL);
  }

  /* --- Set up HackRF ---
   * Configure sample rate, center frequency (offset to avoid DC spike),
   * and RF gain stages. Then start receiving -- the driver will call
   * hackrf_receive_cb on its own thread with batches of raw I/Q samples.
   */
  hackrf_device *device = NULL;
  hackrf_init();
  if (hackrf_open(&device) != HACKRF_SUCCESS) {
    fprintf(stderr, "Error: HackRF not found!\n");
    return 1;
  }
  hackrf_set_sample_rate(device, HACKRF_SAMPLE_RATE);
  hackrf_set_freq(device, frequency - IF_OFFSET);
  hackrf_set_lna_gain(device, LNA_GAIN);
  hackrf_set_vga_gain(device, VGA_GAIN);
  hackrf_start_rx(device, hackrf_receive_cb, ctx);

  /* --- Run until Ctrl+C --- */
  AudioQueueStart(queue, NULL);
  printf("Listening... (Press Ctrl+C to exit)\n");
  while (atomic_load(&ctx->running))
    usleep(100000);

  /* --- Shutdown ---
   * Stop the HackRF first (stop producing), then audio (stop consuming),
   * then free resources.
   */
  printf("\nStopping...\n");
  hackrf_stop_rx(device);
  hackrf_close(device);
  hackrf_exit();
  AudioQueueStop(queue, true);
  AudioQueueDispose(queue, true);
  free(ctx);

  return 0;
}
