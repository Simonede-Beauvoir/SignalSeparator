#ifndef SIGNAL_FFT_H
#define SIGNAL_FFT_H

#include "arm_math.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SIGNAL_FFT_SIZE                              4096U
#define SIGNAL_FFT_MIN_FREQUENCY_HZ                  20000U
#define SIGNAL_FFT_MAX_FREQUENCY_HZ                  100000U
#define SIGNAL_FFT_PEAK_SEARCH_MAX_FREQUENCY_HZ      500000U
#define SIGNAL_FFT_CANDIDATE_MERGE_BINS              1.0f
#define SIGNAL_FFT_MAX_SIGNIFICANT_PEAKS              32U
#define SIGNAL_FFT_MAX_FUNDAMENTAL_CANDIDATES          8U
#define SIGNAL_FFT_MIN_PEAK_VPP_CODE                 50.0f
#define SIGNAL_FFT_SIGNIFICANT_POWER_RATIO            0.0004f
#define SIGNAL_FFT_MODEL_PHASE_STEP_DEGREES            2U
#define SIGNAL_FFT_MODEL_MAX_RESIDUAL_RATIO            0.15f

typedef enum {
    SIGNAL_WAVEFORM_UNKNOWN = 0,
    SIGNAL_WAVEFORM_SINE,
    SIGNAL_WAVEFORM_TRIANGLE,
    SIGNAL_WAVEFORM_SQUARE
} SignalWaveformType;

typedef struct {
    bool valid;
    uint32_t peakBin;
    float32_t binOffset;
    float32_t frequencyHz;
    float32_t nominalFrequencyHz;
    float32_t magnitude;
    float32_t complexReal;
    float32_t complexImaginary;
    float32_t amplitudePeakCode;
    float32_t amplitudePeakToPeakCode;
} SignalFftPeak;

typedef struct {
    bool valid;
    float32_t frequencyHz;
    SignalWaveformType waveform;
    float32_t amplitudePeakCode;
    float32_t phaseDegrees;
} SignalFftSeparatedSignal;

typedef struct {
    bool valid;
    uint32_t significantPeakCount;
    uint32_t fundamentalCandidateCount;
    float32_t meanCode;
    float32_t binResolutionHz;
    float32_t residualRatio;
    SignalFftPeak significantPeaks[SIGNAL_FFT_MAX_SIGNIFICANT_PEAKS];
    SignalFftSeparatedSignal signalA;
    SignalFftSeparatedSignal signalB;
    uint32_t fftCycles;
    uint32_t totalCycles;
    float32_t fftTimeUs;
    float32_t totalTimeUs;
} SignalFftResult;

/**
  * @brief  Initializes the fixed 4096-point real FFT and model workspace.
  * @retval ARM_MATH_SUCCESS on success.
  */
arm_status SignalFFT_Init(void);

/**
  * @brief  Separates two sine, triangle, or square signals using complex FFT models.
  * @param  samples Pointer to unsigned 12-bit ADC samples.
  * @param  sampleCount Number of samples; must equal SIGNAL_FFT_SIZE.
  * @param  sampleRateHz ADC sample rate in samples per second.
  * @param  result Output analysis result.
  * @retval true when a valid two-signal model is found, otherwise false.
  */
bool SignalFFT_Analyze(const uint16_t* samples, uint32_t sampleCount, uint32_t sampleRateHz, SignalFftResult* result);

/**
  * @brief  Fits the retained coarse spectrum at two externally resolved fundamentals.
  * @param  frequencyAHz First fundamental frequency resolved by Zoom FFT.
  * @param  frequencyBHz Second fundamental frequency resolved by Zoom FFT.
  * @param  result Coarse FFT result to update with the selected harmonic model.
  * @retval true when an acceptable sine, triangle, or square two-signal model is found.
  * @note   SignalFFT_Analyze must be called first.
  */
bool SignalFFT_FitResolvedFundamentals(uint32_t frequencyAHz, uint32_t frequencyBHz, SignalFftResult* result);

/**
  * @brief  Returns a printable name for a waveform type.
  * @param  waveform Waveform type.
  * @retval Constant waveform name string.
  */
const char* SignalFFT_GetWaveformName(SignalWaveformType waveform);

#ifdef __cplusplus
}
#endif

#endif /* SIGNAL_FFT_H */
