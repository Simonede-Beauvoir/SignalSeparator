#ifndef SIGNAL_FFT_H
#define SIGNAL_FFT_H

#include "arm_math.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SIGNAL_FFT_SIZE                     4096U
#define SIGNAL_FFT_MIN_FREQUENCY_HZ         20000U
#define SIGNAL_FFT_MAX_FREQUENCY_HZ         100000U
#define SIGNAL_FFT_NOMINAL_STEP_HZ          5000U

typedef struct {
    bool valid;
    uint32_t peakBin;
    float32_t binOffset;
    float32_t frequencyHz;
    float32_t nominalFrequencyHz;
    float32_t magnitude;
    float32_t amplitudePeakCode;
    float32_t amplitudePeakToPeakCode;
} SignalFftPeak;

typedef struct {
    bool valid;
    float32_t meanCode;
    float32_t binResolutionHz;
    SignalFftPeak signalA;
    SignalFftPeak signalB;
    uint32_t fftCycles;
    uint32_t totalCycles;
    float32_t fftTimeUs;
    float32_t totalTimeUs;
} SignalFftResult;

/**
  * @brief  Initializes the fixed 4096-point real FFT and Hann window.
  * @retval ARM_MATH_SUCCESS on success.
  */
arm_status SignalFFT_Init(void);

/**
  * @brief  Analyzes one coherent 4096-sample ADC snapshot.
  * @param  samples Pointer to unsigned 12-bit ADC samples.
  * @param  sampleCount Number of samples; must equal SIGNAL_FFT_SIZE.
  * @param  sampleRateHz ADC sample rate in samples per second.
  * @param  result Output analysis result.
  * @retval true when two valid peaks are found, otherwise false.
  */
bool SignalFFT_Analyze(
    const uint16_t* samples,
    uint32_t sampleCount,
    uint32_t sampleRateHz,
    SignalFftResult* result);

#ifdef __cplusplus
}
#endif

#endif /* SIGNAL_FFT_H */
