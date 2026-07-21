#ifndef SIGNAL_ZOOM_FFT_H
#define SIGNAL_ZOOM_FFT_H

#include "arm_math.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SIGNAL_ZOOM_FFT_SIZE                         4096U
#define SIGNAL_ZOOM_FFT_DECIMATION_FACTOR            64U
#define SIGNAL_ZOOM_FFT_SEARCH_HALF_BANDWIDTH_HZ     6000U
#define SIGNAL_ZOOM_FFT_MIN_PEAK_SEPARATION_BINS     4U
#define SIGNAL_ZOOM_FFT_SECOND_PEAK_MIN_POWER_RATIO  0.0025f
#define SIGNAL_ZOOM_FFT_MIN_PEAK_VPP_CODE            50.0f

typedef enum {
    SIGNAL_ZOOM_FFT_PATTERN_NONE = 0,
    SIGNAL_ZOOM_FFT_PATTERN_SINGLE_CLUSTER,
    SIGNAL_ZOOM_FFT_PATTERN_TWO_SEPARATED
} SignalZoomFftPattern;

typedef struct {
    bool valid;
    int32_t peakBin;
    float32_t binOffset;
    float32_t basebandFrequencyHz;
    float32_t frequencyHz;
    float32_t magnitude;
    float32_t amplitudePeakCode;
    float32_t amplitudePeakToPeakCode;
} SignalZoomFftPeak;

typedef struct {
    bool valid;
    SignalZoomFftPattern pattern;
    float32_t centerFrequencyHz;
    uint32_t inputSampleRateHz;
    uint32_t outputSampleRateHz;
    float32_t binResolutionHz;
    uint32_t rawSampleCount;
    float32_t captureTimeUs;
    uint32_t ddcCycles;
    uint32_t maximumDdcCallbackCycles;
    uint32_t ddcOverrunCount;
    uint32_t fftCycles;
    uint32_t analysisCycles;
    float32_t ddcTimeUs;
    float32_t maximumDdcCallbackTimeUs;
    float32_t fftTimeUs;
    float32_t analysisTimeUs;
    SignalZoomFftPeak dominantPeak;
    SignalZoomFftPeak signalA;
    SignalZoomFftPeak signalB;
} SignalZoomFftResult;

/**
  * @brief  Initializes the fixed 4096-point complex FFT.
  * @retval ARM_MATH_SUCCESS on success.
  */
arm_status SignalZoomFFT_Init(void);

/**
  * @brief  Starts one DDC and Zoom FFT acquisition.
  * @param  centerFrequencyHz Coarse cluster center frequency.
  * @param  meanCode ADC mean code estimated by the coarse FFT.
  * @param  inputSampleRateHz Raw ADC sample rate.
  * @retval ARM_MATH_SUCCESS on success.
  */
arm_status SignalZoomFFT_Start(float32_t centerFrequencyHz, float32_t meanCode, uint32_t inputSampleRateHz);

/**
  * @brief  Pushes consecutive raw ADC samples into the streaming DDC.
  * @param  samples Pointer to consecutive raw ADC samples.
  * @param  sampleCount Number of samples.
  * @note   This function is designed to run from the ADC DMA callback.
  */
void SignalZoomFFT_PushSamples(const uint16_t* samples, uint32_t sampleCount);

/**
  * @brief  Checks whether 4096 valid complex baseband samples are ready.
  * @retval true when capture is complete.
  */
bool SignalZoomFFT_IsCaptureComplete(void);

/**
  * @brief  Runs the 4096-point complex Zoom FFT and searches for two peaks.
  * @param  result Output result.
  * @retval true when at least one valid Zoom FFT peak is found.
  */
bool SignalZoomFFT_Analyze(SignalZoomFftResult* result);

/**
  * @brief  Stops and clears the current acquisition state.
  */
void SignalZoomFFT_Abort(void);

#ifdef __cplusplus
}
#endif

#endif /* SIGNAL_ZOOM_FFT_H */
