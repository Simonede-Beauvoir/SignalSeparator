#ifndef SQUARE_WAVE_OUTPUT_H
#define SQUARE_WAVE_OUTPUT_H

#include "signal_fft.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  Selects the GPIO square-wave path for channels classified as square.
  * @param  signalA Separated channel A signal model.
  * @param  signalB Separated channel B signal model.
  */
void SquareWaveOutput_Start(const SignalFftSeparatedSignal* signalA, const SignalFftSeparatedSignal* signalB);

/**
  * @brief  Updates channel A square-wave frequency while preserving edge timing.
  * @param  frequencyHz New channel A frequency in hertz.
  */
void SquareWaveOutput_SetChannelAFrequency(float32_t frequencyHz);

/**
  * @brief  Updates channel B square-wave frequency while preserving edge timing.
  * @param  frequencyHz New channel B frequency in hertz.
  */
void SquareWaveOutput_SetChannelBFrequency(float32_t frequencyHz);

/**
  * @brief  Prints edge and late-scheduling diagnostics for both channels.
  */
void SquareWaveOutput_PrintDiagnostics(void);

#ifdef __cplusplus
}
#endif

#endif /* SQUARE_WAVE_OUTPUT_H */
