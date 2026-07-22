#ifndef DAC_NCO_H
#define DAC_NCO_H

#include "signal_fft.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DAC_NCO_SAMPLE_RATE_HZ 4000000U

/**
  * @brief  Starts two continuous DAC NCO output streams.
  * @param  signalA Signal routed to PA4.
  * @param  signalB Signal routed to PA5.
  * @retval true when both DMA streams and TIM6 start successfully.
  */
bool DacNco_Start(const SignalFftSeparatedSignal* signalA, const SignalFftSeparatedSignal* signalB);

/**
  * @brief  Updates channel A frequency without resetting its phase accumulator.
  * @param  frequencyHz New channel A frequency in hertz.
  */
void DacNco_SetChannelAFrequency(float32_t frequencyHz);

/**
  * @brief  Updates channel B frequency without resetting its phase accumulator.
  * @param  frequencyHz New channel B frequency in hertz.
  */
void DacNco_SetChannelBFrequency(float32_t frequencyHz);

/**
  * @brief  Prints periodic DMA refill and error diagnostics.
  */
void DacNco_ProcessDiagnostics(void);

#ifdef __cplusplus
}
#endif

#endif /* DAC_NCO_H */
