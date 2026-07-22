#ifndef IQ_PHASE_MONITOR_H
#define IQ_PHASE_MONITOR_H

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  Starts the channel A and B IQ digital PLLs.
  * @param  frequencyAHz Initial channel A PLL and DAC NCO frequency.
  * @param  frequencyBHz Initial channel B PLL and DAC NCO frequency.
  * @param  meanCode ADC DC mean estimated by FFT analysis.
  */
void IqPhaseMonitor_Start(float frequencyAHz, float frequencyBHz, float meanCode);

/**
  * @brief  Processes queued IQ blocks, updates the PLL, and prints diagnostics.
  */
void IqPhaseMonitor_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* IQ_PHASE_MONITOR_H */
