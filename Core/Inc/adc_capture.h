#ifndef ADC_CAPTURE_H
#define ADC_CAPTURE_H

#include "main.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ADC capture configuration. */
#define ADC_CAPTURE_BUFFER_SIZE                 4096U
#define ADC_CAPTURE_BUFFER_ADDRESS              0x3800E000UL
#define ADC_CAPTURE_SAMPLE_RATE_HZ              3200000U

/* Periodic sample-rate report. */
#define ADC_CAPTURE_RATE_REPORT_ENABLE          1U
#define ADC_CAPTURE_RATE_REPORT_INTERVAL_MS     1000U

/*
 * Keep this disabled while the main loop consumes the snapshot for FFT.
 * Set to 1 only when you want the ADC module itself to print all samples.
 */
#define ADC_CAPTURE_PRINT_ALL_SAMPLES           0U
#define ADC_CAPTURE_PRINT_VALUES_PER_LINE       16U

/**
  * @brief  Calibrates ADC3 and starts ADC3, BDMA, and TIM2.
  * @retval HAL status.
  */
HAL_StatusTypeDef ADC_Capture_Start(void);

/**
  * @brief  Services rate reporting and optional sample printing.
  * @note   Call continuously from the main loop.
  */
void ADC_Capture_Process(void);

/**
  * @brief  Requests a new coherent 4096-sample snapshot.
  * @note   The request is completed at the next DMA half/full boundary pair.
  */
void ADC_Capture_RequestSnapshot(void);

/**
  * @brief  Checks whether a coherent snapshot is ready for CPU processing.
  * @retval 1 when ready, otherwise 0.
  */
uint8_t ADC_Capture_IsSnapshotReady(void);

/**
  * @brief  Gets the most recently captured coherent snapshot.
  * @retval Pointer to ADC_CAPTURE_BUFFER_SIZE samples.
  */
const uint16_t* ADC_Capture_GetSnapshot(void);

/**
  * @brief  Releases the current snapshot after CPU processing is complete.
  */
void ADC_Capture_ReleaseSnapshot(void);

#ifdef __cplusplus
}
#endif

#endif /* ADC_CAPTURE_H */
