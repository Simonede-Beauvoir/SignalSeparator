#include "adc_capture.h"

#include "adc.h"
#include "tim.h"
#include "usart.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

typedef enum {
    ADC_SNAPSHOT_IDLE = 0,
    ADC_SNAPSHOT_WAIT_HALF,
    ADC_SNAPSHOT_WAIT_FULL,
    ADC_SNAPSHOT_READY
} ADC_SnapshotState;

#define ADC_CAPTURE_HALF_BUFFER_SIZE \
    (ADC_CAPTURE_BUFFER_SIZE / 2U)

static uint16_t* const adcBuffer = (uint16_t*)ADC_CAPTURE_BUFFER_ADDRESS;

#if defined(__GNUC__)
#define BDMA_BUFFER \
    __attribute__((section(".bdma_buffer"), aligned(32)))
#else
#define BDMA_BUFFER
#endif
static uint16_t adcSnapshot[ADC_CAPTURE_BUFFER_SIZE] BDMA_BUFFER;

static volatile ADC_SnapshotState adcSnapshotState = ADC_SNAPSHOT_IDLE;
static ADC_CaptureStreamCallback volatile adcStreamCallback = NULL;

static volatile uint32_t adcHalfCount = 0U;
static volatile uint32_t adcFullCount = 0U;
static volatile uint32_t adcErrorCount = 0U;

static uint32_t adcRateTestStartTick = 0U;
static uint32_t adcRateTestLastFullCount = 0U;

static void ADC_Capture_PrintSnapshot(void);
static void ADC_Capture_ReportRate(void);
static void ADC_Capture_InvokeStreamCallback(const uint16_t* samples, uint32_t sampleCount);

HAL_StatusTypeDef ADC_Capture_Start(void) {
    adcHalfCount = 0U;
    adcFullCount = 0U;
    adcErrorCount = 0U;
    adcSnapshotState = ADC_SNAPSHOT_IDLE;
    adcStreamCallback = NULL;

    if (HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) {
        return HAL_ERROR;
    }

    if (HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adcBuffer, ADC_CAPTURE_BUFFER_SIZE) != HAL_OK) {
        return HAL_ERROR;
    }

    if (HAL_TIM_Base_Start(&htim2) != HAL_OK) {
        (void)HAL_ADC_Stop_DMA(&hadc3);
        return HAL_ERROR;
    }

    adcRateTestStartTick = HAL_GetTick();
    adcRateTestLastFullCount = adcFullCount;

#if ADC_CAPTURE_PRINT_ALL_SAMPLES
    ADC_Capture_RequestSnapshot();
#endif

    return HAL_OK;
}

void ADC_Capture_Process(void) {
#if ADC_CAPTURE_PRINT_ALL_SAMPLES
    if (adcSnapshotState == ADC_SNAPSHOT_READY) {
        ADC_Capture_PrintSnapshot();
        ADC_Capture_ReleaseSnapshot();
    }
#endif

#if ADC_CAPTURE_RATE_REPORT_ENABLE
    ADC_Capture_ReportRate();
#endif
}

void ADC_Capture_RequestSnapshot(void) {
    if (adcSnapshotState == ADC_SNAPSHOT_IDLE) {
        adcSnapshotState = ADC_SNAPSHOT_WAIT_HALF;
    }
}

uint8_t ADC_Capture_IsSnapshotReady(void) {
    return adcSnapshotState == ADC_SNAPSHOT_READY ? 1U : 0U;
}

const uint16_t* ADC_Capture_GetSnapshot(void) {
    return adcSnapshot;
}

void ADC_Capture_ReleaseSnapshot(void) {
    if (adcSnapshotState == ADC_SNAPSHOT_READY) {
        adcSnapshotState = ADC_SNAPSHOT_IDLE;
    }
}

void ADC_Capture_SetStreamCallback(ADC_CaptureStreamCallback callback) {
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    adcStreamCallback = callback;

    if (primask == 0U) {
        __enable_irq();
    }
}

void ADC_Capture_ClearStreamCallback(void) {
    ADC_Capture_SetStreamCallback(NULL);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance != ADC3) {
        return;
    }

    adcHalfCount++;

    if (adcSnapshotState == ADC_SNAPSHOT_WAIT_HALF) {
        memcpy(&adcSnapshot[0], &adcBuffer[0], ADC_CAPTURE_HALF_BUFFER_SIZE * sizeof(uint16_t));

        adcSnapshotState = ADC_SNAPSHOT_WAIT_FULL;
    }

    ADC_Capture_InvokeStreamCallback(&adcBuffer[0], ADC_CAPTURE_HALF_BUFFER_SIZE);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance != ADC3) {
        return;
    }

    adcFullCount++;

    if (adcSnapshotState == ADC_SNAPSHOT_WAIT_FULL) {
        memcpy(&adcSnapshot[ADC_CAPTURE_HALF_BUFFER_SIZE], &adcBuffer[ADC_CAPTURE_HALF_BUFFER_SIZE], ADC_CAPTURE_HALF_BUFFER_SIZE * sizeof(uint16_t));

        adcSnapshotState = ADC_SNAPSHOT_READY;
    }

    ADC_Capture_InvokeStreamCallback(&adcBuffer[ADC_CAPTURE_HALF_BUFFER_SIZE], ADC_CAPTURE_HALF_BUFFER_SIZE);
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC3) {
        adcErrorCount++;
    }
}

static void ADC_Capture_InvokeStreamCallback(const uint16_t* samples, uint32_t sampleCount) {
    ADC_CaptureStreamCallback callback = adcStreamCallback;

    if (callback != NULL) {
        callback(samples, sampleCount);
    }
}

static void ADC_Capture_PrintSnapshot(void) {
    char line[128];
    uint32_t index = 0U;

    printf("\r\n[ADC_BLOCK_BEGIN] count=%lu rate=%lu\r\n", (unsigned long)ADC_CAPTURE_BUFFER_SIZE, (unsigned long)ADC_CAPTURE_SAMPLE_RATE_HZ);

    while (index < ADC_CAPTURE_BUFFER_SIZE) {
        size_t length = 0U;

        for (uint32_t column = 0U; column < ADC_CAPTURE_PRINT_VALUES_PER_LINE && index < ADC_CAPTURE_BUFFER_SIZE; column++, index++) {
            int written = snprintf(&line[length], sizeof(line) - length, "%u%s", (unsigned int)adcSnapshot[index],
                                   (column + 1U == ADC_CAPTURE_PRINT_VALUES_PER_LINE || index + 1U == ADC_CAPTURE_BUFFER_SIZE) ? "\r\n" : " ");

            if (written < 0 || (size_t)written >= sizeof(line) - length) {
                printf("[ADC_BLOCK_ERROR] formatting failed\r\n");
                return;
            }

            length += (size_t)written;
        }

        if (HAL_UART_Transmit(&huart1, (uint8_t*)line, (uint16_t)length, 1000U) != HAL_OK) {
            printf("[ADC_BLOCK_ERROR] UART transmission failed\r\n");
            return;
        }
    }

    printf("[ADC_BLOCK_END]\r\n");
}

static void ADC_Capture_ReportRate(void) {
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = (uint32_t)(now - adcRateTestStartTick);

    if (elapsed < ADC_CAPTURE_RATE_REPORT_INTERVAL_MS) {
        return;
    }

    uint32_t currentFullCount = adcFullCount;
    uint32_t fullCountDelta = currentFullCount - adcRateTestLastFullCount;

    uint64_t sampleCount = (uint64_t)fullCountDelta * ADC_CAPTURE_BUFFER_SIZE;

    uint32_t measuredSampleRate = (uint32_t)((sampleCount * 1000ULL) / elapsed);

    printf("ADC rate: %lu samples/s, Half=%lu, " "Full=%lu, Error=%lu\r\n", (unsigned long)measuredSampleRate, (unsigned long)adcHalfCount,
           (unsigned long)adcFullCount, (unsigned long)adcErrorCount);

    adcRateTestLastFullCount = currentFullCount;
    adcRateTestStartTick = now;
}
