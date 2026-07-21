/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "bdma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc_capture.h"
#include "signal_fft.h"
#include "signal_zoom_fft.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    SIGNAL_ANALYSIS_WAIT_COARSE = 0,
    SIGNAL_ANALYSIS_WAIT_ZOOM,
    SIGNAL_ANALYSIS_COMPLETE,
    SIGNAL_ANALYSIS_ERROR
} SignalAnalysisState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static SignalAnalysisState signalAnalysisState = SIGNAL_ANALYSIS_WAIT_COARSE;
static SignalFftResult coarseFftResult;
static bool coarseModelFallbackAvailable;
static uint8_t uartRxByte;
static char uartCommandBuffer[16];
static volatile uint32_t uartCommandLength;
static volatile bool uartCommandReady;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
static void ProcessCoarseFFT(void);
static void ProcessZoomFFT(void);
static void ProcessSerialCommand(void);
static void PrintCoarseFFTResult(const SignalFftResult* result);
static void PrintZoomFFTResult(const SignalZoomFftResult* result);
static bool FindUnresolvedFundamentalCluster(const SignalFftResult* result, float32_t* centerFrequencyHz);
static uint32_t RoundFloatToUInt32(float32_t value);
static int32_t RoundFloatToInt32(float32_t value);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MPU Configuration--------------------------------------------------------*/
    MPU_Config();

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* Configure the peripherals common clocks */
    PeriphCommonClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_BDMA_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_ADC3_Init();
    MX_TIM2_Init();
    /* USER CODE BEGIN 2 */
    setvbuf(stdout, NULL, _IONBF, 0);

    if (HAL_UART_Receive_IT(&huart1, &uartRxByte, 1U) != HAL_OK) {
        printf("ERROR: USART1 command reception start failed.\r\n");
        Error_Handler();
    }

    printf("\r\n");
    printf("========================================\r\n");
    printf("Signal separator Zoom FFT and harmonic model test\r\n");
    printf("Coarse FFT: %lu points at %lu samples/s\r\n", (unsigned long)SIGNAL_FFT_SIZE, (unsigned long)ADC_CAPTURE_SAMPLE_RATE_HZ);
    printf("Peak search: %lu to %lu Hz\r\n", (unsigned long)SIGNAL_FFT_MIN_FREQUENCY_HZ, (unsigned long)SIGNAL_FFT_PEAK_SEARCH_MAX_FREQUENCY_HZ);
    printf("Models: sine/triangle, complex harmonic overlap\r\n");
    printf("Zoom FFT: %lu complex points, decimation=%lu\r\n", (unsigned long)SIGNAL_ZOOM_FFT_SIZE, (unsigned long)SIGNAL_ZOOM_FFT_DECIMATION_FACTOR);
    printf("Commands: RESET\r\n");
    printf("========================================\r\n");

    if (SignalFFT_Init() != ARM_MATH_SUCCESS) {
        printf("ERROR: coarse CMSIS-DSP FFT initialization " "failed.\r\n");
        Error_Handler();
    }

    if (SignalZoomFFT_Init() != ARM_MATH_SUCCESS) {
        printf("ERROR: CMSIS-DSP Zoom FFT initialization failed.\r\n");
        Error_Handler();
    }

    if (ADC_Capture_Start() != HAL_OK) {
        printf("ERROR: ADC capture start failed.\r\n");
        Error_Handler();
    }

    ADC_Capture_RequestSnapshot();

    printf("ADC3 sampling started.\r\n");
    printf("One coarse 4096-sample FFT requested.\r\n");

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        ProcessSerialCommand();
        ADC_Capture_Process();

        switch (signalAnalysisState) {
            case SIGNAL_ANALYSIS_WAIT_COARSE:
                ProcessCoarseFFT();
                break;

            case SIGNAL_ANALYSIS_WAIT_ZOOM:
                ProcessZoomFFT();
                break;

            case SIGNAL_ANALYSIS_COMPLETE:
            case SIGNAL_ANALYSIS_ERROR: default:
                break;
        }

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Supply configuration update enable
    */
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

    /** Configure the main internal regulator output voltage
    */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 5;
    RCC_OscInitStruct.PLL.PLLN = 192;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1 |
        RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void) {
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    /** Initializes the peripherals clock
    */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
    PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
static void ProcessSerialCommand(void) {
    if (!uartCommandReady) {
        return;
    }

    printf("ERROR: unknown command: %s\r\n", uartCommandBuffer);
    uartCommandLength = 0U;
    uartCommandReady = false;
    (void)HAL_UART_Receive_IT(&huart1, &uartRxByte, 1U);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    if (huart->Instance != USART1) {
        return;
    }

    if (uartRxByte == '\r' || uartRxByte == '\n') {
        if (uartCommandLength > 0U) {
            uartCommandBuffer[uartCommandLength] = '\0';

            if (strcmp(uartCommandBuffer, "RESET") == 0) {
                NVIC_SystemReset();
            }

            uartCommandReady = true;
            return;
        }
    }
    else if (uartCommandLength < sizeof(uartCommandBuffer) - 1U) {
        uartCommandBuffer[uartCommandLength] = (char)uartRxByte;
        uartCommandLength++;
    }
    else {
        uartCommandLength = 0U;
    }

    (void)HAL_UART_Receive_IT(&huart1, &uartRxByte, 1U);
}

static void ProcessCoarseFFT(void) {
    if (ADC_Capture_IsSnapshotReady() == 0U) {
        return;
    }

    bool success = SignalFFT_Analyze(ADC_Capture_GetSnapshot(), ADC_CAPTURE_BUFFER_SIZE, ADC_CAPTURE_SAMPLE_RATE_HZ, &coarseFftResult);

    PrintCoarseFFTResult(&coarseFftResult);
    ADC_Capture_ReleaseSnapshot();

    float32_t zoomCenterFrequencyHz = 0.0f;
    bool unresolvedCluster = FindUnresolvedFundamentalCluster(&coarseFftResult, &zoomCenterFrequencyHz);

    /*
     * A valid coarse model can still be ambiguous. For example, the third
     * harmonic of a triangle wave may be fitted as an independent sine while
     * two close fundamentals remain inside one coarse FFT main lobe. Resolve
     * that low-frequency cluster with Zoom FFT before accepting the coarse
     * model. Keep the coarse result as a fallback for a genuine fundamental
     * plus an unrelated sine near an odd-harmonic frequency.
     */
    if (success && !unresolvedCluster) {
        printf("Decision: complex harmonic model accepted directly from coarse FFT.\r\n");
        printf("Final signals: A=%lu Hz %s, B=%lu Hz %s\r\n", (unsigned long)RoundFloatToUInt32(coarseFftResult.signalA.frequencyHz),
               SignalFFT_GetWaveformName(coarseFftResult.signalA.waveform), (unsigned long)RoundFloatToUInt32(coarseFftResult.signalB.frequencyHz),
               SignalFFT_GetWaveformName(coarseFftResult.signalB.waveform));
        signalAnalysisState = SIGNAL_ANALYSIS_COMPLETE;
        return;
    }

    coarseModelFallbackAvailable = success;

    if (coarseFftResult.significantPeakCount == 0U) {
        printf("Coarse FFT failed: no significant signal cluster was found.\r\n");
        signalAnalysisState = SIGNAL_ANALYSIS_ERROR;
        return;
    }

    if (!unresolvedCluster) {
        printf("Model selection failed for already separated coarse peaks.\r\n");
        signalAnalysisState = SIGNAL_ANALYSIS_ERROR;
        return;
    }

    arm_status status = SignalZoomFFT_Start(zoomCenterFrequencyHz, coarseFftResult.meanCode, ADC_CAPTURE_SAMPLE_RATE_HZ);
    if (status != ARM_MATH_SUCCESS) {
        printf("ERROR: Zoom FFT acquisition initialization failed, status=%ld.\r\n", (long)status);
        signalAnalysisState = SIGNAL_ANALYSIS_ERROR;
        return;
    }

    printf("Decision: one unresolved coarse cluster; starting DDC and Zoom FFT around %lu Hz.\r\n", (unsigned long)RoundFloatToUInt32(zoomCenterFrequencyHz));
    ADC_Capture_SetStreamCallback(SignalZoomFFT_PushSamples);
    signalAnalysisState = SIGNAL_ANALYSIS_WAIT_ZOOM;
}

static bool FindUnresolvedFundamentalCluster(const SignalFftResult* result, float32_t* centerFrequencyHz) {
    if (result == NULL || centerFrequencyHz == NULL || result->significantPeakCount == 0U) {
        return false;
    }

    if (result->valid && result->signalA.valid && result->signalB.valid && fabsf(result->signalB.frequencyHz - result->signalA.frequencyHz) <= 4.0f * result->
        binResolutionHz) {
        *centerFrequencyHz = 0.5f * (result->signalA.frequencyHz + result->signalB.frequencyHz);
        return true;
    }

    const SignalFftPeak* fundamentalCluster = NULL;

    for (uint32_t index = 0U; index < result->significantPeakCount; index++) {
        const SignalFftPeak* peak = &result->significantPeaks[index];
        if (peak->frequencyHz >= (float32_t)SIGNAL_FFT_MIN_FREQUENCY_HZ && peak->frequencyHz <= (float32_t)SIGNAL_FFT_MAX_FREQUENCY_HZ) {
            fundamentalCluster = peak;
            break;
        }
    }

    if (fundamentalCluster == NULL) {
        return false;
    }

    /*
     * A close pair produces one coarse fundamental cluster. A triangle-wave
     * harmonic must not be mistaken for a second already-resolved
     * fundamental. Four coarse bins cover the frequency bias caused by two
     * unresolved fundamentals sharing the same Hann-window main lobe.
     */
    float32_t harmonicToleranceHz = 5.0f * result->binResolutionHz;

    for (uint32_t index = 0U; index < result->significantPeakCount; index++) {
        const SignalFftPeak* peak = &result->significantPeaks[index];
        if (peak == fundamentalCluster) {
            continue;
        }

        bool explainedByOddHarmonic = false;
        for (uint32_t harmonic = 3U; harmonic <= 15U; harmonic += 2U) {
            float32_t expectedFrequencyHz = fundamentalCluster->frequencyHz * (float32_t)harmonic;
            if (expectedFrequencyHz > (float32_t)SIGNAL_FFT_PEAK_SEARCH_MAX_FREQUENCY_HZ + harmonicToleranceHz) {
                break;
            }

            if (fabsf(peak->frequencyHz - expectedFrequencyHz) <= harmonicToleranceHz) {
                explainedByOddHarmonic = true;
                break;
            }
        }

        if (!explainedByOddHarmonic) {
            return false;
        }
    }

    *centerFrequencyHz = fundamentalCluster->frequencyHz;
    return true;
}

static void ProcessZoomFFT(void) {
    if (!SignalZoomFFT_IsCaptureComplete()) {
        return;
    }

    ADC_Capture_ClearStreamCallback();

    SignalZoomFftResult zoomResult;
    bool zoomSuccess = SignalZoomFFT_Analyze(&zoomResult);
    PrintZoomFFTResult(&zoomResult);

    if (!zoomSuccess || zoomResult.pattern != SIGNAL_ZOOM_FFT_PATTERN_TWO_SEPARATED) {
        if (coarseModelFallbackAvailable) {
            printf("Zoom FFT found no close pair; accepting the saved coarse model.\r\n");
            printf("Final signals: A=%lu Hz %s, B=%lu Hz %s\r\n", (unsigned long)RoundFloatToUInt32(coarseFftResult.signalA.frequencyHz),
                   SignalFFT_GetWaveformName(coarseFftResult.signalA.waveform), (unsigned long)RoundFloatToUInt32(coarseFftResult.signalB.frequencyHz),
                   SignalFFT_GetWaveformName(coarseFftResult.signalB.waveform));
            signalAnalysisState = SIGNAL_ANALYSIS_COMPLETE;
            return;
        }
        printf("Zoom FFT failed to resolve two valid fundamentals.\r\n");
        signalAnalysisState = SIGNAL_ANALYSIS_ERROR;
        return;
    }

    uint32_t frequencyAHz = RoundFloatToUInt32(zoomResult.signalA.frequencyHz);
    uint32_t frequencyBHz = RoundFloatToUInt32(zoomResult.signalB.frequencyHz);

    if (frequencyAHz < SIGNAL_FFT_MIN_FREQUENCY_HZ) {
        frequencyAHz = SIGNAL_FFT_MIN_FREQUENCY_HZ;
    }
    if (frequencyAHz > SIGNAL_FFT_MAX_FREQUENCY_HZ) {
        frequencyAHz = SIGNAL_FFT_MAX_FREQUENCY_HZ;
    }
    if (frequencyBHz < SIGNAL_FFT_MIN_FREQUENCY_HZ) {
        frequencyBHz = SIGNAL_FFT_MIN_FREQUENCY_HZ;
    }
    if (frequencyBHz > SIGNAL_FFT_MAX_FREQUENCY_HZ) {
        frequencyBHz = SIGNAL_FFT_MAX_FREQUENCY_HZ;
    }

    bool modelSuccess = SignalFFT_FitResolvedFundamentals(frequencyAHz, frequencyBHz, &coarseFftResult);

    printf("\r\n[HARMONIC_MODEL_RESULT]\r\n");
    if (coarseFftResult.signalA.valid && coarseFftResult.signalB.valid) {
        printf("A: frequency=%lu Hz, waveform=%s, amplitude=%lu codes peak, phase=%lu deg\r\n",
               (unsigned long)RoundFloatToUInt32(coarseFftResult.signalA.frequencyHz), SignalFFT_GetWaveformName(coarseFftResult.signalA.waveform),
               (unsigned long)RoundFloatToUInt32(coarseFftResult.signalA.amplitudePeakCode),
               (unsigned long)RoundFloatToUInt32(coarseFftResult.signalA.phaseDegrees));
        printf("B: frequency=%lu Hz, waveform=%s, amplitude=%lu codes peak, phase=%lu deg\r\n",
               (unsigned long)RoundFloatToUInt32(coarseFftResult.signalB.frequencyHz), SignalFFT_GetWaveformName(coarseFftResult.signalB.waveform),
               (unsigned long)RoundFloatToUInt32(coarseFftResult.signalB.amplitudePeakCode),
               (unsigned long)RoundFloatToUInt32(coarseFftResult.signalB.phaseDegrees));
        printf("Model residual: %lu permille (%s)\r\n", (unsigned long)RoundFloatToUInt32(coarseFftResult.residualRatio * 1000.0f),
               modelSuccess ? "accepted" : "rejected");
    }
    printf("[HARMONIC_MODEL_RESULT_END]\r\n");

    if (!modelSuccess) {
        printf("Harmonic model rejected the two Zoom FFT fundamentals.\r\n");
        signalAnalysisState = SIGNAL_ANALYSIS_ERROR;
        return;
    }

    printf("Decision: Zoom FFT resolved the close fundamentals and the harmonic model was accepted.\r\n");
    printf("Final signals: A=%lu Hz %s, B=%lu Hz %s\r\n", (unsigned long)frequencyAHz, SignalFFT_GetWaveformName(coarseFftResult.signalA.waveform),
           (unsigned long)frequencyBHz, SignalFFT_GetWaveformName(coarseFftResult.signalB.waveform));
    signalAnalysisState = SIGNAL_ANALYSIS_COMPLETE;
}

static void PrintCoarseFFTResult(const SignalFftResult* result) {
    printf("\r\n[FFT_RESULT]\r\n");
    printf("Mean: %lu codes\r\n", (unsigned long)RoundFloatToUInt32(result->meanCode));
    printf("Resolution: %lu Hz/bin\r\n", (unsigned long)RoundFloatToUInt32(result->binResolutionHz));
    printf("Significant peaks: %lu, fundamental candidates: %lu\r\n", (unsigned long)result->significantPeakCount,
           (unsigned long)result->fundamentalCandidateCount);

    for (uint32_t index = 0U; index < result->significantPeakCount; index++) {
        const SignalFftPeak* peak = &result->significantPeaks[index];
        printf("Peak %lu: raw=%lu Hz, bin=%lu, amplitude=%lu codes Vpp\r\n", (unsigned long)(index + 1U), (unsigned long)RoundFloatToUInt32(peak->frequencyHz),
               (unsigned long)peak->peakBin, (unsigned long)RoundFloatToUInt32(peak->amplitudePeakToPeakCode));
    }

    if (result->signalA.valid && result->signalB.valid) {
        printf("A: frequency=%lu Hz, waveform=%s, amplitude=%lu codes peak, phase=%lu deg\r\n", (unsigned long)RoundFloatToUInt32(result->signalA.frequencyHz),
               SignalFFT_GetWaveformName(result->signalA.waveform), (unsigned long)RoundFloatToUInt32(result->signalA.amplitudePeakCode),
               (unsigned long)RoundFloatToUInt32(result->signalA.phaseDegrees));
        printf("B: frequency=%lu Hz, waveform=%s, amplitude=%lu codes peak, phase=%lu deg\r\n", (unsigned long)RoundFloatToUInt32(result->signalB.frequencyHz),
               SignalFFT_GetWaveformName(result->signalB.waveform), (unsigned long)RoundFloatToUInt32(result->signalB.amplitudePeakCode),
               (unsigned long)RoundFloatToUInt32(result->signalB.phaseDegrees));
        printf("Model residual: %lu permille (%s)\r\n", (unsigned long)RoundFloatToUInt32(result->residualRatio * 1000.0f),
               result->valid ? "accepted" : "rejected");
    }

    printf("FFT time: %lu us, total analysis: %lu us\r\n", (unsigned long)RoundFloatToUInt32(result->fftTimeUs),
           (unsigned long)RoundFloatToUInt32(result->totalTimeUs));
    printf("[FFT_RESULT_END]\r\n");
}

static void PrintZoomFFTResult(const SignalZoomFftResult* result) {
    printf("\r\n[ZOOM_FFT_RESULT]\r\n");
    printf("Center: %lu Hz\r\n", (unsigned long)RoundFloatToUInt32(result->centerFrequencyHz));
    printf("Rates: input=%lu samples/s, baseband=%lu samples/s\r\n", (unsigned long)result->inputSampleRateHz, (unsigned long)result->outputSampleRateHz);
    printf("Resolution: %lu Hz/bin\r\n", (unsigned long)RoundFloatToUInt32(result->binResolutionHz));
    printf("Capture: %lu raw samples, %lu us\r\n", (unsigned long)result->rawSampleCount, (unsigned long)RoundFloatToUInt32(result->captureTimeUs));

    if (!result->valid) {
        printf("Pattern: none\r\n");
    }
    else if (result->pattern == SIGNAL_ZOOM_FFT_PATTERN_TWO_SEPARATED) {
        printf("Pattern: two separated peaks\r\n");
        printf("A: raw=%lu Hz, baseband=%ld Hz, bin=%ld, amplitude=%lu codes Vpp\r\n", (unsigned long)RoundFloatToUInt32(result->signalA.frequencyHz),
               (long)RoundFloatToInt32(result->signalA.basebandFrequencyHz), (long)result->signalA.peakBin,
               (unsigned long)RoundFloatToUInt32(result->signalA.amplitudePeakToPeakCode));
        printf("B: raw=%lu Hz, baseband=%ld Hz, bin=%ld, amplitude=%lu codes Vpp\r\n", (unsigned long)RoundFloatToUInt32(result->signalB.frequencyHz),
               (long)RoundFloatToInt32(result->signalB.basebandFrequencyHz), (long)result->signalB.peakBin,
               (unsigned long)RoundFloatToUInt32(result->signalB.amplitudePeakToPeakCode));
    }
    else {
        printf("Pattern: single unresolved cluster\r\n");
        printf("Cluster: raw=%lu Hz, baseband=%ld Hz, bin=%ld, amplitude=%lu codes Vpp\r\n",
               (unsigned long)RoundFloatToUInt32(result->dominantPeak.frequencyHz), (long)RoundFloatToInt32(result->dominantPeak.basebandFrequencyHz),
               (long)result->dominantPeak.peakBin, (unsigned long)RoundFloatToUInt32(result->dominantPeak.amplitudePeakToPeakCode));
    }

    printf("DDC CPU time: %lu us\r\n", (unsigned long)RoundFloatToUInt32(result->ddcTimeUs));
    printf("DDC callback max: %lu us, overruns=%lu\r\n", (unsigned long)RoundFloatToUInt32(result->maximumDdcCallbackTimeUs),
           (unsigned long)result->ddcOverrunCount);
    printf("Zoom FFT time: %lu us, analysis: %lu us\r\n", (unsigned long)RoundFloatToUInt32(result->fftTimeUs),
           (unsigned long)RoundFloatToUInt32(result->analysisTimeUs));
    printf("[ZOOM_FFT_RESULT_END]\r\n");
}

static uint32_t RoundFloatToUInt32(float32_t value) {
    if (value <= 0.0f) {
        return 0U;
    }

    return (uint32_t)(value + 0.5f);
}

static int32_t RoundFloatToInt32(float32_t value) {
    if (value >= 0.0f) {
        return (int32_t)(value + 0.5f);
    }

    return (int32_t)(value - 0.5f);
}

/**
  * @brief  Redirects printf output to USART1.
  * @param  ch Character to transmit.
  * @retval Transmitted character, or EOF on failure.
  */
int __io_putchar(int ch) {
    uint8_t data = (uint8_t)ch;

    if (HAL_UART_Transmit(&huart1, &data, 1U, HAL_MAX_DELAY) != HAL_OK) {
        return EOF;
    }

    return ch;
}

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void) {
    MPU_Region_InitTypeDef MPU_InitStruct = {0};

    /* Disables the MPU */
    HAL_MPU_Disable();

    /** Initializes and configures the Region and the memory to be protected
    */
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress = 0x0;
    MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
    MPU_InitStruct.SubRegionDisable = 0x87;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    /* Enables the MPU */
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    ADC_Capture_ClearStreamCallback();
    __disable_irq();

    while (1) {}
    /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
