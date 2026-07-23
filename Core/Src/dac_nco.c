#include "dac_nco.h"

#include "dac.h"
#include "square_wave_output.h"
#include "tim.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    volatile uint32_t phase;
    volatile uint32_t phaseStep;
    SignalWaveformType waveform;
    volatile uint32_t halfTransferCount;
    volatile uint32_t fullTransferCount;
    volatile uint32_t maximumFillCycleCount;
    volatile uint32_t errorCount;
    volatile uint32_t underrunCount;
    volatile bool active;
} DacNcoState;

#define DAC_NCO_BUFFER_SAMPLE_COUNT 2048U
#define DAC_NCO_HALF_BUFFER_SAMPLE_COUNT (DAC_NCO_BUFFER_SAMPLE_COUNT / 2U)
#define DAC_NCO_WAVEFORM_TABLE_BITS 10U
#define DAC_NCO_WAVEFORM_TABLE_SAMPLE_COUNT (1U << DAC_NCO_WAVEFORM_TABLE_BITS)
#define DAC_NCO_MID_SCALE_CODE 2048
#define DAC_NCO_PEAK_AMPLITUDE_CODE 680
#define DAC_NCO_PHASE_SCALE 4294967296.0
#define DAC_NCO_OUTPUT_PHASE_REFERENCE_DEGREES 90U
#define DAC_NCO_STATUS_REPORT_INTERVAL_MS 1000U
#define DAC_NCO_TRACKING_MARGIN_HZ 20.0f

static uint16_t channel1Samples[DAC_NCO_BUFFER_SAMPLE_COUNT] __attribute__((aligned(32)));
static uint16_t channel2Samples[DAC_NCO_BUFFER_SAMPLE_COUNT] __attribute__((aligned(32)));
static uint16_t sineTable[DAC_NCO_WAVEFORM_TABLE_SAMPLE_COUNT];
static uint16_t triangleTable[DAC_NCO_WAVEFORM_TABLE_SAMPLE_COUNT];
static uint16_t squareTable[DAC_NCO_WAVEFORM_TABLE_SAMPLE_COUNT];
static DacNcoState channel1Nco;
static DacNcoState channel2Nco;
static bool outputStarted;
static bool appliedPhaseDifferenceValid;
static uint32_t lastStatusTick;
static uint32_t requestedPhaseDifferenceDegrees;
static uint32_t appliedPhaseDifferenceDegrees;

static void InitializeWaveformTables(void);
static void InitializeNco(DacNcoState* nco, float32_t frequencyHz, SignalWaveformType waveform);
static void FillSamples(DacNcoState* nco, uint16_t* samples, uint32_t sampleCount);
static void FillHalf(DacNcoState* nco, uint16_t* samples, uint32_t offset);
static void CleanBufferCache(uint16_t* samples, uint32_t sampleCount);
static uint32_t FrequencyToPhaseStep(float32_t frequencyHz);
static uint32_t DegreesToPhase(uint32_t phaseDegrees);
static uint32_t OutputPhaseDifferenceToPhase(uint32_t phaseDegrees);
static uint64_t PhaseStepToMilliHz(uint32_t phaseStep);
static void ResetOutputTriggerState(void);
static void ClearDmaPendingFlags(DMA_HandleTypeDef* hdma);
static bool RestartOutputsWithPhaseOffset(void);

bool DacNco_Start(const SignalFftSeparatedSignal* signalA, const SignalFftSeparatedSignal* signalB) {
    if (signalA == NULL || signalB == NULL) {
        printf("ERROR: NCO start received a null signal.\r\n");
        return false;
    }

    InitializeWaveformTables();
    InitializeNco(&channel1Nco, signalA->frequencyHz, signalA->waveform);
    InitializeNco(&channel2Nco, signalB->frequencyHz, signalB->waveform);
    channel2Nco.phase = OutputPhaseDifferenceToPhase(requestedPhaseDifferenceDegrees);

    uint64_t requestedAHzMilli = (uint64_t)(signalA->frequencyHz * 1000.0f + 0.5f);
    uint64_t requestedBHzMilli = (uint64_t)(signalB->frequencyHz * 1000.0f + 0.5f);
    uint64_t actualAHzMilli = PhaseStepToMilliHz(channel1Nco.phaseStep);
    uint64_t actualBHzMilli = PhaseStepToMilliHz(channel2Nco.phaseStep);

    printf("Preparing streaming NCO outputs.\r\n");
    printf("NCO A: FFT=%lu.%03lu Hz, FTW=0x%08lX, actual=%lu.%03lu Hz, waveform=%s.\r\n", (unsigned long)(requestedAHzMilli / 1000ULL),
           (unsigned long)(requestedAHzMilli % 1000ULL), (unsigned long)channel1Nco.phaseStep, (unsigned long)(actualAHzMilli / 1000ULL),
           (unsigned long)(actualAHzMilli % 1000ULL), SignalFFT_GetWaveformName(signalA->waveform));
    printf("NCO B: FFT=%lu.%03lu Hz, FTW=0x%08lX, actual=%lu.%03lu Hz, waveform=%s.\r\n", (unsigned long)(requestedBHzMilli / 1000ULL),
           (unsigned long)(requestedBHzMilli % 1000ULL), (unsigned long)channel2Nco.phaseStep, (unsigned long)(actualBHzMilli / 1000ULL),
           (unsigned long)(actualBHzMilli % 1000ULL), SignalFFT_GetWaveformName(signalB->waveform));

    FillSamples(&channel1Nco, channel1Samples, DAC_NCO_BUFFER_SAMPLE_COUNT);
    FillSamples(&channel2Nco, channel2Samples, DAC_NCO_BUFFER_SAMPLE_COUNT);
    CleanBufferCache(channel1Samples, DAC_NCO_BUFFER_SAMPLE_COUNT);
    CleanBufferCache(channel2Samples, DAC_NCO_BUFFER_SAMPLE_COUNT);

    if (HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)channel1Samples, DAC_NCO_BUFFER_SAMPLE_COUNT, DAC_ALIGN_12B_R) != HAL_OK) {
        printf("ERROR: DAC channel 1 DMA start failed, DAC error=0x%08lX.\r\n", (unsigned long)HAL_DAC_GetError(&hdac1));
        return false;
    }

    if (HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_2, (uint32_t*)channel2Samples, DAC_NCO_BUFFER_SAMPLE_COUNT, DAC_ALIGN_12B_R) != HAL_OK) {
        printf("ERROR: DAC channel 2 DMA start failed, DAC error=0x%08lX.\r\n", (unsigned long)HAL_DAC_GetError(&hdac1));
        (void)HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
        return false;
    }

    channel1Nco.active = true;
    channel2Nco.active = true;

    if (HAL_TIM_Base_Start(&htim6) != HAL_OK) {
        printf("ERROR: TIM6 start failed, state=%u.\r\n", (unsigned int)HAL_TIM_Base_GetState(&htim6));
        channel1Nco.active = false;
        channel2Nco.active = false;
        (void)HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_2);
        (void)HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
        return false;
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    outputStarted = true;
    appliedPhaseDifferenceDegrees = requestedPhaseDifferenceDegrees;
    appliedPhaseDifferenceValid = true;
    SquareWaveOutput_Start(signalA, signalB);
    lastStatusTick = HAL_GetTick();
    printf("DAC streaming started: %lu samples/s, %lu samples/channel, half-buffer refill=%lu samples.\r\n", (unsigned long)DAC_NCO_SAMPLE_RATE_HZ,
           (unsigned long)DAC_NCO_BUFFER_SAMPLE_COUNT, (unsigned long)DAC_NCO_HALF_BUFFER_SAMPLE_COUNT);
    return true;
}

void DacNco_SetChannelAFrequency(float32_t frequencyHz) {
    if (!outputStarted) {
        return;
    }

    channel1Nco.phaseStep = FrequencyToPhaseStep(frequencyHz);
    SquareWaveOutput_SetChannelAFrequency(frequencyHz);
}

void DacNco_SetChannelBFrequency(float32_t frequencyHz) {
    if (!outputStarted) {
        return;
    }

    channel2Nco.phaseStep = FrequencyToPhaseStep(frequencyHz);
    SquareWaveOutput_SetChannelBFrequency(frequencyHz);
}

bool DacNco_SetPhaseDifference(uint32_t phaseDegrees) {
    if (phaseDegrees > 180U || phaseDegrees % 5U != 0U) {
        return false;
    }

    if (outputStarted && appliedPhaseDifferenceValid && phaseDegrees == appliedPhaseDifferenceDegrees) {
        requestedPhaseDifferenceDegrees = phaseDegrees;
        return true;
    }

    requestedPhaseDifferenceDegrees = phaseDegrees;

    if (!outputStarted) {
        return true;
    }

    return RestartOutputsWithPhaseOffset();
}

void DacNco_ProcessDiagnostics(void) {
    if (!outputStarted) {
        return;
    }

    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - lastStatusTick) < DAC_NCO_STATUS_REPORT_INTERVAL_MS) {
        return;
    }

    uint32_t channel1FillUs = SystemCoreClock == 0U ? 0U : (uint32_t)(((uint64_t)channel1Nco.maximumFillCycleCount * 1000000ULL) / SystemCoreClock);
    uint32_t channel2FillUs = SystemCoreClock == 0U ? 0U : (uint32_t)(((uint64_t)channel2Nco.maximumFillCycleCount * 1000000ULL) / SystemCoreClock);

    printf("[NCO_STATUS] A: HT=%lu TC=%lu max=%lu us err=%lu underrun=%lu; B: HT=%lu TC=%lu max=%lu us err=%lu underrun=%lu.\r\n",
           (unsigned long)channel1Nco.halfTransferCount, (unsigned long)channel1Nco.fullTransferCount, (unsigned long)channel1FillUs,
           (unsigned long)channel1Nco.errorCount, (unsigned long)channel1Nco.underrunCount, (unsigned long)channel2Nco.halfTransferCount,
           (unsigned long)channel2Nco.fullTransferCount, (unsigned long)channel2FillUs, (unsigned long)channel2Nco.errorCount,
           (unsigned long)channel2Nco.underrunCount);
    SquareWaveOutput_PrintDiagnostics();
    lastStatusTick = now;
}

static void InitializeWaveformTables(void) {
    const float32_t twoPi = 6.28318530717958647692f;

    for (uint32_t index = 0U; index < DAC_NCO_WAVEFORM_TABLE_SAMPLE_COUNT; index++) {
        float32_t phase = twoPi * (float32_t)index / (float32_t)DAC_NCO_WAVEFORM_TABLE_SAMPLE_COUNT;
        int32_t sineQ15 = (int32_t)(32767.0f * sinf(phase));
        int32_t sineScaled = (DAC_NCO_PEAK_AMPLITUDE_CODE * sineQ15 + 16384) >> 15U;
        sineTable[index] = (uint16_t)(DAC_NCO_MID_SCALE_CODE + sineScaled);

        uint32_t position = index << (16U - DAC_NCO_WAVEFORM_TABLE_BITS);
        int32_t triangleQ15;

        if (position < 32768U) {
            triangleQ15 = -32767 + (int32_t)(position << 1U);
        }
        else {
            triangleQ15 = 32767 - (int32_t)((position - 32768U) << 1U);
        }

        int32_t triangleScaled = (DAC_NCO_PEAK_AMPLITUDE_CODE * triangleQ15 + 16384) >> 15U;
        triangleTable[index] = (uint16_t)(DAC_NCO_MID_SCALE_CODE + triangleScaled);

        int32_t squareScaled = index < DAC_NCO_WAVEFORM_TABLE_SAMPLE_COUNT / 2U ? DAC_NCO_PEAK_AMPLITUDE_CODE : -DAC_NCO_PEAK_AMPLITUDE_CODE;
        squareTable[index] = (uint16_t)(DAC_NCO_MID_SCALE_CODE + squareScaled);
    }
}

static void InitializeNco(DacNcoState* nco, float32_t frequencyHz, SignalWaveformType waveform) {
    memset(nco, 0, sizeof(*nco));
    nco->phaseStep = FrequencyToPhaseStep(frequencyHz);
    nco->waveform = waveform;
}

static void FillSamples(DacNcoState* nco, uint16_t* samples, uint32_t sampleCount) {
    uint32_t phase = nco->phase;
    const uint32_t phaseStep = nco->phaseStep;
    const uint16_t* waveformTable = sineTable;

    if (nco->waveform == SIGNAL_WAVEFORM_TRIANGLE) {
        waveformTable = triangleTable;
    }
    else if (nco->waveform == SIGNAL_WAVEFORM_SQUARE) {
        waveformTable = squareTable;
    }

    for (uint32_t index = 0U; index < sampleCount; index++) {
        uint32_t tableIndex = phase >> (32U - DAC_NCO_WAVEFORM_TABLE_BITS);
        samples[index] = waveformTable[tableIndex];
        phase += phaseStep;
    }

    nco->phase = phase;
}

static void FillHalf(DacNcoState* nco, uint16_t* samples, uint32_t offset) {
    uint32_t startCycleCount = DWT->CYCCNT;

    FillSamples(nco, &samples[offset], DAC_NCO_HALF_BUFFER_SAMPLE_COUNT);
    CleanBufferCache(&samples[offset], DAC_NCO_HALF_BUFFER_SAMPLE_COUNT);

    uint32_t elapsedCycleCount = DWT->CYCCNT - startCycleCount;
    if (elapsedCycleCount > nco->maximumFillCycleCount) {
        nco->maximumFillCycleCount = elapsedCycleCount;
    }
}

static void CleanBufferCache(uint16_t* samples, uint32_t sampleCount) {
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
        SCB_CleanDCache_by_Addr((uint32_t*)samples, (int32_t)(sampleCount * sizeof(uint16_t)));
    }
}

static uint32_t FrequencyToPhaseStep(float32_t frequencyHz) {
    const float32_t minimumTrackingFrequencyHz = (float32_t)SIGNAL_FFT_MIN_FREQUENCY_HZ - DAC_NCO_TRACKING_MARGIN_HZ;
    const float32_t maximumTrackingFrequencyHz = (float32_t)SIGNAL_FFT_MAX_FREQUENCY_HZ + DAC_NCO_TRACKING_MARGIN_HZ;

    if (frequencyHz < minimumTrackingFrequencyHz) {
        frequencyHz = minimumTrackingFrequencyHz;
    }
    if (frequencyHz > maximumTrackingFrequencyHz) {
        frequencyHz = maximumTrackingFrequencyHz;
    }

    double phaseStep = (double)frequencyHz * DAC_NCO_PHASE_SCALE / (double)DAC_NCO_SAMPLE_RATE_HZ;
    return (uint32_t)(phaseStep + 0.5);
}

static uint32_t DegreesToPhase(uint32_t phaseDegrees) {
    uint64_t scaledPhase = ((uint64_t)phaseDegrees << 32U) + 180ULL;
    return (uint32_t)(scaledPhase / 360ULL);
}

static uint32_t OutputPhaseDifferenceToPhase(uint32_t phaseDegrees) {
    return DegreesToPhase(phaseDegrees + DAC_NCO_OUTPUT_PHASE_REFERENCE_DEGREES);
}

static uint64_t PhaseStepToMilliHz(uint32_t phaseStep) {
    uint64_t numerator = (uint64_t)phaseStep * DAC_NCO_SAMPLE_RATE_HZ * 1000ULL;
    return (numerator + (1ULL << 31U)) >> 32U;
}

static void ResetOutputTriggerState(void) {
    __HAL_TIM_DISABLE(&htim6);
    __HAL_TIM_SET_COUNTER(&htim6, 0U);
    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);

    __HAL_DAC_CLEAR_FLAG(&hdac1, DAC_FLAG_DMAUDR1 | DAC_FLAG_DMAUDR2);
    hdac1.ErrorCode &= ~(HAL_DAC_ERROR_DMAUNDERRUNCH1 | HAL_DAC_ERROR_DMAUNDERRUNCH2 | HAL_DAC_ERROR_DMA);

    ClearDmaPendingFlags(hdac1.DMA_Handle1);
    ClearDmaPendingFlags(hdac1.DMA_Handle2);
    NVIC_ClearPendingIRQ(DMA1_Stream0_IRQn);
    NVIC_ClearPendingIRQ(DMA1_Stream1_IRQn);
    NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
}

static void ClearDmaPendingFlags(DMA_HandleTypeDef* hdma) {
    uint32_t flags = __HAL_DMA_GET_TC_FLAG_INDEX(hdma) | __HAL_DMA_GET_HT_FLAG_INDEX(hdma) | __HAL_DMA_GET_TE_FLAG_INDEX(hdma) |
        __HAL_DMA_GET_DME_FLAG_INDEX(hdma) | __HAL_DMA_GET_FE_FLAG_INDEX(hdma);
    __HAL_DMA_CLEAR_FLAG(hdma, flags);
}

static bool RestartOutputsWithPhaseOffset(void) {
    uint32_t interruptState = __get_PRIMASK();
    __disable_irq();

    channel1Nco.active = false;
    channel2Nco.active = false;

    HAL_StatusTypeDef timerStopStatus = HAL_TIM_Base_Stop(&htim6);
    HAL_StatusTypeDef channel1StopStatus = HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
    HAL_StatusTypeDef channel2StopStatus = HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_2);
    ResetOutputTriggerState();

    if (timerStopStatus != HAL_OK || channel1StopStatus != HAL_OK || channel2StopStatus != HAL_OK) {
        outputStarted = false;
        appliedPhaseDifferenceValid = false;
        if (interruptState == 0U) {
            __enable_irq();
        }
        printf("ERROR: output stop failed while setting phase.\r\n");
        return false;
    }

    channel1Nco.phase = 0U;
    channel2Nco.phase = OutputPhaseDifferenceToPhase(requestedPhaseDifferenceDegrees);
    FillSamples(&channel1Nco, channel1Samples, DAC_NCO_BUFFER_SAMPLE_COUNT);
    FillSamples(&channel2Nco, channel2Samples, DAC_NCO_BUFFER_SAMPLE_COUNT);
    CleanBufferCache(channel1Samples, DAC_NCO_BUFFER_SAMPLE_COUNT);
    CleanBufferCache(channel2Samples, DAC_NCO_BUFFER_SAMPLE_COUNT);

    if (HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)channel1Samples, DAC_NCO_BUFFER_SAMPLE_COUNT, DAC_ALIGN_12B_R) != HAL_OK) {
        outputStarted = false;
        appliedPhaseDifferenceValid = false;
        if (interruptState == 0U) {
            __enable_irq();
        }
        printf("ERROR: DAC channel 1 restart failed while setting phase.\r\n");
        return false;
    }

    if (HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_2, (uint32_t*)channel2Samples, DAC_NCO_BUFFER_SAMPLE_COUNT, DAC_ALIGN_12B_R) != HAL_OK) {
        (void)HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
        outputStarted = false;
        appliedPhaseDifferenceValid = false;
        if (interruptState == 0U) {
            __enable_irq();
        }
        printf("ERROR: DAC channel 2 restart failed while setting phase.\r\n");
        return false;
    }

    ResetOutputTriggerState();
    channel1Nco.active = true;
    channel2Nco.active = true;

    if (HAL_TIM_Base_Start(&htim6) != HAL_OK) {
        channel1Nco.active = false;
        channel2Nco.active = false;
        (void)HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_2);
        (void)HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
        outputStarted = false;
        appliedPhaseDifferenceValid = false;
        if (interruptState == 0U) {
            __enable_irq();
        }
        printf("ERROR: TIM6 restart failed while setting phase.\r\n");
        return false;
    }

    appliedPhaseDifferenceDegrees = requestedPhaseDifferenceDegrees;
    appliedPhaseDifferenceValid = true;
    if (interruptState == 0U) {
        __enable_irq();
    }

    printf("Output phase synchronized: A=0 deg, B=%lu deg.\r\n", (unsigned long)requestedPhaseDifferenceDegrees);
    return true;
}

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef* hdac) {
    if (hdac->Instance == DAC1 && channel1Nco.active) {
        channel1Nco.halfTransferCount++;
        FillHalf(&channel1Nco, channel1Samples, 0U);
    }
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac) {
    if (hdac->Instance == DAC1 && channel1Nco.active) {
        channel1Nco.fullTransferCount++;
        FillHalf(&channel1Nco, channel1Samples, DAC_NCO_HALF_BUFFER_SAMPLE_COUNT);
    }
}

void HAL_DACEx_ConvHalfCpltCallbackCh2(DAC_HandleTypeDef* hdac) {
    if (hdac->Instance == DAC1 && channel2Nco.active) {
        channel2Nco.halfTransferCount++;
        FillHalf(&channel2Nco, channel2Samples, 0U);
    }
}

void HAL_DACEx_ConvCpltCallbackCh2(DAC_HandleTypeDef* hdac) {
    if (hdac->Instance == DAC1 && channel2Nco.active) {
        channel2Nco.fullTransferCount++;
        FillHalf(&channel2Nco, channel2Samples, DAC_NCO_HALF_BUFFER_SAMPLE_COUNT);
    }
}

void HAL_DAC_ErrorCallbackCh1(DAC_HandleTypeDef* hdac) {
    if (hdac->Instance == DAC1) {
        channel1Nco.errorCount++;
        channel1Nco.active = false;
    }
}

void HAL_DACEx_ErrorCallbackCh2(DAC_HandleTypeDef* hdac) {
    if (hdac->Instance == DAC1) {
        channel2Nco.errorCount++;
        channel2Nco.active = false;
    }
}

void HAL_DAC_DMAUnderrunCallbackCh1(DAC_HandleTypeDef* hdac) {
    if (hdac->Instance == DAC1) {
        channel1Nco.underrunCount++;
    }
}

void HAL_DACEx_DMAUnderrunCallbackCh2(DAC_HandleTypeDef* hdac) {
    if (hdac->Instance == DAC1) {
        channel2Nco.underrunCount++;
    }
}
