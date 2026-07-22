#include "iq_phase_monitor.h"

#include "adc_capture.h"
#include "dac_nco.h"
#include "signal_fft.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define IQ_PLL_CHANNEL_COUNT 2U
#define IQ_PLL_CHANNEL_A 0U
#define IQ_PLL_CHANNEL_B 1U
#define IQ_SINE_TABLE_BITS 10U
#define IQ_SINE_TABLE_SAMPLE_COUNT (1U << IQ_SINE_TABLE_BITS)
#define IQ_DECIMATION_FACTOR 8U
#define IQ_RAW_BLOCK_SAMPLE_COUNT (ADC_CAPTURE_BUFFER_SIZE / 2U)
#define IQ_DECIMATED_BLOCK_SAMPLE_COUNT (IQ_RAW_BLOCK_SAMPLE_COUNT / IQ_DECIMATION_FACTOR)
#define IQ_BLOCK_QUEUE_DEPTH 32U
#define IQ_BLOCKS_PER_PROCESS_CALL 4U
#define IQ_PHASE_RAW_WINDOW_SAMPLE_COUNT 65536U
#define IQ_PHASE_WINDOW_SAMPLE_COUNT (IQ_PHASE_RAW_WINDOW_SAMPLE_COUNT / IQ_DECIMATION_FACTOR)
#define IQ_PHASE_REPORT_INTERVAL_MS 1000U
#define IQ_PHASE_SCALE 4294967296.0
#define IQ_HANN_SCALE 32767.0f
#define IQ_PLL_FREQUENCY_TRACKING_GAIN 0.25f
#define IQ_PLL_CORRECTION_LIMIT_HZ 20.0f
#define IQ_PLL_CORRECTION_STEP_LIMIT_HZ 0.50f
#define IQ_PLL_MIN_VECTOR_MAGNITUDE_CODE 8.0f
#define IQ_PLL_LOCK_FILTER_GAIN 0.0625f
#define IQ_PLL_LOCK_ENTER_RESIDUAL_LIMIT_HZ 0.10f
#define IQ_PLL_LOCK_EXIT_RESIDUAL_LIMIT_HZ 0.50f
#define IQ_PLL_LOCK_ENTER_WINDOW_COUNT 25U
#define IQ_PLL_LOCK_EXIT_WINDOW_COUNT 10U
#define IQ_PLL_WINDOW_SECONDS ((float)IQ_PHASE_RAW_WINDOW_SAMPLE_COUNT / (float)ADC_CAPTURE_SAMPLE_RATE_HZ)

typedef struct {
    uint16_t samples[IQ_DECIMATED_BLOCK_SAMPLE_COUNT];
    uint32_t startPhase[IQ_PLL_CHANNEL_COUNT];
    uint32_t phaseStep[IQ_PLL_CHANNEL_COUNT];
    uint32_t sampleCount;
    volatile bool ready;
} IqSampleBlock;

typedef struct {
    volatile uint32_t referencePhase;
    volatile uint32_t referencePhaseStep;
    int64_t accumulatorI;
    int64_t accumulatorQ;
    int64_t completedI;
    int64_t completedQ;
    float initialFrequencyHz;
    float trackingFrequencyHz;
    float frequencyCorrectionHz;
    float latestPhaseDriftDegrees;
    float latestResidualFrequencyHz;
    float latestVectorMagnitudeCode;
    float previousMeasuredPhaseDegrees;
    float filteredResidualFrequencyHz;
    bool previousPhaseCaptured;
    bool latestVectorValid;
    bool residualFilterInitialized;
    bool pllLocked;
    uint32_t rejectedWindowCount;
    uint32_t lockGoodWindowCount;
    uint32_t lockBadWindowCount;
} IqPllState;

static int16_t sineTable[IQ_SINE_TABLE_SAMPLE_COUNT];
static uint16_t hannWindow[IQ_PHASE_WINDOW_SAMPLE_COUNT];
static IqSampleBlock sampleBlocks[IQ_BLOCK_QUEUE_DEPTH];
static IqPllState pllChannels[IQ_PLL_CHANNEL_COUNT];
static int32_t dcMeanCode;
static volatile uint32_t writeBlockIndex;
static uint32_t readBlockIndex;
static volatile uint32_t receivedBlockCount;
static volatile uint32_t droppedBlockCount;
static uint32_t processedBlockCount;
static uint32_t maximumProcessCycleCount;
static uint32_t accumulatorSampleCount;
static uint32_t completedWindowCount;
static uint32_t overwrittenWindowCount;
static bool resultReady;
static bool active;
static uint32_t lastReportTick;

static void InitializeSineTable(void);
static void InitializeHannWindow(void);
static void InitializePllState(IqPllState* pll, float frequencyHz);
static void ResetPllRecoveryState(IqPllState* pll);
static uint32_t FrequencyToPhaseStep(float frequencyHz);
static void QueueDecimatedSamples(const uint16_t* samples, uint32_t sampleCount);
static void ProcessQueuedBlocks(void);
static void ProcessBlock(const IqSampleBlock* block);
static void UpdatePll(uint32_t channelIndex, int64_t resultI, int64_t resultQ);
static void UpdateLockState(IqPllState* pll);
static void SetDacFrequency(uint32_t channelIndex, float frequencyHz);
static void PrintPllDiagnostics(uint32_t channelIndex);
static uint32_t CountQueuedBlocks(void);
static uint32_t CycleCountToMicroseconds(uint32_t cycleCount);
static float ClampFloat(float value, float minimum, float maximum);
static int32_t RoundFloatToInt32(float value);

void IqPhaseMonitor_Start(float frequencyAHz, float frequencyBHz, float meanCode) {
    InitializeSineTable();
    InitializeHannWindow();
    InitializePllState(&pllChannels[IQ_PLL_CHANNEL_A], frequencyAHz);
    InitializePllState(&pllChannels[IQ_PLL_CHANNEL_B], frequencyBHz);
    dcMeanCode = (int32_t)(meanCode + 0.5f);
    writeBlockIndex = 0U;
    readBlockIndex = 0U;
    receivedBlockCount = 0U;
    droppedBlockCount = 0U;
    processedBlockCount = 0U;
    maximumProcessCycleCount = 0U;
    accumulatorSampleCount = 0U;
    completedWindowCount = 0U;
    overwrittenWindowCount = 0U;
    resultReady = false;
    active = false;
    lastReportTick = HAL_GetTick();

    for (uint32_t index = 0U; index < IQ_BLOCK_QUEUE_DEPTH; index++) {
        sampleBlocks[index].ready = false;
    }

    for (uint32_t channelIndex = 0U; channelIndex < IQ_PLL_CHANNEL_COUNT; channelIndex++) {
        const IqPllState* pll = &pllChannels[channelIndex];
        uint64_t frequencyHzMilli = (uint64_t)(pll->initialFrequencyHz * 1000.0f + 0.5f);

        printf("IQ PLL %c: initial=%lu.%03lu Hz, FTW=0x%08lX, decimation=%lu, " "window=%lu raw samples.\r\n", channelIndex == IQ_PLL_CHANNEL_A ? 'A' : 'B',
               (unsigned long)(frequencyHzMilli / 1000ULL), (unsigned long)(frequencyHzMilli % 1000ULL), (unsigned long)pll->referencePhaseStep,
               (unsigned long)IQ_DECIMATION_FACTOR, (unsigned long)IQ_PHASE_RAW_WINDOW_SAMPLE_COUNT);
    }

    printf("PLL A/B: Hann-windowed phase tracking, gain=0.25, correction=+/-20 Hz.\r\n");
    printf("IQ acquisition armed: ADC callback queues shared decimated blocks; dual I/Q math runs in main loop.\r\n");
    active = true;
    ADC_Capture_SetStreamCallback(QueueDecimatedSamples);
}

void IqPhaseMonitor_Process(void) {
    if (!active) {
        return;
    }

    ProcessQueuedBlocks();

    uint32_t now = HAL_GetTick();
    if (!resultReady || (uint32_t)(now - lastReportTick) < IQ_PHASE_REPORT_INTERVAL_MS) {
        return;
    }

    resultReady = false;
    PrintPllDiagnostics(IQ_PLL_CHANNEL_A);
    PrintPllDiagnostics(IQ_PLL_CHANNEL_B);
    lastReportTick = now;
}

static void InitializeSineTable(void) {
    const float twoPi = 6.28318530717958647692f;

    for (uint32_t index = 0U; index < IQ_SINE_TABLE_SAMPLE_COUNT; index++) {
        float phase = twoPi * (float)index / (float)IQ_SINE_TABLE_SAMPLE_COUNT;
        sineTable[index] = (int16_t)(32767.0f * sinf(phase));
    }
}

static void InitializeHannWindow(void) {
    const float twoPi = 6.28318530717958647692f;

    for (uint32_t index = 0U; index < IQ_PHASE_WINDOW_SAMPLE_COUNT; index++) {
        float phase = twoPi * (float)index / (float)(IQ_PHASE_WINDOW_SAMPLE_COUNT - 1U);
        float weight = 0.5f - 0.5f * cosf(phase);
        hannWindow[index] = (uint16_t)(IQ_HANN_SCALE * weight + 0.5f);
    }
}

static void InitializePllState(IqPllState* pll, float frequencyHz) {
    pll->referencePhase = 0U;
    pll->referencePhaseStep = FrequencyToPhaseStep(frequencyHz);
    pll->accumulatorI = 0;
    pll->accumulatorQ = 0;
    pll->completedI = 0;
    pll->completedQ = 0;
    pll->initialFrequencyHz = frequencyHz;
    pll->trackingFrequencyHz = frequencyHz;
    pll->frequencyCorrectionHz = 0.0f;
    pll->latestPhaseDriftDegrees = 0.0f;
    pll->latestResidualFrequencyHz = 0.0f;
    pll->latestVectorMagnitudeCode = 0.0f;
    pll->previousMeasuredPhaseDegrees = 0.0f;
    pll->previousPhaseCaptured = false;
    pll->latestVectorValid = false;
    pll->rejectedWindowCount = 0U;
    ResetPllRecoveryState(pll);
}

static void ResetPllRecoveryState(IqPllState* pll) {
    pll->filteredResidualFrequencyHz = 0.0f;
    pll->residualFilterInitialized = false;
    pll->lockGoodWindowCount = 0U;
    pll->lockBadWindowCount = 0U;
    pll->pllLocked = false;
}

static uint32_t FrequencyToPhaseStep(float frequencyHz) {
    const float minimumTrackingFrequencyHz = (float)SIGNAL_FFT_MIN_FREQUENCY_HZ - IQ_PLL_CORRECTION_LIMIT_HZ;
    const float maximumTrackingFrequencyHz = (float)SIGNAL_FFT_MAX_FREQUENCY_HZ + IQ_PLL_CORRECTION_LIMIT_HZ;

    if (frequencyHz < minimumTrackingFrequencyHz) {
        frequencyHz = minimumTrackingFrequencyHz;
    }
    if (frequencyHz > maximumTrackingFrequencyHz) {
        frequencyHz = maximumTrackingFrequencyHz;
    }

    double phaseStep = (double)frequencyHz * IQ_PHASE_SCALE / (double)ADC_CAPTURE_SAMPLE_RATE_HZ;
    return (uint32_t)(phaseStep + 0.5);
}

static void QueueDecimatedSamples(const uint16_t* samples, uint32_t sampleCount) {
    uint32_t startPhase[IQ_PLL_CHANNEL_COUNT];
    uint32_t phaseStep[IQ_PLL_CHANNEL_COUNT];

    for (uint32_t channelIndex = 0U; channelIndex < IQ_PLL_CHANNEL_COUNT; channelIndex++) {
        IqPllState* pll = &pllChannels[channelIndex];
        startPhase[channelIndex] = pll->referencePhase;
        phaseStep[channelIndex] = pll->referencePhaseStep;
        pll->referencePhase += phaseStep[channelIndex] * sampleCount;
    }

    receivedBlockCount++;

    IqSampleBlock* block = &sampleBlocks[writeBlockIndex];
    if (block->ready) {
        droppedBlockCount++;
        return;
    }

    uint32_t outputCount = 0U;

    for (uint32_t index = 0U; index + IQ_DECIMATION_FACTOR <= sampleCount && outputCount < IQ_DECIMATED_BLOCK_SAMPLE_COUNT; index += IQ_DECIMATION_FACTOR) {
        uint32_t sum = 0U;

        for (uint32_t tap = 0U; tap < IQ_DECIMATION_FACTOR; tap++) {
            sum += samples[index + tap];
        }

        block->samples[outputCount] = (uint16_t)((sum + IQ_DECIMATION_FACTOR / 2U) / IQ_DECIMATION_FACTOR);
        outputCount++;
    }

    for (uint32_t channelIndex = 0U; channelIndex < IQ_PLL_CHANNEL_COUNT; channelIndex++) {
        uint32_t filterCenterPhaseOffset = (uint32_t)(((uint64_t)phaseStep[channelIndex] * (IQ_DECIMATION_FACTOR - 1U)) / 2ULL);
        block->startPhase[channelIndex] = startPhase[channelIndex] + filterCenterPhaseOffset;
        block->phaseStep[channelIndex] = phaseStep[channelIndex] * IQ_DECIMATION_FACTOR;
    }

    block->sampleCount = outputCount;
    __DMB();
    block->ready = true;
    writeBlockIndex = (writeBlockIndex + 1U) % IQ_BLOCK_QUEUE_DEPTH;
}

static void ProcessQueuedBlocks(void) {
    uint32_t processedThisCall = 0U;

    while (processedThisCall < IQ_BLOCKS_PER_PROCESS_CALL && sampleBlocks[readBlockIndex].ready) {
        IqSampleBlock* block = &sampleBlocks[readBlockIndex];
        uint32_t startCycleCount = DWT->CYCCNT;

        ProcessBlock(block);

        uint32_t elapsedCycleCount = DWT->CYCCNT - startCycleCount;
        if (elapsedCycleCount > maximumProcessCycleCount) {
            maximumProcessCycleCount = elapsedCycleCount;
        }

        __DMB();
        block->ready = false;
        readBlockIndex = (readBlockIndex + 1U) % IQ_BLOCK_QUEUE_DEPTH;
        processedBlockCount++;
        processedThisCall++;
    }
}

static void ProcessBlock(const IqSampleBlock* block) {
    uint32_t phase[IQ_PLL_CHANNEL_COUNT];

    for (uint32_t channelIndex = 0U; channelIndex < IQ_PLL_CHANNEL_COUNT; channelIndex++) {
        phase[channelIndex] = block->startPhase[channelIndex];
    }

    for (uint32_t index = 0U; index < block->sampleCount; index++) {
        int32_t sample = (int32_t)block->samples[index] - dcMeanCode;
        uint32_t windowWeight = hannWindow[accumulatorSampleCount];
        int64_t weightedSample = (int64_t)sample * windowWeight;

        for (uint32_t channelIndex = 0U; channelIndex < IQ_PLL_CHANNEL_COUNT; channelIndex++) {
            IqPllState* pll = &pllChannels[channelIndex];
            uint32_t sineIndex = phase[channelIndex] >> (32U - IQ_SINE_TABLE_BITS);
            uint32_t cosineIndex = (sineIndex + IQ_SINE_TABLE_SAMPLE_COUNT / 4U) & (IQ_SINE_TABLE_SAMPLE_COUNT - 1U);

            pll->accumulatorI += weightedSample * sineTable[cosineIndex];
            pll->accumulatorQ -= weightedSample * sineTable[sineIndex];
            phase[channelIndex] += block->phaseStep[channelIndex];
        }

        accumulatorSampleCount++;

        if (accumulatorSampleCount == IQ_PHASE_WINDOW_SAMPLE_COUNT) {
            if (resultReady) {
                overwrittenWindowCount++;
            }

            for (uint32_t channelIndex = 0U; channelIndex < IQ_PLL_CHANNEL_COUNT; channelIndex++) {
                IqPllState* pll = &pllChannels[channelIndex];
                pll->completedI = pll->accumulatorI;
                pll->completedQ = pll->accumulatorQ;
                UpdatePll(channelIndex, pll->accumulatorI, pll->accumulatorQ);
                pll->accumulatorI = 0;
                pll->accumulatorQ = 0;
            }

            completedWindowCount++;
            resultReady = true;
            accumulatorSampleCount = 0U;
        }
    }
}

static void UpdatePll(uint32_t channelIndex, int64_t resultI, int64_t resultQ) {
    IqPllState* pll = &pllChannels[channelIndex];
    const float coherentGain = 0.5f;
    const float normalization = coherentGain * (float)IQ_PHASE_WINDOW_SAMPLE_COUNT * 32767.0f * IQ_HANN_SCALE;
    float normalizedI = (float)resultI / normalization;
    float normalizedQ = (float)resultQ / normalization;
    pll->latestVectorMagnitudeCode = sqrtf(normalizedI * normalizedI + normalizedQ * normalizedQ);

    if (pll->latestVectorMagnitudeCode < IQ_PLL_MIN_VECTOR_MAGNITUDE_CODE) {
        pll->latestVectorValid = false;
        pll->rejectedWindowCount++;
        pll->previousPhaseCaptured = false;
        pll->latestPhaseDriftDegrees = 0.0f;
        pll->latestResidualFrequencyHz = 0.0f;
        ResetPllRecoveryState(pll);
        return;
    }

    pll->latestVectorValid = true;
    float measuredPhaseDegrees = atan2f(normalizedQ, normalizedI) * 57.2957795130823208768f;

    if (!pll->previousPhaseCaptured) {
        pll->previousMeasuredPhaseDegrees = measuredPhaseDegrees;
        pll->previousPhaseCaptured = true;
        pll->latestPhaseDriftDegrees = 0.0f;
        pll->latestResidualFrequencyHz = 0.0f;
        return;
    }

    pll->latestPhaseDriftDegrees = measuredPhaseDegrees - pll->previousMeasuredPhaseDegrees;
    pll->previousMeasuredPhaseDegrees = measuredPhaseDegrees;

    if (pll->latestPhaseDriftDegrees > 180.0f) {
        pll->latestPhaseDriftDegrees -= 360.0f;
    }
    else if (pll->latestPhaseDriftDegrees < -180.0f) {
        pll->latestPhaseDriftDegrees += 360.0f;
    }

    pll->latestResidualFrequencyHz = pll->latestPhaseDriftDegrees / (360.0f * IQ_PLL_WINDOW_SECONDS);
    float requestedCorrectionStepHz = IQ_PLL_FREQUENCY_TRACKING_GAIN * pll->latestResidualFrequencyHz;
    requestedCorrectionStepHz = ClampFloat(requestedCorrectionStepHz, -IQ_PLL_CORRECTION_STEP_LIMIT_HZ, IQ_PLL_CORRECTION_STEP_LIMIT_HZ);

    pll->frequencyCorrectionHz += requestedCorrectionStepHz;
    pll->frequencyCorrectionHz = ClampFloat(pll->frequencyCorrectionHz, -IQ_PLL_CORRECTION_LIMIT_HZ, IQ_PLL_CORRECTION_LIMIT_HZ);
    pll->trackingFrequencyHz = pll->initialFrequencyHz + pll->frequencyCorrectionHz;
    pll->referencePhaseStep = FrequencyToPhaseStep(pll->trackingFrequencyHz);
    SetDacFrequency(channelIndex, pll->trackingFrequencyHz);
    UpdateLockState(pll);
}

static void UpdateLockState(IqPllState* pll) {
    if (!pll->residualFilterInitialized) {
        pll->filteredResidualFrequencyHz = pll->latestResidualFrequencyHz;
        pll->residualFilterInitialized = true;
    }
    else {
        pll->filteredResidualFrequencyHz += IQ_PLL_LOCK_FILTER_GAIN * (pll->latestResidualFrequencyHz - pll->filteredResidualFrequencyHz);
    }

    if (!pll->pllLocked) {
        pll->lockBadWindowCount = 0U;

        if (fabsf(pll->filteredResidualFrequencyHz) < IQ_PLL_LOCK_ENTER_RESIDUAL_LIMIT_HZ) {
            if (pll->lockGoodWindowCount < IQ_PLL_LOCK_ENTER_WINDOW_COUNT) {
                pll->lockGoodWindowCount++;
            }
        }
        else if (pll->lockGoodWindowCount > 0U) {
            pll->lockGoodWindowCount--;
        }

        if (pll->lockGoodWindowCount >= IQ_PLL_LOCK_ENTER_WINDOW_COUNT) {
            pll->pllLocked = true;
            pll->lockBadWindowCount = 0U;
        }
    }
    else {
        if (fabsf(pll->filteredResidualFrequencyHz) > IQ_PLL_LOCK_EXIT_RESIDUAL_LIMIT_HZ) {
            if (pll->lockBadWindowCount < IQ_PLL_LOCK_EXIT_WINDOW_COUNT) {
                pll->lockBadWindowCount++;
            }
        }
        else {
            pll->lockBadWindowCount = 0U;
        }

        if (pll->lockBadWindowCount >= IQ_PLL_LOCK_EXIT_WINDOW_COUNT) {
            pll->pllLocked = false;
            pll->lockGoodWindowCount = 0U;
        }
    }
}

static void SetDacFrequency(uint32_t channelIndex, float frequencyHz) {
    if (channelIndex == IQ_PLL_CHANNEL_A) {
        DacNco_SetChannelAFrequency(frequencyHz);
    }
    else {
        DacNco_SetChannelBFrequency(frequencyHz);
    }
}

static void PrintPllDiagnostics(uint32_t channelIndex) {
    const IqPllState* pll = &pllChannels[channelIndex];
    const int64_t reportNormalization = (int64_t)(IQ_PHASE_WINDOW_SAMPLE_COUNT / 2U) * 32767LL * 32767LL;
    int32_t averageI = (int32_t)(pll->completedI / reportNormalization);
    int32_t averageQ = (int32_t)(pll->completedQ / reportNormalization);
    int32_t phaseDriftMilliDegrees = RoundFloatToInt32(pll->latestPhaseDriftDegrees * 1000.0f);
    int32_t residualMilliHz = RoundFloatToInt32(pll->latestResidualFrequencyHz * 1000.0f);
    int32_t filteredResidualMilliHz = RoundFloatToInt32(pll->filteredResidualFrequencyHz * 1000.0f);
    int32_t correctionMilliHz = RoundFloatToInt32(pll->frequencyCorrectionHz * 1000.0f);
    uint32_t vectorMagnitudeMilliCode = (uint32_t)(pll->latestVectorMagnitudeCode * 1000.0f + 0.5f);
    uint64_t trackingFrequencyMilliHz = (uint64_t)(pll->trackingFrequencyHz * 1000.0f + 0.5f);

    bool driftNegative = phaseDriftMilliDegrees < 0;
    bool residualNegative = residualMilliHz < 0;
    bool filteredResidualNegative = filteredResidualMilliHz < 0;
    bool correctionNegative = correctionMilliHz < 0;
    uint32_t absolutePhaseDriftMilliDegrees = (uint32_t)(driftNegative ? -phaseDriftMilliDegrees : phaseDriftMilliDegrees);
    uint32_t absoluteResidualMilliHz = (uint32_t)(residualNegative ? -residualMilliHz : residualMilliHz);
    uint32_t absoluteFilteredResidualMilliHz = (uint32_t)(filteredResidualNegative ? -filteredResidualMilliHz : filteredResidualMilliHz);
    uint32_t absoluteCorrectionMilliHz = (uint32_t)(correctionNegative ? -correctionMilliHz : correctionMilliHz);

    printf("[PLL_%c] drift=%s%lu.%03lu deg/win residual=%s%lu.%03lu Hz avg=%s%lu.%03lu Hz "
           "df=%s%lu.%03lu Hz freq=%lu.%03lu Hz lock=%u good=%lu bad=%lu valid=%u "
           "mag=%lu.%03lu I=%ld Q=%ld reject=%lu blocks=%lu/%lu q=%lu drop=%lu max=%lu us\r\n", channelIndex == IQ_PLL_CHANNEL_A ? 'A' : 'B',
           driftNegative ? "-" : "", (unsigned long)(absolutePhaseDriftMilliDegrees / 1000U), (unsigned long)(absolutePhaseDriftMilliDegrees % 1000U),
           residualNegative ? "-" : "", (unsigned long)(absoluteResidualMilliHz / 1000U), (unsigned long)(absoluteResidualMilliHz % 1000U),
           filteredResidualNegative ? "-" : "", (unsigned long)(absoluteFilteredResidualMilliHz / 1000U),
           (unsigned long)(absoluteFilteredResidualMilliHz % 1000U), correctionNegative ? "-" : "", (unsigned long)(absoluteCorrectionMilliHz / 1000U),
           (unsigned long)(absoluteCorrectionMilliHz % 1000U), (unsigned long)(trackingFrequencyMilliHz / 1000ULL),
           (unsigned long)(trackingFrequencyMilliHz % 1000ULL), pll->pllLocked ? 1U : 0U, (unsigned long)pll->lockGoodWindowCount,
           (unsigned long)pll->lockBadWindowCount, pll->latestVectorValid ? 1U : 0U, (unsigned long)(vectorMagnitudeMilliCode / 1000U),
           (unsigned long)(vectorMagnitudeMilliCode % 1000U), (long)averageI, (long)averageQ, (unsigned long)pll->rejectedWindowCount,
           (unsigned long)processedBlockCount, (unsigned long)receivedBlockCount, (unsigned long)CountQueuedBlocks(), (unsigned long)droppedBlockCount,
           (unsigned long)CycleCountToMicroseconds(maximumProcessCycleCount));
    (void)completedWindowCount;
    (void)overwrittenWindowCount;
}

static uint32_t CountQueuedBlocks(void) {
    uint32_t queuedCount = 0U;

    for (uint32_t index = 0U; index < IQ_BLOCK_QUEUE_DEPTH; index++) {
        if (sampleBlocks[index].ready) {
            queuedCount++;
        }
    }

    return queuedCount;
}

static uint32_t CycleCountToMicroseconds(uint32_t cycleCount) {
    if (SystemCoreClock == 0U) {
        return 0U;
    }

    return (uint32_t)(((uint64_t)cycleCount * 1000000ULL) / SystemCoreClock);
}

static float ClampFloat(float value, float minimum, float maximum) {
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }

    return value;
}

static int32_t RoundFloatToInt32(float value) {
    if (value >= 0.0f) {
        return (int32_t)(value + 0.5f);
    }

    return (int32_t)(value - 0.5f);
}
