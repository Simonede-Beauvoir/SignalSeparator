#include "signal_zoom_fft.h"

#include "main.h"

#include <string.h>

#define SIGNAL_ZOOM_FFT_TWO_PI                     6.28318530717958647692f
#define SIGNAL_ZOOM_FFT_HANN_COHERENT_GAIN         0.5f

#define SIGNAL_ZOOM_CIC_ORDER                      3U
#define SIGNAL_ZOOM_FILTER_TRANSIENT_SAMPLES       16U
#define SIGNAL_ZOOM_TOP_CANDIDATE_COUNT            16U
#define SIGNAL_ZOOM_NCO_NORMALIZE_INTERVAL         1024U

#define SIGNAL_ZOOM_CIC_GAIN \
    ((float64_t)SIGNAL_ZOOM_FFT_DECIMATION_FACTOR * \
     (float64_t)SIGNAL_ZOOM_FFT_DECIMATION_FACTOR * \
     (float64_t)SIGNAL_ZOOM_FFT_DECIMATION_FACTOR)

typedef struct {
    int32_t bin;
    float32_t power;
} SignalZoomCandidate;

/*
 * The streaming DDC uses a three-stage CIC decimator. The integrators run
 * for every raw sample, while the comb sections run only once per 64 input
 * samples. Float64 state preserves precision when a near-DC baseband tone
 * causes large integrator values before the comb subtraction.
 */
static arm_cfft_instance_f32 zoomFftInstance;

static float64_t cicIntegratorI[SIGNAL_ZOOM_CIC_ORDER];
static float64_t cicIntegratorQ[SIGNAL_ZOOM_CIC_ORDER];
static float64_t cicCombDelayI[SIGNAL_ZOOM_CIC_ORDER];
static float64_t cicCombDelayQ[SIGNAL_ZOOM_CIC_ORDER];

static float32_t zoomFftBuffer[2U * SIGNAL_ZOOM_FFT_SIZE];
static SignalZoomCandidate zoomCandidates[SIGNAL_ZOOM_TOP_CANDIDATE_COUNT];

static volatile bool zoomCaptureActive = false;
static volatile bool zoomCaptureComplete = false;

static bool zoomInitialized = false;
static float32_t zoomCenterFrequencyHz = 0.0f;
static float32_t zoomMeanCode = 0.0f;
static uint32_t zoomInputSampleRateHz = 0U;
static uint32_t zoomOutputSampleRateHz = 0U;

static float32_t ncoRotationReal = 1.0f;
static float32_t ncoRotationImag = 0.0f;
static float32_t ncoReal = 1.0f;
static float32_t ncoImag = 0.0f;
static uint32_t ncoSampleCount = 0U;

static uint32_t cicDecimationCounter = 0U;
static uint32_t discardedOutputCount = 0U;
static uint32_t zoomOutputCount = 0U;
static uint32_t zoomRawSampleCount = 0U;
static uint64_t zoomDdcCycleAccumulator = 0ULL;
static uint32_t zoomMaximumDdcCallbackCycles = 0U;
static uint32_t zoomDdcOverrunCount = 0U;

static void SignalZoomFFT_EnableCycleCounter(void);
static uint32_t SignalZoomFFT_GetCycleCount(void);
static float32_t SignalZoomFFT_CyclesToMicroseconds(uint32_t cycles);
static void SignalZoomFFT_ResetCic(void);
static void SignalZoomFFT_ProcessCicSample(float32_t mixedI, float32_t mixedQ);
static void SignalZoomFFT_UpdateNco(void);
static void SignalZoomFFT_NormalizeNco(void);
static void SignalZoomFFT_ApplyHannWindow(void);
static uint32_t SignalZoomFFT_MapSignedBin(int32_t signedBin);
static float32_t SignalZoomFFT_GetBinPower(int32_t signedBin);
static void SignalZoomFFT_InsertCandidate(int32_t bin, float32_t power, uint32_t* candidateCount);
static int32_t SignalZoomFFT_AbsoluteDifference(int32_t first, int32_t second);
static float32_t SignalZoomFFT_Clamp(float32_t value, float32_t minimum, float32_t maximum);
static SignalZoomFftPeak SignalZoomFFT_BuildPeak(int32_t peakBin);
static void SignalZoomFFT_FillTiming(SignalZoomFftResult* result, uint32_t analysisStartCycles, uint32_t fftStartCycles, uint32_t fftEndCycles);

arm_status SignalZoomFFT_Init(void) {
    arm_status status = arm_cfft_init_4096_f32(&zoomFftInstance);

    if (status != ARM_MATH_SUCCESS) {
        zoomInitialized = false;
        return status;
    }

    SignalZoomFFT_EnableCycleCounter();
    zoomInitialized = true;

    return ARM_MATH_SUCCESS;
}

arm_status SignalZoomFFT_Start(float32_t centerFrequencyHz, float32_t meanCode, uint32_t inputSampleRateHz) {
    if (!zoomInitialized || centerFrequencyHz <= 0.0f || inputSampleRateHz == 0U || inputSampleRateHz % SIGNAL_ZOOM_FFT_DECIMATION_FACTOR != 0U ||
        centerFrequencyHz >= (float32_t)inputSampleRateHz * 0.5f) {
        return ARM_MATH_ARGUMENT_ERROR;
    }

    zoomCaptureActive = false;
    zoomCaptureComplete = false;

    SignalZoomFFT_ResetCic();

    zoomCenterFrequencyHz = centerFrequencyHz;
    zoomMeanCode = meanCode;
    zoomInputSampleRateHz = inputSampleRateHz;
    zoomOutputSampleRateHz = inputSampleRateHz / SIGNAL_ZOOM_FFT_DECIMATION_FACTOR;

    float32_t phaseIncrement = SIGNAL_ZOOM_FFT_TWO_PI * centerFrequencyHz / (float32_t)inputSampleRateHz;

    ncoRotationReal = arm_cos_f32(phaseIncrement);
    ncoRotationImag = -arm_sin_f32(phaseIncrement);
    ncoReal = 1.0f;
    ncoImag = 0.0f;
    ncoSampleCount = 0U;

    cicDecimationCounter = 0U;
    discardedOutputCount = 0U;
    zoomOutputCount = 0U;
    zoomRawSampleCount = 0U;
    zoomDdcCycleAccumulator = 0ULL;
    zoomMaximumDdcCallbackCycles = 0U;
    zoomDdcOverrunCount = 0U;

    memset(zoomFftBuffer, 0, sizeof(zoomFftBuffer));

    zoomCaptureComplete = false;
    zoomCaptureActive = true;

    return ARM_MATH_SUCCESS;
}

void SignalZoomFFT_PushSamples(const uint16_t* samples, uint32_t sampleCount) {
    if (!zoomCaptureActive || zoomCaptureComplete || samples == NULL || sampleCount == 0U) {
        return;
    }

    uint32_t startCycles = SignalZoomFFT_GetCycleCount();
    uint32_t processedSampleCount = 0U;

    for (uint32_t index = 0U; index < sampleCount && zoomCaptureActive; index++) {
        float32_t centeredSample = (float32_t)samples[index] - zoomMeanCode;
        float32_t mixedI = centeredSample * ncoReal;
        float32_t mixedQ = centeredSample * ncoImag;

        zoomRawSampleCount++;
        processedSampleCount++;

        SignalZoomFFT_UpdateNco();
        SignalZoomFFT_ProcessCicSample(mixedI, mixedQ);
    }

    uint32_t endCycles = SignalZoomFFT_GetCycleCount();
    uint32_t callbackCycles = endCycles - startCycles;

    zoomDdcCycleAccumulator += callbackCycles;

    if (callbackCycles > zoomMaximumDdcCallbackCycles) {
        zoomMaximumDdcCallbackCycles = callbackCycles;
    }

    if (SystemCoreClock != 0U && zoomInputSampleRateHz != 0U && processedSampleCount != 0U) {
        uint64_t availableCycles = (uint64_t)SystemCoreClock * processedSampleCount / zoomInputSampleRateHz;

        if ((uint64_t)callbackCycles > availableCycles) {
            zoomDdcOverrunCount++;
        }
    }
}

bool SignalZoomFFT_IsCaptureComplete(void) {
    return zoomCaptureComplete;
}

bool SignalZoomFFT_Analyze(SignalZoomFftResult* result) {
    if (result != NULL) {
        memset(result, 0, sizeof(*result));
    }

    if (!zoomInitialized || !zoomCaptureComplete || result == NULL || zoomOutputSampleRateHz == 0U) {
        return false;
    }

    uint32_t analysisStartCycles = SignalZoomFFT_GetCycleCount();

    SignalZoomFFT_ApplyHannWindow();

    uint32_t fftStartCycles = SignalZoomFFT_GetCycleCount();

    arm_cfft_f32(&zoomFftInstance, zoomFftBuffer, 0U, 1U);

    uint32_t fftEndCycles = SignalZoomFFT_GetCycleCount();

    result->centerFrequencyHz = zoomCenterFrequencyHz;
    result->inputSampleRateHz = zoomInputSampleRateHz;
    result->outputSampleRateHz = zoomOutputSampleRateHz;
    result->binResolutionHz = (float32_t)zoomOutputSampleRateHz / (float32_t)SIGNAL_ZOOM_FFT_SIZE;
    result->rawSampleCount = zoomRawSampleCount;
    result->captureTimeUs = (float32_t)zoomRawSampleCount * 1000000.0f / (float32_t)zoomInputSampleRateHz;

    if (zoomDdcOverrunCount != 0U) {
        SignalZoomFFT_FillTiming(result, analysisStartCycles, fftStartCycles, fftEndCycles);
        return false;
    }

    int32_t maximumSignedBin = (int32_t)(((uint64_t)SIGNAL_ZOOM_FFT_SEARCH_HALF_BANDWIDTH_HZ * SIGNAL_ZOOM_FFT_SIZE) / zoomOutputSampleRateHz);

    if (maximumSignedBin < 2) {
        SignalZoomFFT_FillTiming(result, analysisStartCycles, fftStartCycles, fftEndCycles);
        return false;
    }

    if (maximumSignedBin >= (int32_t)(SIGNAL_ZOOM_FFT_SIZE / 2U - 1U)) {
        maximumSignedBin = (int32_t)(SIGNAL_ZOOM_FFT_SIZE / 2U - 2U);
    }

    uint32_t candidateCount = 0U;

    for (int32_t bin = -maximumSignedBin; bin <= maximumSignedBin; bin++) {
        float32_t previousPower = SignalZoomFFT_GetBinPower(bin - 1);
        float32_t currentPower = SignalZoomFFT_GetBinPower(bin);
        float32_t nextPower = SignalZoomFFT_GetBinPower(bin + 1);

        if (currentPower >= previousPower && currentPower > nextPower) {
            SignalZoomFFT_InsertCandidate(bin, currentPower, &candidateCount);
        }
    }

    if (candidateCount == 0U) {
        SignalZoomFFT_FillTiming(result, analysisStartCycles, fftStartCycles, fftEndCycles);
        return false;
    }

    SignalZoomCandidate firstCandidate = zoomCandidates[0];

    result->dominantPeak = SignalZoomFFT_BuildPeak(firstCandidate.bin);

    if (result->dominantPeak.amplitudePeakToPeakCode < SIGNAL_ZOOM_FFT_MIN_PEAK_VPP_CODE) {
        SignalZoomFFT_FillTiming(result, analysisStartCycles, fftStartCycles, fftEndCycles);
        return false;
    }

    SignalZoomCandidate secondCandidate = {0, 0.0f};
    bool secondCandidateFound = false;

    for (uint32_t index = 1U; index < candidateCount; index++) {
        int32_t separation = SignalZoomFFT_AbsoluteDifference(zoomCandidates[index].bin, firstCandidate.bin);

        bool powerIsValid = firstCandidate.power > 0.0f && zoomCandidates[index].power >= firstCandidate.power * SIGNAL_ZOOM_FFT_SECOND_PEAK_MIN_POWER_RATIO;

        if (separation >= (int32_t)SIGNAL_ZOOM_FFT_MIN_PEAK_SEPARATION_BINS && powerIsValid) {
            secondCandidate = zoomCandidates[index];
            secondCandidateFound = true;
            break;
        }
    }

    result->valid = true;

    if (!secondCandidateFound) {
        result->pattern = SIGNAL_ZOOM_FFT_PATTERN_SINGLE_CLUSTER;

        SignalZoomFFT_FillTiming(result, analysisStartCycles, fftStartCycles, fftEndCycles);
        return true;
    }

    SignalZoomFftPeak firstPeak = SignalZoomFFT_BuildPeak(firstCandidate.bin);
    SignalZoomFftPeak secondPeak = SignalZoomFFT_BuildPeak(secondCandidate.bin);

    if (firstPeak.frequencyHz < secondPeak.frequencyHz) {
        result->signalA = firstPeak;
        result->signalB = secondPeak;
    }
    else {
        result->signalA = secondPeak;
        result->signalB = firstPeak;
    }

    result->signalA.valid = true;
    result->signalB.valid = true;
    result->pattern = SIGNAL_ZOOM_FFT_PATTERN_TWO_SEPARATED;

    SignalZoomFFT_FillTiming(result, analysisStartCycles, fftStartCycles, fftEndCycles);

    return true;
}

void SignalZoomFFT_Abort(void) {
    zoomCaptureActive = false;
    zoomCaptureComplete = false;
    cicDecimationCounter = 0U;
    discardedOutputCount = 0U;
    zoomOutputCount = 0U;
}

static void SignalZoomFFT_EnableCycleCounter(void) {
#if defined(CoreDebug) && defined(DWT)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif
}

static uint32_t SignalZoomFFT_GetCycleCount(void) {
#if defined(DWT)
    return DWT->CYCCNT;
#else
    return 0U;
#endif
}

static float32_t SignalZoomFFT_CyclesToMicroseconds(uint32_t cycles) {
    if (SystemCoreClock == 0U) {
        return 0.0f;
    }

    return (float32_t)cycles * 1000000.0f / (float32_t)SystemCoreClock;
}

static void SignalZoomFFT_ResetCic(void) {
    memset(cicIntegratorI, 0, sizeof(cicIntegratorI));
    memset(cicIntegratorQ, 0, sizeof(cicIntegratorQ));
    memset(cicCombDelayI, 0, sizeof(cicCombDelayI));
    memset(cicCombDelayQ, 0, sizeof(cicCombDelayQ));

    cicDecimationCounter = 0U;
}

static void SignalZoomFFT_ProcessCicSample(float32_t mixedI, float32_t mixedQ) {
    cicIntegratorI[0] += (float64_t)mixedI;
    cicIntegratorQ[0] += (float64_t)mixedQ;

    for (uint32_t stage = 1U; stage < SIGNAL_ZOOM_CIC_ORDER; stage++) {
        cicIntegratorI[stage] += cicIntegratorI[stage - 1U];
        cicIntegratorQ[stage] += cicIntegratorQ[stage - 1U];
    }

    cicDecimationCounter++;

    if (cicDecimationCounter < SIGNAL_ZOOM_FFT_DECIMATION_FACTOR) {
        return;
    }

    cicDecimationCounter = 0U;

    float64_t outputI = cicIntegratorI[SIGNAL_ZOOM_CIC_ORDER - 1U];
    float64_t outputQ = cicIntegratorQ[SIGNAL_ZOOM_CIC_ORDER - 1U];

    for (uint32_t stage = 0U; stage < SIGNAL_ZOOM_CIC_ORDER; stage++) {
        float64_t delayedI = cicCombDelayI[stage];
        float64_t delayedQ = cicCombDelayQ[stage];

        cicCombDelayI[stage] = outputI;
        cicCombDelayQ[stage] = outputQ;

        outputI -= delayedI;
        outputQ -= delayedQ;
    }

    if (discardedOutputCount < SIGNAL_ZOOM_FILTER_TRANSIENT_SAMPLES) {
        discardedOutputCount++;
        return;
    }

    if (zoomOutputCount < SIGNAL_ZOOM_FFT_SIZE) {
        zoomFftBuffer[2U * zoomOutputCount] = (float32_t)(outputI / SIGNAL_ZOOM_CIC_GAIN);
        zoomFftBuffer[2U * zoomOutputCount + 1U] = (float32_t)(outputQ / SIGNAL_ZOOM_CIC_GAIN);

        zoomOutputCount++;
    }

    if (zoomOutputCount >= SIGNAL_ZOOM_FFT_SIZE) {
        zoomCaptureActive = false;
        zoomCaptureComplete = true;
    }
}

static void SignalZoomFFT_UpdateNco(void) {
    float32_t nextReal = ncoReal * ncoRotationReal - ncoImag * ncoRotationImag;
    float32_t nextImag = ncoReal * ncoRotationImag + ncoImag * ncoRotationReal;

    ncoReal = nextReal;
    ncoImag = nextImag;
    ncoSampleCount++;

    if (ncoSampleCount % SIGNAL_ZOOM_NCO_NORMALIZE_INTERVAL == 0U) {
        SignalZoomFFT_NormalizeNco();
    }
}

static void SignalZoomFFT_NormalizeNco(void) {
    float32_t magnitudeSquared = ncoReal * ncoReal + ncoImag * ncoImag;
    float32_t magnitude = 0.0f;

    if (arm_sqrt_f32(magnitudeSquared, &magnitude) != ARM_MATH_SUCCESS || magnitude <= 0.0f) {
        ncoReal = 1.0f;
        ncoImag = 0.0f;
        return;
    }

    float32_t scale = 1.0f / magnitude;

    ncoReal *= scale;
    ncoImag *= scale;
}

static void SignalZoomFFT_ApplyHannWindow(void) {
    float32_t phaseStep = SIGNAL_ZOOM_FFT_TWO_PI / (float32_t)SIGNAL_ZOOM_FFT_SIZE;
    float32_t rotationReal = arm_cos_f32(phaseStep);
    float32_t rotationImag = arm_sin_f32(phaseStep);
    float32_t oscillatorReal = 1.0f;
    float32_t oscillatorImag = 0.0f;

    for (uint32_t index = 0U; index < SIGNAL_ZOOM_FFT_SIZE; index++) {
        float32_t window = 0.5f - 0.5f * oscillatorReal;

        zoomFftBuffer[2U * index] *= window;
        zoomFftBuffer[2U * index + 1U] *= window;

        float32_t nextReal = oscillatorReal * rotationReal - oscillatorImag * rotationImag;
        float32_t nextImag = oscillatorReal * rotationImag + oscillatorImag * rotationReal;

        oscillatorReal = nextReal;
        oscillatorImag = nextImag;
    }
}

static uint32_t SignalZoomFFT_MapSignedBin(int32_t signedBin) {
    int32_t wrappedBin = signedBin;

    while (wrappedBin < 0) {
        wrappedBin += (int32_t)SIGNAL_ZOOM_FFT_SIZE;
    }

    while (wrappedBin >= (int32_t)SIGNAL_ZOOM_FFT_SIZE) {
        wrappedBin -= (int32_t)SIGNAL_ZOOM_FFT_SIZE;
    }

    return (uint32_t)wrappedBin;
}

static float32_t SignalZoomFFT_GetBinPower(int32_t signedBin) {
    uint32_t bin = SignalZoomFFT_MapSignedBin(signedBin);
    float32_t real = zoomFftBuffer[2U * bin];
    float32_t imaginary = zoomFftBuffer[2U * bin + 1U];

    return real * real + imaginary * imaginary;
}

static void SignalZoomFFT_InsertCandidate(int32_t bin, float32_t power, uint32_t* candidateCount) {
    uint32_t count = *candidateCount;
    uint32_t position = 0U;

    while (position < count && zoomCandidates[position].power >= power) {
        position++;
    }

    if (position >= SIGNAL_ZOOM_TOP_CANDIDATE_COUNT) {
        return;
    }

    uint32_t newCount = count;

    if (newCount < SIGNAL_ZOOM_TOP_CANDIDATE_COUNT) {
        newCount++;
    }

    for (uint32_t index = newCount - 1U; index > position; index--) {
        zoomCandidates[index] = zoomCandidates[index - 1U];
    }

    zoomCandidates[position].bin = bin;
    zoomCandidates[position].power = power;
    *candidateCount = newCount;
}

static int32_t SignalZoomFFT_AbsoluteDifference(int32_t first, int32_t second) {
    int32_t difference = first - second;

    return difference >= 0 ? difference : -difference;
}

static float32_t SignalZoomFFT_Clamp(float32_t value, float32_t minimum, float32_t maximum) {
    if (value < minimum) {
        return minimum;
    }

    if (value > maximum) {
        return maximum;
    }

    return value;
}

static SignalZoomFftPeak SignalZoomFFT_BuildPeak(int32_t peakBin) {
    SignalZoomFftPeak peak = {0};

    float32_t leftPower = SignalZoomFFT_GetBinPower(peakBin - 1);
    float32_t centerPower = SignalZoomFFT_GetBinPower(peakBin);
    float32_t rightPower = SignalZoomFFT_GetBinPower(peakBin + 1);

    float32_t denominator = leftPower - 2.0f * centerPower + rightPower;

    float32_t binOffset = 0.0f;

    if (denominator != 0.0f) {
        binOffset = 0.5f * (leftPower - rightPower) / denominator;
    }

    binOffset = SignalZoomFFT_Clamp(binOffset, -0.5f, 0.5f);

    float32_t interpolatedPower = centerPower - 0.25f * (leftPower - rightPower) * binOffset;

    if (interpolatedPower < 0.0f) {
        interpolatedPower = centerPower;
    }

    float32_t magnitude = 0.0f;

    if (arm_sqrt_f32(interpolatedPower, &magnitude) != ARM_MATH_SUCCESS) {
        magnitude = 0.0f;
    }

    float32_t basebandFrequencyHz = ((float32_t)peakBin + binOffset) * (float32_t)zoomOutputSampleRateHz / (float32_t)SIGNAL_ZOOM_FFT_SIZE;

    /*
     * Real input mixed with exp(-j*w_c*n) produces one retained complex
     * baseband tone with half the original real-signal peak amplitude.
     */
    float32_t amplitudePeakCode = 2.0f * magnitude / ((float32_t)SIGNAL_ZOOM_FFT_SIZE * SIGNAL_ZOOM_FFT_HANN_COHERENT_GAIN);

    peak.valid = true;
    peak.peakBin = peakBin;
    peak.binOffset = binOffset;
    peak.basebandFrequencyHz = basebandFrequencyHz;
    peak.frequencyHz = zoomCenterFrequencyHz + basebandFrequencyHz;
    peak.magnitude = magnitude;
    peak.amplitudePeakCode = amplitudePeakCode;
    peak.amplitudePeakToPeakCode = 2.0f * amplitudePeakCode;

    return peak;
}

static void SignalZoomFFT_FillTiming(SignalZoomFftResult* result, uint32_t analysisStartCycles, uint32_t fftStartCycles, uint32_t fftEndCycles) {
    uint32_t analysisEndCycles = SignalZoomFFT_GetCycleCount();

    uint64_t ddcCycles = zoomDdcCycleAccumulator;

    if (ddcCycles > UINT32_MAX) {
        ddcCycles = UINT32_MAX;
    }

    result->ddcCycles = (uint32_t)ddcCycles;
    result->maximumDdcCallbackCycles = zoomMaximumDdcCallbackCycles;
    result->ddcOverrunCount = zoomDdcOverrunCount;
    result->fftCycles = fftEndCycles - fftStartCycles;
    result->analysisCycles = analysisEndCycles - analysisStartCycles;
    result->ddcTimeUs = SignalZoomFFT_CyclesToMicroseconds(result->ddcCycles);
    result->maximumDdcCallbackTimeUs = SignalZoomFFT_CyclesToMicroseconds(result->maximumDdcCallbackCycles);
    result->fftTimeUs = SignalZoomFFT_CyclesToMicroseconds(result->fftCycles);
    result->analysisTimeUs = SignalZoomFFT_CyclesToMicroseconds(result->analysisCycles);
}
