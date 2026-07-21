#include "signal_fft.h"

#include "main.h"

#include <string.h>

#define SIGNAL_FFT_TWO_PI                         6.28318530717958647692f
#define SIGNAL_FFT_HANN_COHERENT_GAIN             0.5f
#define SIGNAL_FFT_MAX_CANDIDATE_COUNT             128U
#define SIGNAL_FFT_MIN_PEAK_SEPARATION_BINS        4U

/*
 * The second detected tone must have at least 5 percent of the first tone's
 * amplitude. Because power is magnitude squared, the power ratio is 0.0025.
 */
#define SIGNAL_FFT_SECOND_PEAK_MIN_POWER_RATIO     0.0025f

typedef struct {
    uint32_t bin;
    float32_t power;
} SignalFftCandidate;

static arm_rfft_fast_instance_f32 fftInstance;
static float32_t fftInput[SIGNAL_FFT_SIZE];
static float32_t fftOutput[SIGNAL_FFT_SIZE];
static float32_t hannWindow[SIGNAL_FFT_SIZE];

static bool fftInitialized = false;

static void SignalFFT_EnableCycleCounter(void);
static uint32_t SignalFFT_GetCycleCount(void);
static float32_t SignalFFT_CyclesToMicroseconds(uint32_t cycles);
static float32_t SignalFFT_GetBinPower(uint32_t bin);
static uint32_t SignalFFT_AbsoluteDifference(uint32_t first, uint32_t second);
static float32_t SignalFFT_Clamp(float32_t value, float32_t minimum, float32_t maximum);
static float32_t SignalFFT_SnapFrequency(float32_t frequencyHz);
static SignalFftPeak SignalFFT_BuildPeak(
    uint32_t peakBin,
    uint32_t sampleRateHz);

arm_status SignalFFT_Init(void) {
    arm_status status = arm_rfft_fast_init_4096_f32(&fftInstance);

    if (status != ARM_MATH_SUCCESS) {
        fftInitialized = false;
        return status;
    }

    for (uint32_t index = 0U; index < SIGNAL_FFT_SIZE; index++) {
        float32_t phase =
            SIGNAL_FFT_TWO_PI * (float32_t)index /
            (float32_t)SIGNAL_FFT_SIZE;

        hannWindow[index] =
            0.5f - 0.5f * arm_cos_f32(phase);
    }

    SignalFFT_EnableCycleCounter();
    fftInitialized = true;

    return ARM_MATH_SUCCESS;
}

bool SignalFFT_Analyze(
    const uint16_t* samples,
    uint32_t sampleCount,
    uint32_t sampleRateHz,
    SignalFftResult* result) {
    if (!fftInitialized ||
        samples == NULL ||
        result == NULL ||
        sampleCount != SIGNAL_FFT_SIZE ||
        sampleRateHz == 0U) {
        return false;
    }

    memset(result, 0, sizeof(*result));

    uint32_t totalStartCycles = SignalFFT_GetCycleCount();

    uint64_t sampleSum = 0ULL;

    for (uint32_t index = 0U; index < SIGNAL_FFT_SIZE; index++) {
        sampleSum += samples[index];
    }

    result->meanCode =
        (float32_t)sampleSum / (float32_t)SIGNAL_FFT_SIZE;

    for (uint32_t index = 0U; index < SIGNAL_FFT_SIZE; index++) {
        float32_t centeredSample =
            (float32_t)samples[index] - result->meanCode;

        fftInput[index] =
            centeredSample * hannWindow[index];
    }

    uint32_t fftStartCycles = SignalFFT_GetCycleCount();

    /*
     * CMSIS-DSP modifies fftInput while calculating the real FFT.
     * fftOutput uses the packed real-FFT format documented by CMSIS-DSP.
     */
    arm_rfft_fast_f32(
        &fftInstance,
        fftInput,
        fftOutput,
        0U);

    uint32_t fftEndCycles = SignalFFT_GetCycleCount();

    result->binResolutionHz =
        (float32_t)sampleRateHz / (float32_t)SIGNAL_FFT_SIZE;

    uint32_t minimumBin =
        (uint32_t)(
            ((uint64_t)SIGNAL_FFT_MIN_FREQUENCY_HZ *
             SIGNAL_FFT_SIZE +
             sampleRateHz - 1U) /
            sampleRateHz);

    uint32_t maximumBin =
        (uint32_t)(
            ((uint64_t)SIGNAL_FFT_MAX_FREQUENCY_HZ *
             SIGNAL_FFT_SIZE) /
            sampleRateHz);

    if (minimumBin < 1U) {
        minimumBin = 1U;
    }

    if (maximumBin >= SIGNAL_FFT_SIZE / 2U) {
        maximumBin = SIGNAL_FFT_SIZE / 2U - 1U;
    }

    if (minimumBin >= maximumBin) {
        return false;
    }

    SignalFftCandidate candidates[SIGNAL_FFT_MAX_CANDIDATE_COUNT];
    uint32_t candidateCount = 0U;

    for (uint32_t bin = minimumBin; bin <= maximumBin; bin++) {
        float32_t previousPower = SignalFFT_GetBinPower(bin - 1U);
        float32_t currentPower = SignalFFT_GetBinPower(bin);
        float32_t nextPower = SignalFFT_GetBinPower(bin + 1U);

        if (currentPower >= previousPower &&
            currentPower > nextPower &&
            candidateCount < SIGNAL_FFT_MAX_CANDIDATE_COUNT) {
            candidates[candidateCount].bin = bin;
            candidates[candidateCount].power = currentPower;
            candidateCount++;
        }
    }

    if (candidateCount < 2U) {
        return false;
    }

    /*
     * Candidate count is small, so insertion sort keeps the code simple and
     * deterministic while avoiding dynamic allocation.
     */
    for (uint32_t index = 1U; index < candidateCount; index++) {
        SignalFftCandidate value = candidates[index];
        uint32_t position = index;

        while (position > 0U &&
               candidates[position - 1U].power < value.power) {
            candidates[position] = candidates[position - 1U];
            position--;
        }

        candidates[position] = value;
    }

    SignalFftCandidate firstCandidate = candidates[0];
    SignalFftCandidate secondCandidate = {0U, 0.0f};
    bool secondCandidateFound = false;

    for (uint32_t index = 1U; index < candidateCount; index++) {
        uint32_t separation = SignalFFT_AbsoluteDifference(
            candidates[index].bin,
            firstCandidate.bin);

        if (separation >= SIGNAL_FFT_MIN_PEAK_SEPARATION_BINS) {
            secondCandidate = candidates[index];
            secondCandidateFound = true;
            break;
        }
    }

    if (!secondCandidateFound ||
        firstCandidate.power <= 0.0f ||
        secondCandidate.power <
            firstCandidate.power *
            SIGNAL_FFT_SECOND_PEAK_MIN_POWER_RATIO) {
        return false;
    }

    SignalFftPeak firstPeak =
        SignalFFT_BuildPeak(firstCandidate.bin, sampleRateHz);

    SignalFftPeak secondPeak =
        SignalFFT_BuildPeak(secondCandidate.bin, sampleRateHz);

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
    result->valid = true;

    uint32_t totalEndCycles = SignalFFT_GetCycleCount();

    result->fftCycles = fftEndCycles - fftStartCycles;
    result->totalCycles = totalEndCycles - totalStartCycles;
    result->fftTimeUs =
        SignalFFT_CyclesToMicroseconds(result->fftCycles);
    result->totalTimeUs =
        SignalFFT_CyclesToMicroseconds(result->totalCycles);

    return true;
}

static void SignalFFT_EnableCycleCounter(void) {
#if defined(CoreDebug) && defined(DWT)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif
}

static uint32_t SignalFFT_GetCycleCount(void) {
#if defined(DWT)
    return DWT->CYCCNT;
#else
    return 0U;
#endif
}

static float32_t SignalFFT_CyclesToMicroseconds(uint32_t cycles) {
    if (SystemCoreClock == 0U) {
        return 0.0f;
    }

    return
        (float32_t)cycles * 1000000.0f /
        (float32_t)SystemCoreClock;
}

static float32_t SignalFFT_GetBinPower(uint32_t bin) {
    float32_t real = fftOutput[2U * bin];
    float32_t imaginary = fftOutput[2U * bin + 1U];

    return real * real + imaginary * imaginary;
}

static uint32_t SignalFFT_AbsoluteDifference(
    uint32_t first,
    uint32_t second) {
    return first >= second ? first - second : second - first;
}

static float32_t SignalFFT_Clamp(
    float32_t value,
    float32_t minimum,
    float32_t maximum) {
    if (value < minimum) {
        return minimum;
    }

    if (value > maximum) {
        return maximum;
    }

    return value;
}

static float32_t SignalFFT_SnapFrequency(float32_t frequencyHz) {
    float32_t step = (float32_t)SIGNAL_FFT_NOMINAL_STEP_HZ;
    float32_t snapped =
        (float32_t)((uint32_t)(frequencyHz / step + 0.5f)) *
        step;

    return SignalFFT_Clamp(
        snapped,
        (float32_t)SIGNAL_FFT_MIN_FREQUENCY_HZ,
        (float32_t)SIGNAL_FFT_MAX_FREQUENCY_HZ);
}

static SignalFftPeak SignalFFT_BuildPeak(
    uint32_t peakBin,
    uint32_t sampleRateHz) {
    SignalFftPeak peak = {0};

    float32_t leftPower = SignalFFT_GetBinPower(peakBin - 1U);
    float32_t centerPower = SignalFFT_GetBinPower(peakBin);
    float32_t rightPower = SignalFFT_GetBinPower(peakBin + 1U);

    float32_t denominator =
        leftPower - 2.0f * centerPower + rightPower;

    float32_t binOffset = 0.0f;

    if (denominator != 0.0f) {
        binOffset =
            0.5f * (leftPower - rightPower) /
            denominator;
    }

    binOffset = SignalFFT_Clamp(binOffset, -0.5f, 0.5f);

    float32_t interpolatedPower =
        centerPower -
        0.25f * (leftPower - rightPower) * binOffset;

    if (interpolatedPower < 0.0f) {
        interpolatedPower = centerPower;
    }

    float32_t magnitude = 0.0f;

    if (arm_sqrt_f32(interpolatedPower, &magnitude) !=
        ARM_MATH_SUCCESS) {
        magnitude = 0.0f;
    }

    float32_t frequencyHz =
        ((float32_t)peakBin + binOffset) *
        (float32_t)sampleRateHz /
        (float32_t)SIGNAL_FFT_SIZE;

    /*
     * For a periodic Hann window, coherent gain is 0.5.
     * The real FFT single-sided peak amplitude is:
     *
     *     amplitudePeak = 2 * |X[k]| / (N * coherentGain)
     */
    float32_t amplitudePeakCode =
        2.0f * magnitude /
        ((float32_t)SIGNAL_FFT_SIZE *
         SIGNAL_FFT_HANN_COHERENT_GAIN);

    peak.valid = true;
    peak.peakBin = peakBin;
    peak.binOffset = binOffset;
    peak.frequencyHz = frequencyHz;
    peak.nominalFrequencyHz =
        SignalFFT_SnapFrequency(frequencyHz);
    peak.magnitude = magnitude;
    peak.amplitudePeakCode = amplitudePeakCode;
    peak.amplitudePeakToPeakCode =
        2.0f * amplitudePeakCode;

    return peak;
}
