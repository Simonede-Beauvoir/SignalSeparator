#include "signal_fft.h"

#include "main.h"

#include <float.h>
#include <math.h>
#include <string.h>

#define FFT_PI                     3.14159265358979323846f
#define FFT_TWO_PI                 6.28318530717958647692f
#define FFT_TRIANGLE_SCALE         0.81056946913870217155f
#define FFT_SQUARE_SCALE           1.27323954473516268615f
#define FFT_HANN_GAIN              0.5f
#define FFT_MAX_LOCAL_PEAKS        128U
#define FFT_MAX_MODEL_BINS          48U
#define FFT_PHASE_COUNT            (360U / SIGNAL_FFT_MODEL_PHASE_STEP_DEGREES)
#define FFT_COARSE_PHASE_STEP_COUNT (15U / SIGNAL_FFT_MODEL_PHASE_STEP_DEGREES)
#define FFT_FINE_PHASE_RADIUS_COUNT (10U / SIGNAL_FFT_MODEL_PHASE_STEP_DEGREES)

typedef struct {
    float32_t real;
    float32_t imaginary;
} ComplexValue;

typedef struct {
    uint32_t bin;
    float32_t power;
} LocalPeak;

typedef struct {
    uint32_t frequencyHz;
    float32_t score;
} FundamentalCandidate;

typedef struct {
    float32_t residual;
    float32_t amplitudeA;
    float32_t amplitudeB;
    uint32_t phaseA;
    uint32_t phaseB;
} ModelFit;

static arm_rfft_fast_instance_f32 fftInstance;
static float32_t fftInput[SIGNAL_FFT_SIZE];
static float32_t fftOutput[SIGNAL_FFT_SIZE];
static LocalPeak localPeaks[FFT_MAX_LOCAL_PEAKS];
static FundamentalCandidate candidates[SIGNAL_FFT_MAX_FUNDAMENTAL_CANDIDATES];
static uint32_t modelBins[FFT_MAX_MODEL_BINS];
static ComplexValue modelA[FFT_PHASE_COUNT][FFT_MAX_MODEL_BINS];
static ComplexValue modelB[FFT_PHASE_COUNT][FFT_MAX_MODEL_BINS];
static bool fftInitialized;
static bool coarseSpectrumReady;
static uint32_t coarseSampleRateHz;

static uint32_t GetCycles(void);
static float32_t CyclesToUs(uint32_t cycles);
static ComplexValue Add(ComplexValue a, ComplexValue b);
static ComplexValue Scale(ComplexValue value, float32_t scale);
static float32_t Inner(ComplexValue a, ComplexValue b);
static ComplexValue GetBin(uint32_t bin);
static float32_t GetPower(uint32_t bin);
static void PrepareInput(const uint16_t* samples, float32_t mean);
static SignalFftPeak BuildPeak(uint32_t bin, uint32_t sampleRateHz);
static uint32_t FindSignificantPeaks(uint32_t sampleRateHz, SignalFftResult* result);
static uint32_t BuildCandidates(const SignalFftResult* result);
static void AddCandidate(float32_t frequencyHz, float32_t score, float32_t mergeToleranceHz, uint32_t* count);
static void AddModelBin(uint32_t bin, uint32_t* count);
static void AddHarmonicBins(uint32_t frequencyHz, SignalWaveformType waveform, uint32_t sampleRateHz, uint32_t* count);
static uint32_t BuildModelBins(uint32_t frequencyA, SignalWaveformType waveformA, uint32_t frequencyB, SignalWaveformType waveformB,
                               const SignalFftResult* result, uint32_t sampleRateHz);
static ComplexValue RectangularResponse(float32_t offset);
static ComplexValue HannResponse(float32_t offset);
static ComplexValue SourceCoefficient(uint32_t frequencyHz, SignalWaveformType waveform, float32_t phase, uint32_t bin, uint32_t sampleRateHz);
static void BuildModels(uint32_t frequencyHz, SignalWaveformType waveform, uint32_t sampleRateHz, uint32_t binCount, ComplexValue models[][FFT_MAX_MODEL_BINS]);
static void EvaluatePhasePair(uint32_t phaseA, uint32_t phaseB, uint32_t binCount, float32_t observationEnergy, ModelFit* best);
static ModelFit FitModel(uint32_t binCount);
static bool FindBestModel(uint32_t sampleRateHz, SignalFftResult* result);
static void FinishTiming(SignalFftResult* result, uint32_t totalStart, uint32_t fftStart, uint32_t fftEnd);

arm_status SignalFFT_Init(void) {
    arm_status status = arm_rfft_fast_init_4096_f32(&fftInstance);

    if (status != ARM_MATH_SUCCESS) {
        fftInitialized = false;
        return status;
    }

#if defined(CoreDebug) && defined(DWT)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif
    fftInitialized = true;
    coarseSpectrumReady = false;
    coarseSampleRateHz = 0U;
    return ARM_MATH_SUCCESS;
}

bool SignalFFT_Analyze(const uint16_t* samples, uint32_t sampleCount, uint32_t sampleRateHz, SignalFftResult* result) {
    if (result != NULL) {
        memset(result, 0, sizeof(*result));
    }

    if (!fftInitialized || samples == NULL || result == NULL || sampleCount != SIGNAL_FFT_SIZE || sampleRateHz == 0U) {
        return false;
    }

    uint32_t totalStart = GetCycles();
    uint64_t sum = 0ULL;

    for (uint32_t index = 0U; index < SIGNAL_FFT_SIZE; index++) {
        sum += samples[index];
    }

    result->meanCode = (float32_t)sum / (float32_t)SIGNAL_FFT_SIZE;
    result->binResolutionHz = (float32_t)sampleRateHz / (float32_t)SIGNAL_FFT_SIZE;
    PrepareInput(samples, result->meanCode);

    uint32_t fftStart = GetCycles();
    arm_rfft_fast_f32(&fftInstance, fftInput, fftOutput, 0U);
    uint32_t fftEnd = GetCycles();

    coarseSpectrumReady = true;
    coarseSampleRateHz = sampleRateHz;

    result->significantPeakCount = FindSignificantPeaks(sampleRateHz, result);
    result->fundamentalCandidateCount = BuildCandidates(result);

    if (result->significantPeakCount != 0U && result->fundamentalCandidateCount >= 2U) {
        result->valid = FindBestModel(sampleRateHz, result) && result->residualRatio <= SIGNAL_FFT_MODEL_MAX_RESIDUAL_RATIO;
    }

    FinishTiming(result, totalStart, fftStart, fftEnd);
    return result->valid;
}

bool SignalFFT_FitResolvedFundamentals(uint32_t frequencyAHz, uint32_t frequencyBHz, SignalFftResult* result) {
    if (!fftInitialized || !coarseSpectrumReady || coarseSampleRateHz == 0U || result == NULL || frequencyAHz < SIGNAL_FFT_MIN_FREQUENCY_HZ || frequencyAHz >
        SIGNAL_FFT_MAX_FREQUENCY_HZ || frequencyBHz < SIGNAL_FFT_MIN_FREQUENCY_HZ || frequencyBHz > SIGNAL_FFT_MAX_FREQUENCY_HZ || frequencyAHz == frequencyBHz
        || result->significantPeakCount == 0U) {
        return false;
    }

    uint32_t startCycles = GetCycles();

    if (frequencyAHz > frequencyBHz) {
        uint32_t temporary = frequencyAHz;
        frequencyAHz = frequencyBHz;
        frequencyBHz = temporary;
    }

    memset(candidates, 0, sizeof(candidates));
    candidates[0].frequencyHz = frequencyAHz;
    candidates[0].score = 1.0f;
    candidates[1].frequencyHz = frequencyBHz;
    candidates[1].score = 1.0f;
    result->fundamentalCandidateCount = 2U;
    result->signalA = (SignalFftSeparatedSignal){0};
    result->signalB = (SignalFftSeparatedSignal){0};
    result->residualRatio = FLT_MAX;

    result->valid = FindBestModel(coarseSampleRateHz, result) && result->residualRatio <= SIGNAL_FFT_MODEL_MAX_RESIDUAL_RATIO;
    result->totalCycles += GetCycles() - startCycles;
    result->totalTimeUs = CyclesToUs(result->totalCycles);
    return result->valid;
}

const char* SignalFFT_GetWaveformName(SignalWaveformType waveform) {
    if (waveform == SIGNAL_WAVEFORM_SINE) {
        return "sine";
    }
    if (waveform == SIGNAL_WAVEFORM_TRIANGLE) {
        return "triangle";
    }
    if (waveform == SIGNAL_WAVEFORM_SQUARE) {
        return "square";
    }
    return "unknown";
}

static uint32_t GetCycles(void) {
#if defined(DWT)
    return DWT->CYCCNT;
#else
    return 0U;
#endif
}

static float32_t CyclesToUs(uint32_t cycles) {
    return SystemCoreClock == 0U ? 0.0f : (float32_t)cycles * 1000000.0f / (float32_t)SystemCoreClock;
}

static ComplexValue Add(ComplexValue a, ComplexValue b) {
    ComplexValue result = {a.real + b.real, a.imaginary + b.imaginary};
    return result;
}

static ComplexValue Scale(ComplexValue value, float32_t scale) {
    ComplexValue result = {value.real * scale, value.imaginary * scale};
    return result;
}

static float32_t Inner(ComplexValue a, ComplexValue b) {
    return a.real * b.real + a.imaginary * b.imaginary;
}

static ComplexValue GetBin(uint32_t bin) {
    ComplexValue value = {fftOutput[2U * bin], fftOutput[2U * bin + 1U]};
    return value;
}

static float32_t GetPower(uint32_t bin) {
    ComplexValue value = GetBin(bin);
    return Inner(value, value);
}

static void PrepareInput(const uint16_t* samples, float32_t mean) {
    float32_t step = FFT_TWO_PI / (float32_t)SIGNAL_FFT_SIZE;
    float32_t rotationReal = arm_cos_f32(step);
    float32_t rotationImaginary = arm_sin_f32(step);
    float32_t oscillatorReal = 1.0f;
    float32_t oscillatorImaginary = 0.0f;

    for (uint32_t index = 0U; index < SIGNAL_FFT_SIZE; index++) {
        fftInput[index] = ((float32_t)samples[index] - mean) * (0.5f - 0.5f * oscillatorReal);
        float32_t nextReal = oscillatorReal * rotationReal - oscillatorImaginary * rotationImaginary;
        float32_t nextImaginary = oscillatorReal * rotationImaginary + oscillatorImaginary * rotationReal;
        oscillatorReal = nextReal;
        oscillatorImaginary = nextImaginary;
    }
}

static SignalFftPeak BuildPeak(uint32_t bin, uint32_t sampleRateHz) {
    SignalFftPeak peak = {0};
    float32_t left = GetPower(bin - 1U);
    float32_t center = GetPower(bin);
    float32_t right = GetPower(bin + 1U);
    float32_t logLeft = logf(left > FLT_MIN ? left : FLT_MIN);
    float32_t logCenter = logf(center > FLT_MIN ? center : FLT_MIN);
    float32_t logRight = logf(right > FLT_MIN ? right : FLT_MIN);
    float32_t denominator = logLeft - 2.0f * logCenter + logRight;
    float32_t offset = denominator == 0.0f ? 0.0f : 0.5f * (logLeft - logRight) / denominator;

    if (offset < -0.5f) {
        offset = -0.5f;
    }
    if (offset > 0.5f) {
        offset = 0.5f;
    }

    float32_t interpolatedPower = center - 0.25f * (left - right) * offset;
    float32_t magnitude = 0.0f;
    ComplexValue complexValue = GetBin(bin);
    (void)arm_sqrt_f32(interpolatedPower > 0.0f ? interpolatedPower : center, &magnitude);

    peak.valid = true;
    peak.peakBin = bin;
    peak.binOffset = offset;
    peak.frequencyHz = ((float32_t)bin + offset) * (float32_t)sampleRateHz / (float32_t)SIGNAL_FFT_SIZE;
    peak.nominalFrequencyHz = peak.frequencyHz;
    peak.magnitude = magnitude;
    peak.complexReal = complexValue.real;
    peak.complexImaginary = complexValue.imaginary;
    peak.amplitudePeakCode = 2.0f * magnitude / ((float32_t)SIGNAL_FFT_SIZE * FFT_HANN_GAIN);
    peak.amplitudePeakToPeakCode = 2.0f * peak.amplitudePeakCode;
    return peak;
}

static uint32_t FindSignificantPeaks(uint32_t sampleRateHz, SignalFftResult* result) {
    uint32_t firstBin = (uint32_t)(((uint64_t)SIGNAL_FFT_MIN_FREQUENCY_HZ * SIGNAL_FFT_SIZE + sampleRateHz - 1U) / sampleRateHz);
    uint32_t lastBin = (uint32_t)(((uint64_t)SIGNAL_FFT_PEAK_SEARCH_MAX_FREQUENCY_HZ * SIGNAL_FFT_SIZE) / sampleRateHz);
    uint32_t localCount = 0U;
    if (lastBin >= SIGNAL_FFT_SIZE / 2U - 1U) {
        lastBin = SIGNAL_FFT_SIZE / 2U - 2U;
    }

    for (uint32_t bin = firstBin; bin <= lastBin && localCount < FFT_MAX_LOCAL_PEAKS; bin++) {
        float32_t power = GetPower(bin);
        if (power >= GetPower(bin - 1U) && power > GetPower(bin + 1U)) {
            localPeaks[localCount].bin = bin;
            localPeaks[localCount].power = power;
            localCount++;
        }
    }

    for (uint32_t index = 1U; index < localCount; index++) {
        LocalPeak value = localPeaks[index];
        uint32_t position = index;
        while (position > 0U && localPeaks[position - 1U].power < value.power) {
            localPeaks[position] = localPeaks[position - 1U];
            position--;
        }
        localPeaks[position] = value;
    }

    if (localCount == 0U || BuildPeak(localPeaks[0].bin, sampleRateHz).amplitudePeakToPeakCode < SIGNAL_FFT_MIN_PEAK_VPP_CODE) {
        return 0U;
    }

    uint32_t count = 0U;
    float32_t minimumPower = localPeaks[0].power * SIGNAL_FFT_SIGNIFICANT_POWER_RATIO;
    while (count < localCount && count < SIGNAL_FFT_MAX_SIGNIFICANT_PEAKS && localPeaks[count].power >= minimumPower) {
        SignalFftPeak peak = BuildPeak(localPeaks[count].bin, sampleRateHz);
        if (peak.amplitudePeakToPeakCode >= SIGNAL_FFT_MIN_PEAK_VPP_CODE) {
            result->significantPeaks[count] = peak;
            count++;
        }
        else {
            break;
        }
    }
    return count;
}

static void AddCandidate(float32_t frequencyHz, float32_t score, float32_t mergeToleranceHz, uint32_t* count) {
    if (frequencyHz < (float32_t)SIGNAL_FFT_MIN_FREQUENCY_HZ - mergeToleranceHz || frequencyHz > (float32_t)SIGNAL_FFT_MAX_FREQUENCY_HZ + mergeToleranceHz) {
        return;
    }
    if (frequencyHz < (float32_t)SIGNAL_FFT_MIN_FREQUENCY_HZ) {
        frequencyHz = (float32_t)SIGNAL_FFT_MIN_FREQUENCY_HZ;
    }
    if (frequencyHz > (float32_t)SIGNAL_FFT_MAX_FREQUENCY_HZ) {
        frequencyHz = (float32_t)SIGNAL_FFT_MAX_FREQUENCY_HZ;
    }
    for (uint32_t index = 0U; index < *count; index++) {
        if (fabsf((float32_t)candidates[index].frequencyHz - frequencyHz) <= mergeToleranceHz) {
            float32_t combinedScore = candidates[index].score + score;
            float32_t combinedFrequency = ((float32_t)candidates[index].frequencyHz * candidates[index].score + frequencyHz * score) / combinedScore;
            candidates[index].frequencyHz = (uint32_t)(combinedFrequency + 0.5f);
            candidates[index].score = combinedScore;
            return;
        }
    }
    uint32_t roundedFrequencyHz = (uint32_t)(frequencyHz + 0.5f);
    if (*count < SIGNAL_FFT_MAX_FUNDAMENTAL_CANDIDATES) {
        candidates[*count].frequencyHz = roundedFrequencyHz;
        candidates[*count].score = score;
        (*count)++;
        return;
    }
    uint32_t weakest = 0U;
    for (uint32_t index = 1U; index < *count; index++) {
        if (candidates[index].score < candidates[weakest].score) {
            weakest = index;
        }
    }
    if (score > candidates[weakest].score) {
        candidates[weakest].frequencyHz = roundedFrequencyHz;
        candidates[weakest].score = score;
    }
}

static uint32_t BuildCandidates(const SignalFftResult* result) {
    uint32_t count = 0U;
    uint32_t directFundamentalPeakCount = 0U;
    float32_t mergeToleranceHz = SIGNAL_FFT_CANDIDATE_MERGE_BINS * result->binResolutionHz;

    /*
     * Peaks already present inside the legal fundamental band are much
     * stronger evidence than a frequency obtained by dividing a harmonic.
     * If two such peaks are resolved, they are the two fundamental
     * candidates.  This prevents, for example, a real 100 kHz fundamental
     * from also creating a fictitious 33.3 kHz candidate through /3.
     */
    for (uint32_t index = 0U; index < result->significantPeakCount; index++) {
        const SignalFftPeak* peak = &result->significantPeaks[index];
        if (peak->frequencyHz <= (float32_t)SIGNAL_FFT_MAX_FREQUENCY_HZ + result->binResolutionHz) {
            float32_t power = peak->magnitude * peak->magnitude;
            AddCandidate(peak->frequencyHz, power, mergeToleranceHz, &count);
            directFundamentalPeakCount++;
        }
    }

    if (directFundamentalPeakCount < 2U) {
        for (uint32_t index = 0U; index < result->significantPeakCount; index++) {
            const SignalFftPeak* peak = &result->significantPeaks[index];
            float32_t power = peak->magnitude * peak->magnitude;

            for (uint32_t harmonic = 3U; harmonic <= 25U; harmonic += 2U) {
                float32_t fundamental = peak->frequencyHz / (float32_t)harmonic;
                if (fundamental < (float32_t)SIGNAL_FFT_MIN_FREQUENCY_HZ - result->binResolutionHz) {
                    break;
                }
                AddCandidate(fundamental, power / (float32_t)(harmonic * harmonic), mergeToleranceHz, &count);
            }
        }
    }
    for (uint32_t index = 1U; index < count; index++) {
        FundamentalCandidate value = candidates[index];
        uint32_t position = index;
        while (position > 0U && candidates[position - 1U].frequencyHz > value.frequencyHz) {
            candidates[position] = candidates[position - 1U];
            position--;
        }
        candidates[position] = value;
    }
    return count;
}

static void AddModelBin(uint32_t bin, uint32_t* count) {
    if (bin == 0U || bin >= SIGNAL_FFT_SIZE / 2U || *count >= FFT_MAX_MODEL_BINS) {
        return;
    }
    for (uint32_t index = 0U; index < *count; index++) {
        if (modelBins[index] == bin) {
            return;
        }
    }
    modelBins[*count] = bin;
    (*count)++;
}

static void AddHarmonicBins(uint32_t frequencyHz, SignalWaveformType waveform, uint32_t sampleRateHz, uint32_t* count) {
    for (uint32_t harmonic = 1U; harmonic * frequencyHz <= SIGNAL_FFT_PEAK_SEARCH_MAX_FREQUENCY_HZ; harmonic += 2U) {
        uint32_t bin = (uint32_t)(((uint64_t)harmonic * frequencyHz * SIGNAL_FFT_SIZE + sampleRateHz / 2U) / sampleRateHz);
        AddModelBin(bin, count);
        if (waveform == SIGNAL_WAVEFORM_SINE) {
            break;
        }
    }
}

static uint32_t BuildModelBins(uint32_t frequencyA, SignalWaveformType waveformA, uint32_t frequencyB, SignalWaveformType waveformB,
                               const SignalFftResult* result, uint32_t sampleRateHz) {
    uint32_t count = 0U;
    for (uint32_t index = 0U; index < result->significantPeakCount; index++) {
        AddModelBin(result->significantPeaks[index].peakBin, &count);
    }
    AddHarmonicBins(frequencyA, waveformA, sampleRateHz, &count);
    AddHarmonicBins(frequencyB, waveformB, sampleRateHz, &count);
    return count;
}

static ComplexValue RectangularResponse(float32_t offset) {
    int32_t rounded = offset >= 0.0f ? (int32_t)(offset + 0.5f) : (int32_t)(offset - 0.5f);
    float32_t integerError = offset - (float32_t)rounded;
    if (integerError < 1.0e-5f && integerError > -1.0e-5f) {
        ComplexValue exact = {rounded == 0 ? (float32_t)SIGNAL_FFT_SIZE : 0.0f, 0.0f};
        return exact;
    }
    float32_t numerator = arm_sin_f32(FFT_PI * offset);
    float32_t denominator = arm_sin_f32(FFT_PI * offset / (float32_t)SIGNAL_FFT_SIZE);
    if (denominator == 0.0f) {
        ComplexValue zero = {0.0f, 0.0f};
        return zero;
    }
    float32_t magnitude = numerator / denominator;
    float32_t phase = FFT_PI * offset * ((float32_t)SIGNAL_FFT_SIZE - 1.0f) / (float32_t)SIGNAL_FFT_SIZE;
    ComplexValue value = {magnitude * arm_cos_f32(phase), magnitude * arm_sin_f32(phase)};
    return value;
}

static ComplexValue HannResponse(float32_t offset) {
    return Add(Scale(RectangularResponse(offset), 0.5f),
               Add(Scale(RectangularResponse(offset + 1.0f), -0.25f), Scale(RectangularResponse(offset - 1.0f), -0.25f)));
}

static ComplexValue SourceCoefficient(uint32_t frequencyHz, SignalWaveformType waveform, float32_t phase, uint32_t bin, uint32_t sampleRateHz) {
    ComplexValue result = {0.0f, 0.0f};
    for (uint32_t harmonic = 1U; harmonic * frequencyHz <= SIGNAL_FFT_PEAK_SEARCH_MAX_FREQUENCY_HZ; harmonic += 2U) {
        float32_t coefficient = 1.0f;
        if (waveform == SIGNAL_WAVEFORM_TRIANGLE) {
            float32_t sign = (((harmonic - 1U) / 2U) & 1U) == 0U ? 1.0f : -1.0f;
            coefficient = sign * FFT_TRIANGLE_SCALE / (float32_t)(harmonic * harmonic);
        }
        else if (waveform == SIGNAL_WAVEFORM_SQUARE) {
            coefficient = FFT_SQUARE_SCALE / (float32_t)harmonic;
        }
        float32_t harmonicPhase = (float32_t)harmonic * phase;
        float32_t sine = arm_sin_f32(harmonicPhase);
        float32_t cosine = arm_cos_f32(harmonicPhase);
        float32_t toneBin = (float32_t)(harmonic * frequencyHz) * SIGNAL_FFT_SIZE / (float32_t)sampleRateHz;
        ComplexValue positiveResponse = HannResponse(toneBin - (float32_t)bin);
        ComplexValue negativeResponse = HannResponse(-toneBin - (float32_t)bin);
        ComplexValue positiveCoefficient = {0.5f * sine, -0.5f * cosine};
        ComplexValue negativeCoefficient = {0.5f * sine, 0.5f * cosine};
        ComplexValue positive = {
            positiveCoefficient.real * positiveResponse.real - positiveCoefficient.imaginary * positiveResponse.imaginary,
            positiveCoefficient.real * positiveResponse.imaginary + positiveCoefficient.imaginary * positiveResponse.real
        };
        ComplexValue negative = {
            negativeCoefficient.real * negativeResponse.real - negativeCoefficient.imaginary * negativeResponse.imaginary,
            negativeCoefficient.real * negativeResponse.imaginary + negativeCoefficient.imaginary * negativeResponse.real
        };
        result = Add(result, Scale(Add(positive, negative), coefficient));
        if (waveform == SIGNAL_WAVEFORM_SINE) {
            break;
        }
    }
    return result;
}

static void BuildModels(uint32_t frequencyHz, SignalWaveformType waveform, uint32_t sampleRateHz, uint32_t binCount,
                        ComplexValue models[][FFT_MAX_MODEL_BINS]) {
    for (uint32_t phaseIndex = 0U; phaseIndex < FFT_PHASE_COUNT; phaseIndex++) {
        float32_t phase = (float32_t)(phaseIndex * SIGNAL_FFT_MODEL_PHASE_STEP_DEGREES) * FFT_PI / 180.0f;
        for (uint32_t binIndex = 0U; binIndex < binCount; binIndex++) {
            models[phaseIndex][binIndex] = SourceCoefficient(frequencyHz, waveform, phase, modelBins[binIndex], sampleRateHz);
        }
    }
}

static void EvaluatePhasePair(uint32_t phaseA, uint32_t phaseB, uint32_t binCount, float32_t observationEnergy, ModelFit* best) {
    float32_t aa = 0.0f;
    float32_t ab = 0.0f;
    float32_t bb = 0.0f;
    float32_t ay = 0.0f;
    float32_t by = 0.0f;
    for (uint32_t binIndex = 0U; binIndex < binCount; binIndex++) {
        ComplexValue observation = GetBin(modelBins[binIndex]);
        aa += Inner(modelA[phaseA][binIndex], modelA[phaseA][binIndex]);
        ab += Inner(modelA[phaseA][binIndex], modelB[phaseB][binIndex]);
        bb += Inner(modelB[phaseB][binIndex], modelB[phaseB][binIndex]);
        ay += Inner(modelA[phaseA][binIndex], observation);
        by += Inner(modelB[phaseB][binIndex], observation);
    }
    float32_t determinant = aa * bb - ab * ab;
    if (determinant <= aa * bb * 1.0e-6f) {
        return;
    }
    float32_t amplitudeA = (ay * bb - by * ab) / determinant;
    float32_t amplitudeB = (by * aa - ay * ab) / determinant;
    if (amplitudeA < SIGNAL_FFT_MIN_PEAK_VPP_CODE * 0.25f || amplitudeB < SIGNAL_FFT_MIN_PEAK_VPP_CODE * 0.25f) {
        return;
    }
    float32_t errorEnergy = 0.0f;
    for (uint32_t binIndex = 0U; binIndex < binCount; binIndex++) {
        ComplexValue observation = GetBin(modelBins[binIndex]);
        ComplexValue prediction = Add(Scale(modelA[phaseA][binIndex], amplitudeA), Scale(modelB[phaseB][binIndex], amplitudeB));
        ComplexValue error = {observation.real - prediction.real, observation.imaginary - prediction.imaginary};
        errorEnergy += Inner(error, error);
    }
    float32_t residual = observationEnergy > 0.0f ? errorEnergy / observationEnergy : FLT_MAX;
    if (residual < best->residual) {
        best->residual = residual;
        best->amplitudeA = amplitudeA;
        best->amplitudeB = amplitudeB;
        best->phaseA = phaseA;
        best->phaseB = phaseB;
    }
}

static ModelFit FitModel(uint32_t binCount) {
    ModelFit best = {FLT_MAX, 0.0f, 0.0f, 0U, 0U};
    float32_t observationEnergy = 0.0f;
    for (uint32_t binIndex = 0U; binIndex < binCount; binIndex++) {
        ComplexValue observation = GetBin(modelBins[binIndex]);
        observationEnergy += Inner(observation, observation);
    }

    for (uint32_t phaseA = 0U; phaseA < FFT_PHASE_COUNT; phaseA += FFT_COARSE_PHASE_STEP_COUNT) {
        for (uint32_t phaseB = 0U; phaseB < FFT_PHASE_COUNT; phaseB += FFT_COARSE_PHASE_STEP_COUNT) {
            EvaluatePhasePair(phaseA, phaseB, binCount, observationEnergy, &best);
        }
    }

    uint32_t coarsePhaseA = best.phaseA;
    uint32_t coarsePhaseB = best.phaseB;
    for (int32_t offsetA = -(int32_t)FFT_FINE_PHASE_RADIUS_COUNT; offsetA <= (int32_t)FFT_FINE_PHASE_RADIUS_COUNT; offsetA++) {
        int32_t wrappedPhaseA = (int32_t)coarsePhaseA + offsetA;
        if (wrappedPhaseA < 0) {
            wrappedPhaseA += (int32_t)FFT_PHASE_COUNT;
        }
        uint32_t phaseA = (uint32_t)wrappedPhaseA % FFT_PHASE_COUNT;
        for (int32_t offsetB = -(int32_t)FFT_FINE_PHASE_RADIUS_COUNT; offsetB <= (int32_t)FFT_FINE_PHASE_RADIUS_COUNT; offsetB++) {
            int32_t wrappedPhaseB = (int32_t)coarsePhaseB + offsetB;
            if (wrappedPhaseB < 0) {
                wrappedPhaseB += (int32_t)FFT_PHASE_COUNT;
            }
            uint32_t phaseB = (uint32_t)wrappedPhaseB % FFT_PHASE_COUNT;
            EvaluatePhasePair(phaseA, phaseB, binCount, observationEnergy, &best);
        }
    }
    return best;
}

static bool FindBestModel(uint32_t sampleRateHz, SignalFftResult* result) {
    float32_t bestResidual = FLT_MAX;
    bool found = false;
    for (uint32_t indexA = 0U; indexA + 1U < result->fundamentalCandidateCount; indexA++) {
        for (uint32_t indexB = indexA + 1U; indexB < result->fundamentalCandidateCount; indexB++) {
            for (SignalWaveformType waveformA = SIGNAL_WAVEFORM_SINE; waveformA <= SIGNAL_WAVEFORM_SQUARE; waveformA++) {
                for (SignalWaveformType waveformB = SIGNAL_WAVEFORM_SINE; waveformB <= SIGNAL_WAVEFORM_SQUARE; waveformB++) {
                    uint32_t frequencyA = candidates[indexA].frequencyHz;
                    uint32_t frequencyB = candidates[indexB].frequencyHz;
                    uint32_t binCount = BuildModelBins(frequencyA, waveformA, frequencyB, waveformB, result, sampleRateHz);
                    BuildModels(frequencyA, waveformA, sampleRateHz, binCount, modelA);
                    BuildModels(frequencyB, waveformB, sampleRateHz, binCount, modelB);
                    ModelFit fit = FitModel(binCount);
                    if (fit.residual < bestResidual) {
                        bestResidual = fit.residual;
                        result->signalA = (SignalFftSeparatedSignal){
                            true, (float32_t)frequencyA, waveformA, fit.amplitudeA, (float32_t)(fit.phaseA * SIGNAL_FFT_MODEL_PHASE_STEP_DEGREES)
                        };
                        result->signalB = (SignalFftSeparatedSignal){
                            true, (float32_t)frequencyB, waveformB, fit.amplitudeB, (float32_t)(fit.phaseB * SIGNAL_FFT_MODEL_PHASE_STEP_DEGREES)
                        };
                        found = true;
                    }
                }
            }
        }
    }
    result->residualRatio = bestResidual;
    return found;
}

static void FinishTiming(SignalFftResult* result, uint32_t totalStart, uint32_t fftStart, uint32_t fftEnd) {
    result->fftCycles = fftEnd - fftStart;
    result->totalCycles = GetCycles() - totalStart;
    result->fftTimeUs = CyclesToUs(result->fftCycles);
    result->totalTimeUs = CyclesToUs(result->totalCycles);
}
