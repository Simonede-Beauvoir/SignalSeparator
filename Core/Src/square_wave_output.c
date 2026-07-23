#include "square_wave_output.h"

#include "main.h"
#include "tim.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define SQUARE_WAVE_CHANNEL_COUNT 2U
#define SQUARE_WAVE_CHANNEL_A 0U
#define SQUARE_WAVE_CHANNEL_B 1U
#define SQUARE_WAVE_TIMER_FRACTION_SCALE 4294967296.0
#define SQUARE_WAVE_TIMER_HALF_SCALE (SQUARE_WAVE_TIMER_FRACTION_SCALE / 2.0)
#define SQUARE_WAVE_MINIMUM_INTERVAL_TICKS 2U

typedef struct {
    volatile uint32_t intervalWholeTicks;
    volatile uint32_t intervalFractionTicks;
    uint32_t intervalFractionAccumulator;
    volatile uint32_t edgeCount;
    volatile uint32_t lateEdgeCount;
    volatile bool active;
} SquareWaveChannelState;

static SquareWaveChannelState channels[SQUARE_WAVE_CHANNEL_COUNT];
static uint32_t timerClockHz;
static bool initialized;

static bool InitializeTimer(void);
static void ConfigureChannel(uint32_t channelIndex, const SignalFftSeparatedSignal* signal);
static void SetControlPin(uint32_t channelIndex, GPIO_PinState state);
static void SetFrequency(uint32_t channelIndex, float32_t frequencyHz);
static uint32_t GetTimerClockHz(void);
static uint32_t GetNextIntervalTicks(SquareWaveChannelState* channel);
static void ScheduleNextEdge(uint32_t channelIndex);
static uint32_t GetHalChannel(uint32_t channelIndex);

void SquareWaveOutput_Start(const SignalFftSeparatedSignal* signalA, const SignalFftSeparatedSignal* signalB) {
    if (signalA == NULL || signalB == NULL) {
        return;
    }

    if (!InitializeTimer()) {
        return;
    }

    ConfigureChannel(SQUARE_WAVE_CHANNEL_A, signalA);
    ConfigureChannel(SQUARE_WAVE_CHANNEL_B, signalB);
}

void SquareWaveOutput_SetChannelAFrequency(float32_t frequencyHz) {
    SetFrequency(SQUARE_WAVE_CHANNEL_A, frequencyHz);
}

void SquareWaveOutput_SetChannelBFrequency(float32_t frequencyHz) {
    SetFrequency(SQUARE_WAVE_CHANNEL_B, frequencyHz);
}

void SquareWaveOutput_PrintDiagnostics(void) {
    printf("[SQ_STATUS] A: active=%u edges=%lu late=%lu; B: active=%u edges=%lu late=%lu.\r\n", channels[SQUARE_WAVE_CHANNEL_A].active ? 1U : 0U,
           (unsigned long)channels[SQUARE_WAVE_CHANNEL_A].edgeCount, (unsigned long)channels[SQUARE_WAVE_CHANNEL_A].lateEdgeCount,
           channels[SQUARE_WAVE_CHANNEL_B].active ? 1U : 0U, (unsigned long)channels[SQUARE_WAVE_CHANNEL_B].edgeCount,
           (unsigned long)channels[SQUARE_WAVE_CHANNEL_B].lateEdgeCount);
}

static bool InitializeTimer(void) {
    if (initialized) {
        return true;
    }

    if (htim5.Instance != TIM5) {
        printf("ERROR: TIM5 was not initialized by MX_TIM5_Init().\r\n");
        return false;
    }

    if (htim5.Init.Period != UINT32_MAX) {
        printf("ERROR: TIM5 counter period must be 4294967295.\r\n");
        return false;
    }

    timerClockHz = GetTimerClockHz();
    initialized = true;
    return true;
}

static void ConfigureChannel(uint32_t channelIndex, const SignalFftSeparatedSignal* signal) {
    SquareWaveChannelState* channel = &channels[channelIndex];
    uint32_t halChannel = GetHalChannel(channelIndex);

    (void)HAL_TIM_OC_Stop_IT(&htim5, halChannel);
    channel->active = false;
    channel->edgeCount = 0U;
    channel->lateEdgeCount = 0U;
    SetControlPin(channelIndex, GPIO_PIN_RESET);

    if (signal->waveform != SIGNAL_WAVEFORM_SQUARE) {
        return;
    }

    SetFrequency(channelIndex, signal->frequencyHz);
    channel->intervalFractionAccumulator = 0U;

    uint32_t firstIntervalTicks = GetNextIntervalTicks(channel);
    __HAL_TIM_SET_COMPARE(&htim5, halChannel, __HAL_TIM_GET_COUNTER(&htim5) + firstIntervalTicks);

    channel->active = true;

    if (HAL_TIM_OC_Start_IT(&htim5, halChannel) != HAL_OK) {
        channel->active = false;
        printf("Square output %c failed to start.\r\n", channelIndex == SQUARE_WAVE_CHANNEL_A ? 'A' : 'B');
        return;
    }

    SetControlPin(channelIndex, GPIO_PIN_SET);

    uint32_t frequencyMilliHz = (uint32_t)(signal->frequencyHz * 1000.0f + 0.5f);
    printf("Square output %c: GPIO=%s, control=HIGH, frequency=%lu.%03lu Hz.\r\n", channelIndex == SQUARE_WAVE_CHANNEL_A ? 'A' : 'B',
           channelIndex == SQUARE_WAVE_CHANNEL_A ? "PA2/TIM5_CH3" : "PA3/TIM5_CH4", (unsigned long)(frequencyMilliHz / 1000U),
           (unsigned long)(frequencyMilliHz % 1000U));
}

static void SetControlPin(uint32_t channelIndex, GPIO_PinState state) {
    if (channelIndex == SQUARE_WAVE_CHANNEL_A) {
        HAL_GPIO_WritePin(A_SQ_CTL_GPIO_Port, A_SQ_CTL_Pin, state);
    }
    else {
        HAL_GPIO_WritePin(B_SQ_CTL_GPIO_Port, B_SQ_CTL_Pin, state);
    }
}

static void SetFrequency(uint32_t channelIndex, float32_t frequencyHz) {
    SquareWaveChannelState* channel = &channels[channelIndex];

    if (!initialized || frequencyHz <= 0.0f) {
        return;
    }

    double intervalQ32Double = (double)timerClockHz * SQUARE_WAVE_TIMER_HALF_SCALE / (double)frequencyHz;
    uint64_t intervalQ32 = (uint64_t)(intervalQ32Double + 0.5);
    uint32_t intervalWholeTicks = (uint32_t)(intervalQ32 >> 32U);

    if (intervalWholeTicks < SQUARE_WAVE_MINIMUM_INTERVAL_TICKS) {
        intervalWholeTicks = SQUARE_WAVE_MINIMUM_INTERVAL_TICKS;
        intervalQ32 = (uint64_t)intervalWholeTicks << 32U;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    channel->intervalWholeTicks = intervalWholeTicks;
    channel->intervalFractionTicks = (uint32_t)intervalQ32;

    if (primask == 0U) {
        __enable_irq();
    }
}

static uint32_t GetTimerClockHz(void) {
    uint32_t clockHz = HAL_RCC_GetPCLK1Freq();

    if ((RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) != RCC_D2CFGR_D2PPRE1_DIV1) {
        clockHz *= 2U;
    }

    return clockHz / (htim5.Init.Prescaler + 1U);
}

static uint32_t GetNextIntervalTicks(SquareWaveChannelState* channel) {
    uint32_t previousFraction = channel->intervalFractionAccumulator;
    channel->intervalFractionAccumulator += channel->intervalFractionTicks;
    uint32_t carry = channel->intervalFractionAccumulator < previousFraction ? 1U : 0U;
    return channel->intervalWholeTicks + carry;
}

static void ScheduleNextEdge(uint32_t channelIndex) {
    SquareWaveChannelState* channel = &channels[channelIndex];
    uint32_t intervalTicks = GetNextIntervalTicks(channel);
    uint32_t halChannel = GetHalChannel(channelIndex);
    uint32_t nextCompare = __HAL_TIM_GET_COMPARE(&htim5, halChannel) + intervalTicks;

    if ((int32_t)(nextCompare - __HAL_TIM_GET_COUNTER(&htim5)) <= 0) {
        nextCompare = __HAL_TIM_GET_COUNTER(&htim5) + intervalTicks;
        channel->lateEdgeCount++;
    }

    __HAL_TIM_SET_COMPARE(&htim5, halChannel, nextCompare);
    channel->edgeCount++;
}

static uint32_t GetHalChannel(uint32_t channelIndex) {
    return channelIndex == SQUARE_WAVE_CHANNEL_A ? TIM_CHANNEL_3 : TIM_CHANNEL_4;
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim->Instance != TIM5) {
        return;
    }

    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
        if (channels[SQUARE_WAVE_CHANNEL_A].active) {
            ScheduleNextEdge(SQUARE_WAVE_CHANNEL_A);
        }
    }
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
        if (channels[SQUARE_WAVE_CHANNEL_B].active) {
            ScheduleNextEdge(SQUARE_WAVE_CHANNEL_B);
        }
    }
}
