#include "serial_command.h"

#include "dac_nco.h"
#include "usart.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    UART_HandleTypeDef* huart;
    uint8_t rxByte;
    uint8_t resetMatchLength;
    char commandBuffer[32];
    volatile uint32_t commandLength;
    volatile bool commandReady;
} SerialCommandContext;

static SerialCommandContext commandContexts[] = {{.huart = &huart1}, {.huart = &huart2}};

static SerialCommandContext* GetCommandContext(UART_HandleTypeDef* huart);
static void CheckImmediateReset(SerialCommandContext* context);
static void ProcessCommand(SerialCommandContext* context);
static bool ParsePhaseCommand(const char* command, uint32_t* phaseDegrees);
static void SendLine(UART_HandleTypeDef* huart, const char* text);

HAL_StatusTypeDef SerialCommand_Init(void) {
    for (uint32_t index = 0U; index < sizeof(commandContexts) / sizeof(commandContexts[0]); index++) {
        SerialCommandContext* context = &commandContexts[index];
        HAL_StatusTypeDef status = HAL_UART_Receive_IT(context->huart, &context->rxByte, 1U);
        if (status != HAL_OK) {
            return status;
        }
    }

    return HAL_OK;
}

void SerialCommand_Process(void) {
    for (uint32_t index = 0U; index < sizeof(commandContexts) / sizeof(commandContexts[0]); index++) {
        ProcessCommand(&commandContexts[index]);
    }
}

void SerialCommand_RxCpltCallback(UART_HandleTypeDef* huart) {
    SerialCommandContext* context = GetCommandContext(huart);
    if (context == NULL) {
        return;
    }

    CheckImmediateReset(context);

    if (!context->commandReady) {
        if (context->rxByte == 13U || context->rxByte == 10U) {
            if (context->commandLength > 0U) {
                context->commandBuffer[context->commandLength] = 0;
                context->commandReady = true;
            }
        }
        else if (context->commandLength < sizeof(context->commandBuffer) - 1U) {
            context->commandBuffer[context->commandLength] = (char)context->rxByte;
            context->commandLength++;
        }
        else {
            context->commandLength = 0U;
        }
    }

    (void)HAL_UART_Receive_IT(context->huart, &context->rxByte, 1U);
}

void SerialCommand_ErrorCallback(UART_HandleTypeDef* huart) {
    SerialCommandContext* context = GetCommandContext(huart);
    if (context == NULL) {
        return;
    }

    __HAL_UART_CLEAR_OREFLAG(huart);
    context->resetMatchLength = 0U;
    context->commandLength = 0U;
    context->commandReady = false;
    (void)HAL_UART_Receive_IT(context->huart, &context->rxByte, 1U);
}

static SerialCommandContext* GetCommandContext(UART_HandleTypeDef* huart) {
    for (uint32_t index = 0U; index < sizeof(commandContexts) / sizeof(commandContexts[0]); index++) {
        if (commandContexts[index].huart->Instance == huart->Instance) {
            return &commandContexts[index];
        }
    }

    return NULL;
}

static void CheckImmediateReset(SerialCommandContext* context) {
    static const uint8_t resetCommand[] = {'R', 'E', 'S', 'E', 'T'};

    if (context->rxByte == resetCommand[context->resetMatchLength]) {
        context->resetMatchLength++;
        if (context->resetMatchLength == sizeof(resetCommand)) {
            context->resetMatchLength = 0U;
            NVIC_SystemReset();
        }

        return;
    }

    context->resetMatchLength = context->rxByte == resetCommand[0] ? 1U : 0U;
}

static void ProcessCommand(SerialCommandContext* context) {
    if (!context->commandReady) {
        return;
    }

    if (strcmp(context->commandBuffer, "RESET") == 0) {
        NVIC_SystemReset();
    }

    if (strncmp(context->commandBuffer, "SET PHASE", 9U) == 0) {
        uint32_t phaseDegrees;

        if (!ParsePhaseCommand(context->commandBuffer, &phaseDegrees)) {
            SendLine(context->huart, "ERROR: invalid SET PHASE syntax");
        }
        else if (phaseDegrees > 180U || phaseDegrees % 5U != 0U) {
            SendLine(context->huart, "ERROR: phase must be 0..180 in 5-degree steps");
        }
        else if (!DacNco_SetPhaseDifference(phaseDegrees)) {
            SendLine(context->huart, "ERROR: phase update failed");
        }
        else {
            char response[32];
            (void)snprintf(response, sizeof(response), "OK PHASE=%lu", (unsigned long)phaseDegrees);
            SendLine(context->huart, response);
        }
    }
    else {
        char response[64];
        (void)snprintf(response, sizeof(response), "ERROR: unknown command: %s", context->commandBuffer);
        SendLine(context->huart, response);
    }

    context->commandLength = 0U;
    context->commandReady = false;
}

static bool ParsePhaseCommand(const char* command, uint32_t* phaseDegrees) {
    static const char prefix[] = "SET PHASE ";

    if (strncmp(command, prefix, sizeof(prefix) - 1U) != 0) {
        return false;
    }

    const char* cursor = command + sizeof(prefix) - 1U;
    if (*cursor < '0' || *cursor > '9') {
        return false;
    }

    uint32_t value = 0U;
    while (*cursor >= '0' && *cursor <= '9') {
        if (value > 1000U) {
            return false;
        }

        value = value * 10U + (uint32_t)(*cursor - '0');
        cursor++;
    }

    if (*cursor != 0) {
        return false;
    }

    *phaseDegrees = value;
    return true;
}

static void SendLine(UART_HandleTypeDef* huart, const char* text) {
    static const uint8_t lineEnding[] = {13U, 10U};
    uint32_t length = (uint32_t)strlen(text);
    (void)HAL_UART_Transmit(huart, (const uint8_t*)text, (uint16_t)length, 1000U);
    (void)HAL_UART_Transmit(huart, lineEnding, sizeof(lineEnding), 1000U);
}
