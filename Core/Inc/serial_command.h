#ifndef SERIAL_COMMAND_H
#define SERIAL_COMMAND_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  Starts interrupt-driven command reception on USART1 and USART2.
  * @retval HAL status of the receive initialization.
  */
HAL_StatusTypeDef SerialCommand_Init(void);

/**
  * @brief  Processes complete commands outside interrupt context.
  */
void SerialCommand_Process(void);

/**
  * @brief  Handles a completed one-byte UART receive operation.
  * @param  huart UART that completed the receive operation.
  */
void SerialCommand_RxCpltCallback(UART_HandleTypeDef* huart);

/**
  * @brief  Recovers command reception after a UART error.
  * @param  huart UART that reported the error.
  */
void SerialCommand_ErrorCallback(UART_HandleTypeDef* huart);

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_COMMAND_H */
