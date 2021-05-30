/*
 * interrupts.c
 *
 *  Created on: May 29, 2021
 *      Author: imond
 */

#include "grbl.h"

void HAL_UART_TxCpltCallback (UART_HandleTypeDef *huart)
{
  if (huart->Instance == huart3.Instance)
    Serial_UART_TxCpltCallback (huart);
}

void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart)
{
  if (huart->Instance == huart3.Instance)
    Serial_UART_RxCpltCallback (huart);
}

void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{
  if ((GPIO_Pin & LIMIT_MASK) != 0) {
    Limit_GPIO_EXTI_Callback ();
  }
  if ((GPIO_Pin & CONTROL_MASK) != 0) {
    Control_GPIO_EXTI_Callback ();
  }
}
