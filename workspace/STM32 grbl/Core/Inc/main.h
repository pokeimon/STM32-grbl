/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern UART_HandleTypeDef huart3;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
#define FLASH_BASE_ADDR      (uint32_t)(FLASH_BASE)
#define FLASH_END_ADDR       (uint32_t)(0x081FFFFF)

/* Base address of the Flash sectors Bank 1 */
#define ADDR_FLASH_SECTOR_0_BANK1     ((uint32_t)0x08000000) /* Base @ of Sector 0, 128 Kbytes */
#define ADDR_FLASH_SECTOR_1_BANK1     ((uint32_t)0x08020000) /* Base @ of Sector 1, 128 Kbytes */
#define ADDR_FLASH_SECTOR_2_BANK1     ((uint32_t)0x08040000) /* Base @ of Sector 2, 128 Kbytes */
#define ADDR_FLASH_SECTOR_3_BANK1     ((uint32_t)0x08060000) /* Base @ of Sector 3, 128 Kbytes */
#define ADDR_FLASH_SECTOR_4_BANK1     ((uint32_t)0x08080000) /* Base @ of Sector 4, 128 Kbytes */
#define ADDR_FLASH_SECTOR_5_BANK1     ((uint32_t)0x080A0000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6_BANK1     ((uint32_t)0x080C0000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7_BANK1     ((uint32_t)0x080E0000) /* Base @ of Sector 7, 128 Kbytes */

/* Base address of the Flash sectors Bank 2 */
#define ADDR_FLASH_SECTOR_0_BANK2     ((uint32_t)0x08100000) /* Base @ of Sector 0, 128 Kbytes */
#define ADDR_FLASH_SECTOR_1_BANK2     ((uint32_t)0x08120000) /* Base @ of Sector 1, 128 Kbytes */
#define ADDR_FLASH_SECTOR_2_BANK2     ((uint32_t)0x08140000) /* Base @ of Sector 2, 128 Kbytes */
#define ADDR_FLASH_SECTOR_3_BANK2     ((uint32_t)0x08160000) /* Base @ of Sector 3, 128 Kbytes */
#define ADDR_FLASH_SECTOR_4_BANK2     ((uint32_t)0x08180000) /* Base @ of Sector 4, 128 Kbytes */
#define ADDR_FLASH_SECTOR_5_BANK2     ((uint32_t)0x081A0000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6_BANK2     ((uint32_t)0x081C0000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7_BANK2     ((uint32_t)0x081E0000) /* Base @ of Sector 7, 128 Kbytes */
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define X_DIRECTION_Pin GPIO_PIN_4
#define X_DIRECTION_GPIO_Port GPIOE
#define Y_DIRECTION_Pin GPIO_PIN_5
#define Y_DIRECTION_GPIO_Port GPIOE
#define Z_DIRECTION_Pin GPIO_PIN_6
#define Z_DIRECTION_GPIO_Port GPIOE
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define PROBE_Pin GPIO_PIN_10
#define PROBE_GPIO_Port GPIOF
#define SPINDLE_Pin GPIO_PIN_3
#define SPINDLE_GPIO_Port GPIOA
#define STEPPERS_DISABLE_Pin GPIO_PIN_6
#define STEPPERS_DISABLE_GPIO_Port GPIOA
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define Coolant_Flood_Pin GPIO_PIN_1
#define Coolant_Flood_GPIO_Port GPIOB
#define X_STEP_Pin GPIO_PIN_11
#define X_STEP_GPIO_Port GPIOE
#define Y_STEP_Pin GPIO_PIN_13
#define Y_STEP_GPIO_Port GPIOE
#define Z_STEP_Pin GPIO_PIN_14
#define Z_STEP_GPIO_Port GPIOE
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLINK_RX_Pin GPIO_PIN_8
#define STLINK_RX_GPIO_Port GPIOD
#define STLINK_TX_Pin GPIO_PIN_9
#define STLINK_TX_GPIO_Port GPIOD
#define USB_OTG_FS_PWR_EN_Pin GPIO_PIN_10
#define USB_OTG_FS_PWR_EN_GPIO_Port GPIOD
#define Z_LIMIT_Pin GPIO_PIN_13
#define Z_LIMIT_GPIO_Port GPIOD
#define Z_LIMIT_EXTI_IRQn EXTI15_10_IRQn
#define Y_LIMIT_Pin GPIO_PIN_14
#define Y_LIMIT_GPIO_Port GPIOD
#define Y_LIMIT_EXTI_IRQn EXTI15_10_IRQn
#define X_LIMIT_Pin GPIO_PIN_15
#define X_LIMIT_GPIO_Port GPIOD
#define X_LIMIT_EXTI_IRQn EXTI15_10_IRQn
#define CONTROL_RESET_Pin GPIO_PIN_6
#define CONTROL_RESET_GPIO_Port GPIOC
#define CONTROL_RESET_EXTI_IRQn EXTI9_5_IRQn
#define CONTROL_FEED_HOLD_Pin GPIO_PIN_7
#define CONTROL_FEED_HOLD_GPIO_Port GPIOC
#define CONTROL_FEED_HOLD_EXTI_IRQn EXTI9_5_IRQn
#define CONTROL_CYCLE_START_Pin GPIO_PIN_8
#define CONTROL_CYCLE_START_GPIO_Port GPIOC
#define CONTROL_CYCLE_START_EXTI_IRQn EXTI9_5_IRQn
#define CONTROL_SAFETY_DOOR_Pin GPIO_PIN_9
#define CONTROL_SAFETY_DOOR_GPIO_Port GPIOC
#define CONTROL_SAFETY_DOOR_EXTI_IRQn EXTI9_5_IRQn
#define LD2_Pin GPIO_PIN_1
#define LD2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
