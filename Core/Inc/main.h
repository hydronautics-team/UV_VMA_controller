/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DRDY_Pin GPIO_PIN_2
#define DRDY_GPIO_Port GPIOE
#define CS_I2C_SPI_Pin GPIO_PIN_3
#define CS_I2C_SPI_GPIO_Port GPIOE
#define MEMS_INT3_Pin GPIO_PIN_4
#define MEMS_INT3_GPIO_Port GPIOE
#define MEMS_INT4_Pin GPIO_PIN_5
#define MEMS_INT4_GPIO_Port GPIOE
#define OSC32_IN_Pin GPIO_PIN_14
#define OSC32_IN_GPIO_Port GPIOC
#define OSC32_OUT_Pin GPIO_PIN_15
#define OSC32_OUT_GPIO_Port GPIOC
#define OSC_IN_Pin GPIO_PIN_0
#define OSC_IN_GPIO_Port GPIOF
#define OSC_OUT_Pin GPIO_PIN_1
#define OSC_OUT_GPIO_Port GPIOF
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOA
#define VMA_2_Pin GPIO_PIN_1
#define VMA_2_GPIO_Port GPIOA
#define OSD_TX_Pin GPIO_PIN_2
#define OSD_TX_GPIO_Port GPIOA
#define OSD_RX_Pin GPIO_PIN_3
#define OSD_RX_GPIO_Port GPIOA
#define SPI1_SCK_Pin GPIO_PIN_5
#define SPI1_SCK_GPIO_Port GPIOA
#define SPI1_MISO_Pin GPIO_PIN_6
#define SPI1_MISO_GPIO_Port GPIOA
#define SPI1_MISOA7_Pin GPIO_PIN_7
#define SPI1_MISOA7_GPIO_Port GPIOA
#define VMA_7_Pin GPIO_PIN_0
#define VMA_7_GPIO_Port GPIOB
#define VMA_8_Pin GPIO_PIN_1
#define VMA_8_GPIO_Port GPIOB
#define LD4_Pin GPIO_PIN_8
#define LD4_GPIO_Port GPIOE
#define LD3_Pin GPIO_PIN_9
#define LD3_GPIO_Port GPIOE
#define LD7_Pin GPIO_PIN_11
#define LD7_GPIO_Port GPIOE
#define LD9_Pin GPIO_PIN_12
#define LD9_GPIO_Port GPIOE
#define LD10_Pin GPIO_PIN_13
#define LD10_GPIO_Port GPIOE
#define LD8_Pin GPIO_PIN_14
#define LD8_GPIO_Port GPIOE
#define LD6_Pin GPIO_PIN_15
#define LD6_GPIO_Port GPIOE
#define TX_485_Pin GPIO_PIN_10
#define TX_485_GPIO_Port GPIOB
#define RX_485_Pin GPIO_PIN_11
#define RX_485_GPIO_Port GPIOB
#define GRAB_DIR_Pin GPIO_PIN_13
#define GRAB_DIR_GPIO_Port GPIOB
#define ROT_DIR_Pin GPIO_PIN_15
#define ROT_DIR_GPIO_Port GPIOB
#define ROT_ENABLE_Pin GPIO_PIN_12
#define ROT_ENABLE_GPIO_Port GPIOD
#define GRAB_ENABLE_Pin GPIO_PIN_13
#define GRAB_ENABLE_GPIO_Port GPIOD
#define MAN_EN1_Pin GPIO_PIN_14
#define MAN_EN1_GPIO_Port GPIOD
#define SERVO_PWM_TILT_Pin GPIO_PIN_6
#define SERVO_PWM_TILT_GPIO_Port GPIOC
#define VMA_6_Pin GPIO_PIN_7
#define VMA_6_GPIO_Port GPIOC
#define POWER_ON_Pin GPIO_PIN_8
#define POWER_ON_GPIO_Port GPIOC
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define DE_485_Pin GPIO_PIN_6
#define DE_485_GPIO_Port GPIOF
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define IMU_TX_Pin GPIO_PIN_12
#define IMU_TX_GPIO_Port GPIOC
#define CS_SPI3_Pin GPIO_PIN_0
#define CS_SPI3_GPIO_Port GPIOD
#define IMU_RX_Pin GPIO_PIN_2
#define IMU_RX_GPIO_Port GPIOD
#define VMA_1_Pin GPIO_PIN_3
#define VMA_1_GPIO_Port GPIOD
#define VMA_4_Pin GPIO_PIN_6
#define VMA_4_GPIO_Port GPIOD
#define VMA_3_Pin GPIO_PIN_7
#define VMA_3_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define VMA_5_Pin GPIO_PIN_4
#define VMA_5_GPIO_Port GPIOB
#define I2C1_SCL_Pin GPIO_PIN_6
#define I2C1_SCL_GPIO_Port GPIOB
#define I2C1_SDA_Pin GPIO_PIN_7
#define I2C1_SDA_GPIO_Port GPIOB
#define SERVO_PWM_MAN_Pin GPIO_PIN_8
#define SERVO_PWM_MAN_GPIO_Port GPIOB
#define MEMS_INT1_Pin GPIO_PIN_0
#define MEMS_INT1_GPIO_Port GPIOE
#define MEMS_INT2_Pin GPIO_PIN_1
#define MEMS_INT2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
