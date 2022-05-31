/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void createPhoto();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Cam_Btn_Pin GPIO_PIN_0
#define Cam_Btn_GPIO_Port GPIOA
#define Cam_Btn_EXTI_IRQn EXTI0_IRQn
#define SPI_CS_Pin GPIO_PIN_4
#define SPI_CS_GPIO_Port GPIOC
#define CAMERA_RESET_Pin GPIO_PIN_5
#define CAMERA_RESET_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */
//#define IMAGE_SIZE 10560  // 176 * 120 / 2
//#define IMAGE_SIZE 10440  // 174 * 120 / 2
//#define IMAGE_SIZE 12672  // 176 * 144 / 2
//#define IMAGE_SIZE 12398  // 171 * 144 / 2
#define IMAGE_SIZE 24795  // 171 * 145 / 2
//#define IMAGE_SIZE 23040  // 320 * 144 / 2
//#define IMAGE_SIZE 12528  // 174 * 144 / 2
/* USER CODE END Private defines */


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
