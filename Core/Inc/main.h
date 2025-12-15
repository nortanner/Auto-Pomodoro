/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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
#define BUZZ_Pin GPIO_PIN_0
#define BUZZ_GPIO_Port GPIOC
#define B_MODE_Pin GPIO_PIN_1
#define B_MODE_GPIO_Port GPIOC
#define B_MODE_EXTI_IRQn EXTI1_IRQn
#define RST_Pin GPIO_PIN_0
#define RST_GPIO_Port GPIOA
#define DC_Pin GPIO_PIN_1
#define DC_GPIO_Port GPIOA
#define B_DOWN_Pin GPIO_PIN_4
#define B_DOWN_GPIO_Port GPIOA
#define B_DOWN_EXTI_IRQn EXTI4_IRQn
#define B_UP_Pin GPIO_PIN_0
#define B_UP_GPIO_Port GPIOB
#define B_UP_EXTI_IRQn EXTI0_IRQn
#define LED_B_Pin GPIO_PIN_6
#define LED_B_GPIO_Port GPIOC
#define LED_G_Pin GPIO_PIN_7
#define LED_G_GPIO_Port GPIOC
#define LED_R_Pin GPIO_PIN_8
#define LED_R_GPIO_Port GPIOC
#define US_ECHO_Pin GPIO_PIN_8
#define US_ECHO_GPIO_Port GPIOA
#define US_TRIG_Pin GPIO_PIN_9
#define US_TRIG_GPIO_Port GPIOA
#define MS_Pin GPIO_PIN_10
#define MS_GPIO_Port GPIOA
#define MS_EXTI_IRQn EXTI15_10_IRQn
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define CS_Pin GPIO_PIN_6
#define CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

// Display dimensions
#define NUMROWS 160
#define NUMCOLS 128

// Common 16-bit RGB colors: bbbbb gggggg rrrrr
#define BLACK 0x0000 // 00000 000000 00000
#define WHITE 0xFFFF // 11111 111111 11111 (red + green + blue)
#define RED 0x001F // 00000 000000 11111
#define GREEN 0x07E0 // 00000 111111 00000
#define BLUE 0xF800 // 11111 000000 00000
#define CYAN 0xFFE0 // 11111 111111 00000 (green + blue)
#define MAGENTA 0xF81F // 11111 000000 11111 (red + blue)
#define YELLOW 0x07FF // 00000 111111 11111 (red + green)
#define ORANGE 0x041F // 00000 100000 11111 (red + 50% green)
#define PURPLE 0x8010 // 10000 000000 10000 (50% red + 50% blue)
#define GRAY 0x8410 // 10000 100000 10000 (50% red + 50% green + 50% blue)
#define LIGHTGRAY 0xC618 // 11000 110000 11000 (75% red + 75% green + 75% blue)
#define DARKGRAY 0x4208 // 01000 010000 01000 (25% red + 25% green + 25% blue)

// Sensor constants
#define PRESENCE_DISTANCE_CM 80.0f  // Person must be within 80cm
#define MOTION_TIMEOUT_MS 30000       // 30 seconds without motion = no presence
#define US_MIN_WIDTH 100
#define US_MAX_WIDTH 25000
#define US_DISTANCE_CALC 0.01715f

// ST7735 command codes
#define SWRESET 0x01
#define SLPOUT 0x11
#define COLMOD 0x3A
#define MADCTL 0x36
#define DISPON 0x29
#define CASET 0x2A
#define RASET 0x2B
#define RAMWR 0x2C
#define COLOR_MODE_16BIT 0x05
#define MADCTL_DEFAULT 0xC8

// Duration constants
#define MIN_WORK_RESUME_TIME 5
#define LED_MAX_DUTY 0.99f
#define LED_MIN_DUTY 0.21f

typedef enum {
	US_IDLE, US_TRIGGERED, US_WAITING_FOR_ECHO
} UltrasonicState;

// LED breathing state machine
typedef enum {
    LED_IDLE,
    LED_BREATHING_UP,
    LED_BREATHING_DOWN
} LEDState;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
