/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include "font.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

volatile uint32_t ic_rising = 0;
volatile uint32_t ic_falling = 0;
volatile uint8_t ic_flag = 0;
volatile uint8_t ic_state = 0; // 0 = waiting for rising, 1 = waiting for falling

volatile float distance_cm = 0.0f;

volatile UltrasonicState us_state = US_IDLE;
volatile uint32_t us_timeout_start = 0;

volatile LEDState led_state = LED_IDLE;
volatile float led_duty_cycle = 0.2f;  // Current duty cycle
uint32_t PWM_PERIOD = 999;  // Cache this value

volatile uint32_t last_motion_time = 0;

volatile uint8_t working = 0;	// 0 = not working, 1 = working
volatile uint8_t taking_break = 0;// 0 = not taking a break, 1 = taking a break

uint8_t edit_period = 0;// Make changes to timer durations 1 = working, 2 = break

// Default timer periods in minutes
uint32_t working_period = 20;
uint32_t break_period = 5;

// Current timer period in seconds
uint32_t current_period = 1200;

uint16_t cursor_x = 0; // start at left
uint16_t cursor_y = 0; // start at top

// Display framebuffer - 128x160 pixels at 16-bit color (RGB565)
uint16_t framebuffer[NUMROWS][NUMCOLS];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

// Display Prototypes
void ST7735_WriteData(uint8_t data[], uint16_t size);
void ST7735_WriteCommand(uint8_t cmd);

void ST7735_Init(void);

void ST7735_SetAddressWindow(uint16_t xStart, uint16_t yStart, uint16_t xEnd,
		uint16_t yEnd);

void ST7735_DrawPixel(uint8_t x, uint8_t y, uint16_t color);

void ST7735_DrawScreen(uint16_t frame[NUMROWS][NUMCOLS]);

void ST7735_UpdateCharCursor(uint16_t x, uint16_t y);
void ST7735_PrintChar(char character);
void ST7735_PrintString(char *string);

void RenderCountdownDisplay(uint16_t minutes, uint16_t seconds, float progress);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_SPI1_Init();
	MX_TIM3_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM4_Init();
	/* USER CODE BEGIN 2 */

	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);	// Start Timer 1

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);// start PWM on Timer3’s Channel 1
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);// start PWM on Timer3’s Channel 2
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);// start PWM on Timer3’s Channel 3

	HAL_TIM_Base_Start_IT(&htim2);	// Start Timer 2 with interrupts
	HAL_TIM_Base_Start_IT(&htim3);	// Start Timer 3 with interrupts
	HAL_TIM_Base_Start_IT(&htim4);	// Start Timer 4 with interrupts

	ST7735_Init();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		if (edit_period == 0) {
			RenderCountdownDisplay(((int) (current_period / 60)),
					(current_period % 60),
					current_period
							/ (taking_break ?
									(float) (break_period * 60) :
									(float) (working_period * 60)));

		} else if (edit_period == 1) {
			RenderCountdownDisplay(working_period, 0, 1);
		} else {
			RenderCountdownDisplay(break_period, 0, 1);
		}

		HAL_Delay(1);

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 10;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_IC_InitTypeDef sConfigIC = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 79;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65534;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_IC_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 7999;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 999;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 79;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 999;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 7999;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 9999;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, RST_Pin | DC_Pin | US_TRIG_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : BUZZ_Pin */
	GPIO_InitStruct.Pin = BUZZ_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BUZZ_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : B_MODE_Pin */
	GPIO_InitStruct.Pin = B_MODE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(B_MODE_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : RST_Pin DC_Pin US_TRIG_Pin */
	GPIO_InitStruct.Pin = RST_Pin | DC_Pin | US_TRIG_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : B_DOWN_Pin */
	GPIO_InitStruct.Pin = B_DOWN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(B_DOWN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : B_UP_Pin */
	GPIO_InitStruct.Pin = B_UP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(B_UP_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : MS_Pin */
	GPIO_InitStruct.Pin = MS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(MS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : CS_Pin */
	GPIO_InitStruct.Pin = CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief Transmits the provided bytes as data to the ST7735 over SPI1
 * @param data: the array of data / pointer to where the data starts
 * @param size: the number of bytes of data to transit
 * @retval None
 */
void Delay_us(uint32_t us) {
	uint32_t start = DWT->CYCCNT;
	uint32_t cycles = us * (SystemCoreClock / 1000000);

	while ((DWT->CYCCNT - start) < cycles) {
		// busy wait
	}
}

float median_filter(float new_value) {
	static float buffer[3] = { 0 };  // Store last 3 readings
	static uint8_t index = 0;

	// Add new value to buffer
	buffer[index] = new_value;
	index = (index + 1) % 3;

	// Copy buffer for sorting (don't modify original)
	float sorted[3];
	for (int i = 0; i < 3; i++) {
		sorted[i] = buffer[i];
	}

	// Simple bubble sort
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 2 - i; j++) {
			if (sorted[j] > sorted[j + 1]) {
				float temp = sorted[j];
				sorted[j] = sorted[j + 1];
				sorted[j + 1] = temp;
			}
		}
	}

	// Return median
	return sorted[1];
}

/**
 * @brief Clears the entire framebuffer to a single color
 */
void FB_Clear(uint16_t color) {
	for (int y = 0; y < NUMROWS; y++) {
		for (int x = 0; x < NUMCOLS; x++) {
			framebuffer[y][x] = color;
		}
	}
}

/**
 * @brief Draws a pixel into the framebuffer with bounds checking
 */
void FB_DrawPixel(int x, int y, uint16_t color) {
	if (x < 0 || x >= NUMCOLS || y < 0 || y >= NUMROWS) {
		return;
	}
	framebuffer[y][x] = color;
}

/**
 * @brief Draws a countdown arc showing progress (optimized)
 */
void FB_DrawCountdownArc(int cx, int cy, int radius, int thickness,
		float progress, uint16_t color) {
	if (progress < 0.0f)
		progress = 0.0f;
	if (progress > 1.0f)
		progress = 1.0f;

	// Pre-calculate squared distances to avoid sqrt()
	int inner_radius = radius - thickness;
	int inner_sq = inner_radius * inner_radius;
	int outer_sq = radius * radius;

	// Draw arc by checking each pixel
	for (int y = cy - radius - thickness; y <= cy + radius + thickness; y++) {
		for (int x = cx - radius - thickness; x <= cx + radius + thickness;
				x++) {
			int dx = x - cx;
			int dy = y - cy;
			int dist_sq = dx * dx + dy * dy;

			// Check if pixel is within the ring (no sqrt needed!)
			if (dist_sq >= inner_sq && dist_sq <= outer_sq) {
				// Calculate angle (-π to π)
				float angle = atan2f(dy, dx);
				// Normalize to 0-360 starting from top
				float angle_deg = (angle * 180.0f / 3.14159f) + 90.0f;
				if (angle_deg < 0)
					angle_deg += 360.0f;

				// Check if within progress arc
				if (angle_deg >= (1.0f - progress) * 360.0f) {
					FB_DrawPixel(x, y, color);
				}
			}
		}
	}
}

/**
 * @brief Draws a character into framebuffer
 */
void FB_DrawChar(int x, int y, char character, uint16_t fg_color,
		uint16_t bg_color) {
	if (character < ' ' || character > '~')
		return;

	int fontIndex = character - 0x20;
	const uint8_t *characterData = fontTable[fontIndex];

	for (int row = 0; row < 8; row++) {
		uint8_t rowData = characterData[row];
		for (int col = 7; col >= 0; col--) {
			int px = x + (7 - col);
			int py = y + row;

			if ((rowData & (1 << col)) != 0) {
				FB_DrawPixel(px, py, fg_color);
			} else if (bg_color != 0xFFFF) {
				FB_DrawPixel(px, py, bg_color);
			}
		}
	}
}

/**
 * @brief Draws centered string
 */
void FB_DrawStringCentered(int cx, int cy, char *text, uint16_t fg_color,
		uint16_t bg_color) {
	int len = 0;
	while (text[len] != '\0')
		len++;

	int total_width = len * 8;
	int start_x = cx - (total_width / 2);
	int start_y = cy - 4;

	for (int i = 0; i < len; i++) {
		FB_DrawChar(start_x + (i * 8), start_y, text[i], fg_color, bg_color);
	}
}

/**
 * @brief Complete countdown display
 */
void RenderCountdownDisplay(uint16_t minutes, uint16_t seconds, float progress) {
	const int cx = 64;  // Center X
	const int cy = 80;  // Center Y
	const int radius = 50;
	const int thickness = 8;

	uint16_t ring_color;

	if (edit_period == 0) {
		ring_color =
				taking_break ?
						(((PURPLE) >> 8) & 0x00FF)
								| (((PURPLE) & 0x00FF) << 8) :
						(((YELLOW) >> 8) & 0x00FF) | (((YELLOW) & 0x00FF) << 8); // Purple or Yellow

	} else if (edit_period == 1) {
		ring_color = (((YELLOW) >> 8) & 0x00FF) | (((YELLOW) & 0x00FF) << 8);
	} else {
		ring_color = (((PURPLE) >> 8) & 0x00FF) | (((PURPLE) & 0x00FF) << 8);
	}

	// Clear framebuffer
	FB_Clear(BLACK);

	// Draw countdown arc
	FB_DrawCountdownArc(cx, cy, radius, thickness, progress, ring_color);

	// Format time
	char time_str[6];
	time_str[0] = (minutes / 10) + '0';
	time_str[1] = (minutes % 10) + '0';
	time_str[2] = ':';
	time_str[3] = (seconds / 10) + '0';
	time_str[4] = (seconds % 10) + '0';
	time_str[5] = '\0';

	// Draw text
	FB_DrawStringCentered(cx, cy, time_str, WHITE, 0xFFFF);

	if (edit_period == 0) {
		FB_DrawStringCentered(cx, cy + 12, taking_break ? "BREAK" : "WORK",
		WHITE, 0xFFFF);

	} else if (edit_period == 1) {
		FB_DrawStringCentered(cx, cy + 12, "WORK", WHITE, 0xFFFF);
	} else {
		FB_DrawStringCentered(cx, cy + 12, "BREAK", WHITE, 0xFFFF);
	}

	// Push to display using DrawScreen()
	ST7735_DrawScreen(framebuffer);
}

void ST7735_WriteData(uint8_t data[], uint16_t size) {
// Change the DC (Data/Command) pin to "Data" mode
	HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_SET);

// Send the data
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); // select the device
	HAL_SPI_Transmit(&hspi1, data, size, HAL_MAX_DELAY); // transmit bytes
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); // deselect the device
}

/**
 * @brief Transmits the provided byte as a command to the ST7735 over SPI1
 * @param cmd: the 1-byte command to transmit
 * @retval None
 */
void ST7735_WriteCommand(uint8_t cmd) {
// Change DC (Data/Command) pin to "Command" mode
	HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_RESET);
// Send the command
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); // select the device
	HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY); // transmit byte
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); // deselect the device
}

/**
 * @brief Initializes the ST7735 over SPI1 with startup commands
 * @param None
 * @retval None
 */
void ST7735_Init(void) {
// Perform a hardware reset (reset low pulse)
	HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET); // RST pin low
	HAL_Delay(5); // wait 5ms
	HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET); // RST pin high
	HAL_Delay(5); // wait 5ms
// Perform a software reset
	ST7735_WriteCommand(SWRESET); // send a SWRESET (Software reset) command
	HAL_Delay(150);
// Wake the display (exit low-power sleep mode)
	ST7735_WriteCommand(SLPOUT); // Send a SLPOUT (Sleep out) command
	HAL_Delay(150);
// Enter "Interface Pixel Format" mode and set to 16-bit color
	ST7735_WriteCommand(COLMOD); // Send a COLMOD (Color Mode) command
	uint8_t colorMode = 0x05; // color mode = 16-bit/pixel
	ST7735_WriteData(&colorMode, 1); // send color mode value as a data message
// Enter "Memory Data Access Control" mode and set to row/column order
	ST7735_WriteCommand(MADCTL); // Send a MADCTL (Memory Access Data Control) command
	uint8_t accessMode = 0xC8; // access mode = row/column order
	ST7735_WriteData(&accessMode, 1); // send access mode value as a data message
// Turn on display
	ST7735_WriteCommand(DISPON); // send a DISPON (Display On) command
	HAL_Delay(10);
}

/**
 * @brief Updates a specified pixel to a specified color
 * @param x: the x address of the pixel
 * @param y: the y address of the pixel
 * @param color: two-byte color value (bbbbb gggggg rrrrr)
 * @retval None
 */
void ST7735_DrawPixel(uint8_t x, uint8_t y, uint16_t color) {
// Early exit if (x,y) is out of bounds
	if ((x >= NUMCOLS) || (y >= NUMROWS))
		return;
// Single pixel, so start and stop on the same (x,y) pixel
	ST7735_SetAddressWindow(x, y, x, y);
// Organize the color as a two-byte value
	uint8_t pixelData[2] = { color >> 8, color & 0xFF }; // swap for Big Endian
// Send the data
	ST7735_WriteData(pixelData, 2);
}

/**
 * @brief Specifies the address window where the next pixel data will go
 * @param xStart: the starting x address
 * @param yStart: the starting y address
 * @param xEnd: the ending x address
 * @param yEnd: the ending y address
 * @retval None
 */
void ST7735_SetAddressWindow(uint16_t xStart, uint16_t yStart, uint16_t xEnd,
		uint16_t yEnd) {
// Early exit if any dimensions are out of bounds
	if ((xStart > xEnd) || (yStart > yEnd) || (xEnd >= NUMCOLS)
			|| (yEnd >= NUMROWS))
		return;
// Data array for configuration data
	uint8_t address[4];
// Set column start/end
	ST7735_WriteCommand(0x2A); // Enter "Column address set" mode
	address[0] = xStart >> 8; // upper 8 bits of x starting address
	address[1] = xStart & 0xFF; // lower 8 bits of x starting address
	address[2] = xEnd >> 8; // upper 8 bits of x ending address
	address[3] = xEnd & 0xFF; // lower 8 bits of x ending address
	ST7735_WriteData(address, 4); // Transmit the 4 bytes of parameter data
// Set row start/end
	ST7735_WriteCommand(0x2B); // Enter "Row address set" mode
	address[0] = yStart >> 8; // upper 8 bits of y starting address
	address[1] = yStart & 0xFF; // lower 8 bits of y starting address
	address[2] = yEnd >> 8; // upper 8 bits of y ending address
	address[3] = yEnd & 0xFF; // lower 8 bits of y ending address
	ST7735_WriteData(address, 4); // Transmit the 4 bytes of parameter data
// Get ready to send pixel data
	ST7735_WriteCommand(0x2C); // Enter "RAM Write" mode (pixel writing mode)
}

/**
 * @brief Redraws the entire ST7735 display with provided frame
 * @param frame: the 2-dimensional array of 16-bit pixel colors
 * @retval None
 */
void ST7735_DrawScreen(uint16_t frame[NUMROWS][NUMCOLS]) {
	// Select entire display
	ST7735_SetAddressWindow(0, 0, NUMCOLS - 1, NUMROWS - 1);
	// Transmit entire frame to display
	ST7735_WriteData((uint8_t*) frame, NUMROWS * NUMCOLS * 2);
}

/**
 * @brief Updated the character cursor
 * @param x: the desired x coordinate (0 to 127)
 * @param y: the desired y coordinate (0 to 159)
 * @retval 0: success, 1: failure
 */
void ST7735_UpdateCharCursor(uint16_t x, uint16_t y) {
// Update
	cursor_x = x;
	cursor_y = y;
// Perform any necessary wrap-around
	if (cursor_x > (NUMCOLS - 8)) { // cursor goes off the right
		cursor_x = 0; // wrap back to left
		cursor_y += 8; // move cursor down one row
	}
	if (cursor_y > (NUMROWS - 9)) { // cursor goes off the bottom
		cursor_x = 0;
		cursor_y = 0; // wrap back to top
	}
}

/**
 * @brief Draws an 8x8 character to the ST7735 (black on white) using the cursor
 * @param character: the ASCII value of the character to print (' ' through '~')
 * @retval None
 */
void ST7735_PrintChar(char character) {
	// Early exit if character is not supported
	if ((character < ' ') || (character > '~'))
		return;
	// Realign ASCII character to start of font table
	int fontIndex = character - 0x20; // 0x20 is where printable characters begin in ASCII
	// Get character data from font table
	const uint8_t *characterData = fontTable[fontIndex];
	// Set address window for 8x8 character
	ST7735_SetAddressWindow(cursor_x, cursor_y, cursor_x + 7, cursor_y + 7);
	// Process each row of the character, one at a time
	for (int row = 0; row < 8; row++) {
		// Get the data for the current row from the character array
		uint8_t rowData = characterData[row];
		// Process each pixel in the row (left to right, msb to lsb)
		for (int col = 7; col >= 0; col--) {
			// See if that pixel is black (1) or white (0)
			uint16_t color;
			if ((rowData & (1 << col)) == 0) { // bitmask one bit at a time
				color = WHITE;
			} else {
				color = BLACK;
			}
			// Transmit pixel data
			uint8_t pixelData[2] = { color >> 8, color & 0xFF }; // swap for Big Endian
			ST7735_WriteData(pixelData, 2); // send the data
		}
	}
	// Move cursor to next character window
	ST7735_UpdateCharCursor(cursor_x + 8, cursor_y);
}

/**
 * @brief Draws the characters (A-Z) from string to display
 * @param x: the x coordinate of the top-left corner (0 to NUMCOLS-8)
 * @param y: the y coordinate of the top-left corner (0 to NUMROWS-8)
 * @param string: the character array
 * @retval None
 */
void ST7735_PrintString(char *string) {
	// Print each character, one at a time
	for (int i = 0; string[i] != '\0'; i++) {
		ST7735_PrintChar(string[i]);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		// TIM2 fires every 100ms for ultrasonic measurements

		switch (us_state) {
		case US_IDLE:
			// Start new measurement
			ic_flag = 0;
			ic_state = 0;

			// Ensure we're ready for rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1,
					TIM_INPUTCHANNELPOLARITY_RISING);

			// Send 10 µs trigger pulse
			HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_SET);
			Delay_us(10);
			HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_RESET);

			us_state = US_WAITING_FOR_ECHO;
			us_timeout_start = HAL_GetTick();
			break;

		case US_WAITING_FOR_ECHO:
			// Check if measurement completed
			if (ic_flag) {
				ic_flag = 0;

				uint32_t width;
				if (ic_falling >= ic_rising)
					width = ic_falling - ic_rising;
				else
					width = (htim1.Init.Period - ic_rising) + ic_falling;

				// Check the width of the pulse for a reasonable range
				if (width > US_MIN_WIDTH && width < US_MAX_WIDTH) {
					float raw_distance = width * US_DISTANCE_CALC; // Calculate the distance of the measurement
					distance_cm = median_filter(raw_distance);
				} else {
					distance_cm = -1.0f;  // Out of range
				}

				us_state = US_IDLE;
			}
			// Check for timeout (100ms)
			else if ((HAL_GetTick() - us_timeout_start) > 100) {
				distance_cm = -2.0f;  // Timeout
				us_state = US_IDLE;
			}
			break;

		default:
			us_state = US_IDLE;
			break;
		}

		uint32_t time_since_motion = HAL_GetTick() - last_motion_time;

		if ((distance_cm > 0) && (distance_cm < PRESENCE_DISTANCE_CM)
				&& (time_since_motion < MOTION_TIMEOUT_MS)
				&& (edit_period == 0)) {
			working = 1;  // Person is present
		} else if ((time_since_motion > MOTION_TIMEOUT_MS)
				&& (taking_break == 0)) {
			working = 0;  // No person detected or timeout
			if ((current_period / 60) < MIN_WORK_RESUME_TIME) {	// Reset to full work period if less than 5 minutes left
				current_period = working_period * 60;// Working period in seconds
			}
		}
	}

	// If person is present, the system starts
	// TIM3: LED breathing effect (fires every 1ms with PWM period)
	if (htim->Instance == TIM3) {

		if (working) {
			// Breathing effect when person is present
			if (led_state == LED_IDLE) {
				led_state = LED_BREATHING_UP;
			}

			if (led_state == LED_BREATHING_UP) {
				led_duty_cycle += 0.0016f;  // Increment
				if (led_duty_cycle >= LED_MAX_DUTY) { // Tolerance to account for floating point rounding errors
					led_duty_cycle = 1.0f;
					led_state = LED_BREATHING_DOWN;
				}
			}

			if (led_state == LED_BREATHING_DOWN) {
				led_duty_cycle -= 0.0016f;  // Decrement
				if (led_duty_cycle <= LED_MIN_DUTY) { // Tolerance to account for floating point rounding errors
					led_duty_cycle = 0.2f;
					led_state = LED_BREATHING_UP;
				}
			}

			// Update PWM based on duty cycle
			int resetValue = (int) (led_duty_cycle * PWM_PERIOD);
			TIM3->CCR3 = resetValue;  // Red

			if (taking_break) {  // Purple
				TIM3->CCR2 = 0;
				TIM3->CCR1 = resetValue;
			} else {  // Amber
				TIM3->CCR2 = (int) (resetValue * 0.25f);
				TIM3->CCR1 = 0;
			}

		} else {
			// Solid colors when !working
			led_state = LED_IDLE;
			led_duty_cycle = 0.2f;  // Reset

			if (edit_period == 1) {	// Edit working_period duration
				// Solid amber
				TIM3->CCR3 = 1000;
				TIM3->CCR2 = 250;
				TIM3->CCR1 = 0;
			} else if (edit_period == 2) {	// Edit break_period duration
				// Solid purple
				TIM3->CCR3 = 1000;
				TIM3->CCR2 = 0;
				TIM3->CCR1 = 1000;
			} else {	// System idle
				// Dim red
				TIM3->CCR1 = 0;
				TIM3->CCR2 = 0;
				TIM3->CCR3 = 200;
			}

		}

	}

	// TIM4: Pomodoro countdown (every 1 second)
	if (htim->Instance == TIM4) {
		HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);// Turn off buzzer
		// Only countdown when person is present
		if (working) {
			if (current_period > 0) {
				current_period--;  // Decrement 1 second
				// Did we just hit zero?
				if (current_period == 0) {
					// Timer expired! Switch between work and break
					if (taking_break) {
						// Break finished → Start work
						taking_break = 0;
						current_period = working_period * 60;
					} else {
						// Work finished → Start break
						taking_break = 1;
						current_period = break_period * 60;
					}

					// Beep once when switching periods
					HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
				}
			}
		}
		// else: paused → keep current_period frozen, buzzer already off
	}

}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {

	// Find pulse width of the Ultrasonic Distance Sensor
	if (htim->Instance == TIM1) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
			if (ic_state == 0) {
				// RISING edge (start of echo pulse)
				ic_rising = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

				// Switch to capture FALLING edge next
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,
						TIM_INPUTCHANNELPOLARITY_FALLING);
				ic_state = 1;
			} else {
				// FALLING edge (end of echo pulse)
				ic_falling = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

				// Switch back to capture RISING edge for next measurement
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,
						TIM_INPUTCHANNELPOLARITY_RISING);
				ic_state = 0;

				// Set flag indicating measurement is complete
				ic_flag = 1;
			}
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == MS_Pin) {
		last_motion_time = HAL_GetTick();
	}

	// Register button presses for debouncing
	static uint32_t last_press = 0;
	uint32_t now = HAL_GetTick();

	if (GPIO_Pin == B_MODE_Pin) {

		if (now - last_press < 250)
			return;  // 250ms debounce
		last_press = now;

		if (edit_period < 2) {
			edit_period += 1;
			working = 0;
		} else {
			edit_period = 0;

			// Restore correct period based on current state
			current_period =
					taking_break ? (break_period * 60) : (working_period * 60);
		}

	}

	if (GPIO_Pin == B_UP_Pin) {

		if (now - last_press < 250)
			return;  // 250ms debounce
		last_press = now;

		switch (edit_period) {
		case 1:	// Increase work duration
			working_period += 1;
			break;
		case 2:	// Increase break duration
			break_period += 1;
			break;
		case 0:	// No action
			break;
		}

	}

	if (GPIO_Pin == B_DOWN_Pin) {

		if (now - last_press < 250)
			return;  // 250ms debounce
		last_press = now;

		switch (edit_period) {
		case 1:	// Decrease work duration if working_period > 0
			if (working_period > 1) {
				working_period -= 1;
			}
			break;
		case 2:	// Decrease break duration  if break_period > 0
			if (break_period > 1) {
				break_period -= 1;
			}
			break;
		case 0:	// No action
			break;
		}

	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
