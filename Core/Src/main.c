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
#include "fonts.h"
#include "ssd1306.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRIG_PIN GPIO_PIN_12
#define TRIG_PORT GPIOA
#define ECHO_PIN GPIO_PIN_15
#define ECHO_PORT GPIOA
#define HISTORYSIZE  64
#define MAXDEPTH  400
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_Delay_us(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER(&htim1) < us);
}

uint16_t getDistance(){
	uint16_t distance = 0;
	 // ★★ ЗАПУСКАЮЩИЙ ИМПУЛЬС НА TRIG ★★
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
	HAL_Delay_us(2);
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
	HAL_Delay_us(10);
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

		      // Ждем начало Echo (низкий уровень)
	uint32_t timeout = HAL_GetTick();
	while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) && (HAL_GetTick() - timeout < 100));

		      // Ждем конец Echo (высокий уровень)
		timeout = HAL_GetTick();
		__HAL_TIM_SET_COUNTER(&htim1, 0);
		while (!HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) && (HAL_GetTick() - timeout < 100));

		if (HAL_GetTick() - timeout >= 100) {
		          distance = 55;  // Таймаут - все еще 55?
		} else {
		          // Измеряем длительность высокого уровня
		      uint32_t start_time = __HAL_TIM_GET_COUNTER(&htim1);
		      timeout = HAL_GetTick();
		      while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) && (HAL_GetTick() - timeout < 50));
		      uint32_t end_time = __HAL_TIM_GET_COUNTER(&htim1);

		      uint32_t pulse_duration = end_time - start_time;
		     distance = pulse_duration * 0.034 / 2;  // см

		          // Ограничение разумного диапазона
		     if (distance > 400) distance = 2;
		      }

	return distance;
}

void graphAxes(){
	SSD1306_DrawLine(0, 0, 127, 0, SSD1306_COLOR_WHITE);
	    SSD1306_DrawLine(0, 63, 127, 63, SSD1306_COLOR_WHITE);
	    SSD1306_DrawLine(2, 0, 2, 63, SSD1306_COLOR_WHITE);
	    //  Y
	    for(int y = 10; y < 63; y += 10){
	        SSD1306_DrawLine(0, y, 5, y, SSD1306_COLOR_WHITE);
	        int depth_m = y/10;
	        char depth_str[4];
	        sprintf(depth_str, "%d", depth_m);
	        SSD1306_GotoXY(8, y-3);
	        SSD1306_Puts(depth_str, &Font_7x10, SSD1306_COLOR_WHITE);
	    }

	    //  X
	    for (int x_mark = 0; x_mark < 128; x_mark += 10) {
	        SSD1306_DrawLine(x_mark, 60, x_mark, 63, SSD1306_COLOR_WHITE);
	        int distance_m = x_mark / 10;
	        if (distance_m <= 12) {
	            char dist_str[4];
	            sprintf(dist_str, "%d", distance_m);
	            SSD1306_GotoXY(x_mark-3, 50);
	            SSD1306_Puts(dist_str, &Font_7x10, SSD1306_COLOR_WHITE);
	        }
	    }
}

int scaleDepth(uint32_t depth_cm) {
    int pixels = (depth_cm * 50) / MAXDEPTH;
    if (pixels > 50) pixels = 50;
    return pixels+1;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */
  SSD1306_Init();
  HAL_TIM_Base_Start(&htim1);
  uint32_t historyData[HISTORYSIZE];
  uint32_t HistoryIndex = 0;
  uint32_t IndexStart = 0;
  uint32_t DataCount = 0;
  uint32_t data[20] = {15, 15, 25, 35, 28, 18, 10, 20, 30, 40,
                  32, 22, 12, 8, 16, 24, 34, 26, 18, 12};
  int x = 0;
  uint16_t distance = 0;
  /* USER CODE END 2 */

  while (1)
  {

	SSD1306_Clear();
	distance = getDistance();

	historyData[HistoryIndex] = distance;
	HistoryIndex ++;
	if(HistoryIndex >= HISTORYSIZE){
		HistoryIndex = 0;
	}

	if(DataCount < HISTORYSIZE){
		DataCount++;
	}
	for(int i = 0; i < DataCount -1; i++){
		uint32_t NowIndex = (HistoryIndex + i) % HISTORYSIZE;
		uint32_t NextIndex = (HistoryIndex + i + 1) % HISTORYSIZE;
		  int x0 = i * 2;
		  int y0 = scaleDepth(historyData[NowIndex]);;
		  int x1 = (i + 1) * 2;
		  int y1 = scaleDepth(historyData[NextIndex]);;
		  SSD1306_DrawLine(x0, y0, x1, y1, SSD1306_COLOR_WHITE);

	}

    char depth_str[10];
    sprintf(depth_str, "%d cm", distance);
    SSD1306_GotoXY(40, 0);
    SSD1306_Puts(depth_str, &Font_7x10, SSD1306_COLOR_WHITE);

    graphAxes();


//    for (int i = 0 + x; i < 19; i++) {
//        int x0 = i * 10 - x * 10;
//        int y0 = 63 - data[i];
//        int x1 = (i+1) * 10 - x * 10;
//        int y1 = 63 - data[i+1];
//        SSD1306_DrawLine(x0, y0, x1, y1, SSD1306_COLOR_WHITE);
//    }




    x++;
    SSD1306_UpdateScreen();
    HAL_Delay(100);

  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
  sClockSourceConfig.ClockFilter = 0;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA3 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
