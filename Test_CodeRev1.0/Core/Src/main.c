/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define CAMERA_TRIGGER_PIN  GPIO_PIN_13
#define LASER_TRIGGER_PIN   GPIO_PIN_8
#define T1_TRIGGER_PIN      GPIO_PIN_12
#define T2_TRIGGER_PIN      GPIO_PIN_14


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void delay_us (uint32_t us);

uint8_t g_ucTrigger1Selectflag;
uint8_t g_ucTrigger2Selectflag;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

//	uint8_t State = 0;
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
  MX_NVIC_Init();
  MX_TIM1_Init();
  HAL_TIM_Base_Start(&htim1);
  /* Initialize interrupts */
//  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

//  State = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);
  //Trigger Camera
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
//  delay_us(100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
//	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
//	  HAL_Delay(500);
//	  delay_us(1);


    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure. */
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
//  /** Initializes the CPU, AHB and APB buses clocks
//  */
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 6;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CAMERA_TRIGGER_PIN, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LASER_TRIGGER_PIN, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);


  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);



  /*Configure GPIO pins : PB12 PB14 */
  GPIO_InitStruct.Pin = T1_TRIGGER_PIN|T2_TRIGGER_PIN;
//  GPIO_InitStruct.Pin = T1_TRIGGER_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = CAMERA_TRIGGER_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */


   GPIO_InitStruct.Pin = LASER_TRIGGER_PIN |GPIO_PIN_2;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);



//     GPIO_InitStruct.Pin = GPIO_PIN_15;
//     GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//     HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void delay_us (uint32_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);

  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */

//  HAL_GPIO_TogglePin(GPIOA, LASER_TRIGGER_PIN);
//  delay_us(1);
//  HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);

//	if(GPIO_Pin == T1_TRIGGER_PIN)
//	{

//		 HAL_GPIO_WritePin(GPIOB, CAMERA_TRIGGER_PIN, GPIO_PIN_SET);   // LASER ON
//		 delay_us(1);
//		 HAL_GPIO_WritePin(GPIOB, CAMERA_TRIGGER_PIN, GPIO_PIN_RESET);
//		 delay_us(1);
//	}
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,GPIO_PIN_SET);

//  if(GPIO_Pin == T1_TRIGGER_PIN)
//  {
//	  g_ucTrigger1Selectflag = 1;
//  }
//  else
//  {
//	  g_ucTrigger2Selectflag = 1;
//  }
//
//	switch(GPIO_Pin)
//	{
//	  case T1_TRIGGER_PIN:
//
//		  if(g_ucTrigger1Selectflag && (!g_ucTrigger2Selectflag))
//		  {
//			  HAL_GPIO_WritePin(GPIOA, LASER_TRIGGER_PIN, GPIO_PIN_RESET);   // LASER ON
//			  delay_us(1);
//
//		  }
//		  else if(g_ucTrigger1Selectflag && g_ucTrigger2Selectflag)
//		  {
//			  g_ucTrigger1Selectflag = 0;
//			  g_ucTrigger2Selectflag = 0;
//			  HAL_GPIO_WritePin(GPIOB, CAMERA_TRIGGER_PIN, GPIO_PIN_SET);   // CAMERA ON
//			  delay_us(100);											    // 100us Delay
//			  HAL_GPIO_WritePin(GPIOA, LASER_TRIGGER_PIN, GPIO_PIN_SET);    // LASER OFF
//			  HAL_GPIO_WritePin(GPIOB, CAMERA_TRIGGER_PIN, GPIO_PIN_RESET); // CAMERA OFF
//
//		  }
//		  break;
//
//	  case T2_TRIGGER_PIN:
//		  if(g_ucTrigger1Selectflag && g_ucTrigger2Selectflag)
//		  {
//			  g_ucTrigger1Selectflag = 0;
//			  g_ucTrigger2Selectflag = 0;
//			  HAL_GPIO_WritePin(GPIOB, CAMERA_TRIGGER_PIN, GPIO_PIN_SET);   // CAMERA ON
//			  delay_us(100);											    // 100us Delay
//			  HAL_GPIO_WritePin(GPIOA, LASER_TRIGGER_PIN, GPIO_PIN_SET);    // LASER OFF
//			  HAL_GPIO_WritePin(GPIOB, CAMERA_TRIGGER_PIN, GPIO_PIN_RESET); // CAMERA OFF
//
//		  }
//		  else if((!g_ucTrigger1Selectflag) && g_ucTrigger2Selectflag)
//		  {
//			  HAL_GPIO_WritePin(GPIOA, LASER_TRIGGER_PIN, GPIO_PIN_RESET);   // LASER ON
//			  delay_us(1);
//		  }
//		  break;
//
//	}








//  if(GPIO_Pin ==T1_TRIGGER_PIN)
//  {
//	  HAL_GPIO_WritePin(GPIOA, LASER_TRIGGER_PIN, GPIO_PIN_RESET);   //LASER ON
//	  delay_us(1);
//
//  }
//  else
//  {
//	  HAL_GPIO_WritePin(GPIOB, CAMERA_TRIGGER_PIN, GPIO_PIN_SET);   // CAMERA ON
//	  delay_us(50);													// 50us Delay
//	  HAL_GPIO_WritePin(GPIOA, LASER_TRIGGER_PIN, GPIO_PIN_SET);    //LASER OFF
//	  HAL_GPIO_WritePin(GPIOB, CAMERA_TRIGGER_PIN, GPIO_PIN_RESET); //CAMERA OFF
//
//  }


}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
