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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
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
UART_HandleTypeDef huart2;

osThreadId SenderTask1Handle;
osThreadId ReceiverTaskHandle;
osThreadId SenderTask2Handle;
/* USER CODE BEGIN PV */
QueueHandle_t xQueue1 = NULL, xQueue2 = NULL;
QueueSetHandle_t xQueueSet = NULL;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void Sender1Function(void const * argument);
void ReceiveFunction(void const * argument);
void Sender2Function(void const * argument);

/* USER CODE BEGIN PFP */
void UartPrint(char* message);
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */

  xQueue1 = xQueueCreate( 1, sizeof( char * ) );
  xQueue2 = xQueueCreate( 1, sizeof( char * ) );

  // Alttaki 3 fonksiyonun kullanilmasi icin FreeRTOSConfig.h icerisinde configUSE_QUEUE_SETS 1 olarak ayarlanmalidir.
  xQueueSet = xQueueCreateSet( 1 * 2 );

  xQueueAddToSet( xQueue1, xQueueSet );
  xQueueAddToSet( xQueue2, xQueueSet );

  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of SenderTask1 */
  osThreadDef(SenderTask1, Sender1Function, osPriorityNormal, 0, 128);
  SenderTask1Handle = osThreadCreate(osThread(SenderTask1), NULL);

  /* definition and creation of ReceiverTask */
  osThreadDef(ReceiverTask, ReceiveFunction, osPriorityAboveNormal, 0, 128);
  ReceiverTaskHandle = osThreadCreate(osThread(ReceiverTask), NULL);

  /* definition and creation of SenderTask2 */
  osThreadDef(SenderTask2, Sender2Function, osPriorityNormal, 0, 128);
  SenderTask2Handle = osThreadCreate(osThread(SenderTask2), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USER_LED_G_Pin|USER_LED_R_Pin|USER_LED_RD14_Pin|USER_LED_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BTN_Pin */
  GPIO_InitStruct.Pin = USER_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USER_LED_G_Pin USER_LED_R_Pin USER_LED_RD14_Pin USER_LED_B_Pin */
  GPIO_InitStruct.Pin = USER_LED_G_Pin|USER_LED_R_Pin|USER_LED_RD14_Pin|USER_LED_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void UartPrint(char* message)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 20);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_Sender1Function */
/**
  * @brief  Function implementing the SenderTask1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Sender1Function */
void Sender1Function(void const * argument)
{
  /* USER CODE BEGIN 5 */

	const TickType_t xBlockTime = pdMS_TO_TICKS( 1000 );
	const char * const pcMessage = "Message from SenderTask1\r\n";

  /* Infinite loop */
  for(;;)
  {
	  vTaskDelay( xBlockTime );
	  xQueueSend( xQueue1, &pcMessage, 0 );
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_ReceiveFunction */
/**
* @brief Function implementing the ReceiverTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ReceiveFunction */
void ReceiveFunction(void const * argument)
{
  /* USER CODE BEGIN ReceiveFunction */

	// QueueHandle_t xQueueThatContainsData;

	QueueSetMemberHandle_t xQueueSetMember;
	char *pcReceivedString;

  /* Infinite loop */
  for(;;)
  {
	  // QueueSet donusunun Timeout suresi sonsuz olarak ayarlanmistir. Bu sebeple hicbir zaman NULL degeri donmeyecek.
	  // Fakat Timeout suresini kisaltirsak 500ms icerisinde veri gelmezse xQueueSelectFromSet() fonk. donusu NULL olacaktir..
//	  xQueueSetMember =  xQueueSelectFromSet( xQueueSet, pdMS_TO_TICKS(500) );

	  xQueueSetMember =  xQueueSelectFromSet( xQueueSet, portMAX_DELAY );

	  // QueueSet her zaman kuyruk icermeyebilir.Semaphore yapilarini da icerebilir. Bu yuzden bu sekilde bir kullanim daha dogrudur.
	  if(xQueueSetMember == NULL)
	  {
		  pcReceivedString = "NULL\r\n";
	  }
	  else if(xQueueSetMember == (QueueSetMemberHandle_t) xQueue1)
	  {
		  xQueueReceive( xQueue1, &pcReceivedString, 0 );
	  }
	  else if(xQueueSetMember == (QueueSetMemberHandle_t) xQueue2 )
	  {
		  xQueueReceive( xQueue2, &pcReceivedString, 0 );
	  }

	  UartPrint( pcReceivedString );

  }

  /* USER CODE END ReceiveFunction */
}

/* USER CODE BEGIN Header_Sender2Function */
/**
* @brief Function implementing the SenderTask2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Sender2Function */
void Sender2Function(void const * argument)
{
  /* USER CODE BEGIN Sender2Function */
	const TickType_t xBlockTime = pdMS_TO_TICKS( 2000 );
	const char * const pcMessage = "Message from SenderTask2\r\n";

  /* Infinite loop */
  for(;;)
  {
	  vTaskDelay( xBlockTime );
	  xQueueSend( xQueue2, &pcMessage, 0 );

  }
  /* USER CODE END Sender2Function */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
