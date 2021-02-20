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
osThreadId SenderTask2Handle;
osThreadId ReceiverTaskHandle;
/* USER CODE BEGIN PV */
QueueHandle_t xQueue;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void SenderFunction(void const * argument);
void ReceiveFunction(void const * argument);

/* USER CODE BEGIN PFP */
void UartPrint(char* message);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef enum
{
	eSender1,
	eSender2
} DataSource_t;

typedef struct
{
	uint8_t ucValue;
	DataSource_t eDataSource;
} Data_t;

static const Data_t xStructsToSend[ 2 ] =
{
	{ 100, eSender1 },
	{ 200, eSender2 }
};

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
  xQueue = xQueueCreate(3, sizeof(Data_t));

  if(xQueue != NULL)
  {
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of SenderTask1 */
  osThreadDef(SenderTask1, SenderFunction, osPriorityNormal, 0, 128);
  SenderTask1Handle = osThreadCreate(osThread(SenderTask1), (void*)&xStructsToSend[0]);

  /* definition and creation of SenderTask2 */
  osThreadDef(SenderTask2, SenderFunction, osPriorityNormal, 0, 128);
  SenderTask2Handle = osThreadCreate(osThread(SenderTask2), (void*)&xStructsToSend[1]);

  /* definition and creation of ReceiverTask */
  osThreadDef(ReceiverTask, ReceiveFunction, osPriorityBelowNormal, 0, 128);
  ReceiverTaskHandle = osThreadCreate(osThread(ReceiverTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  }
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

/* USER CODE BEGIN Header_SenderFunction */
/**
  * @brief  Function implementing the SenderTask1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_SenderFunction */
void SenderFunction(void const * argument)
{
  /* USER CODE BEGIN 5 */
	BaseType_t xStatus;
	const TickType_t xTicksToWait = pdMS_TO_TICKS( 5000 );


  /* Infinite loop */
  for(;;)
  {
	  osThreadState stateS1;
	  stateS1 = osThreadGetState(SenderTask1Handle);

	  osThreadState stateS2;
	  stateS2 = osThreadGetState(SenderTask2Handle);

	  // Iki ayri task'den gelen veriyi kuyruga yazar. Eger kuyruk full olursa ve 100ms icerisinde bosluk acilmazsa
	  // Kernel kaldigi yerden kodu islemeye devam edecektir. Fakat 100ms daha once kuyrukta yer acilirsa veriye kuyruga yazip gecicektir.

	  // Gonderme TASK'inin onceligi daha fazla oldugundan kuyruk full olana kadar icerisine veri yazmaya devam eder.
	  // Kuyruk full oldugundan ise artik veri yazamayacagi icin KERNEL Gonderme TASK'ini 100 ms boyunca kuyrukta bosluk acilana kadar "Blocked" moduna alir.

	  xStatus = xQueueSendToBack( xQueue, argument, xTicksToWait );

	  if( xStatus != pdPASS )
	  {
		  UartPrint("Veri kuyruga yazilamadi..\r\n");
	  }
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

	Data_t xReceivedStructure;
	BaseType_t xStatus;

	// Kuyruk full olmadan OKUMA taski calismayacaktir. Cunku OKUMA taskinin onceligi dusuktur bu sebeple
	// GONDERME task'i "Blocked" moda girmeden OKUMA taski calismayacaktir.

	if( uxQueueMessagesWaiting( xQueue ) != 3 )
	{
		UartPrint("Kuyruk tam dolmadi!!!\r\n" );
	}

  /* Infinite loop */
  for(;;)
  {

	  // Gonderme TASK'i kuyruk full oldugu icin "Blocked" durumdadir ve bu yüzden KERNEL Okuma Task'ini calistirir.
	  // Kuyrukdan veri okundugu gibi Gonderme TASK'i calismaya devam eder.

	  xStatus = xQueueReceive( xQueue, &xReceivedStructure, 0 );

	  if( xStatus == pdPASS )
	  {
		  char msg[25];

		  if( xReceivedStructure.eDataSource == eSender1 )
		  {
			  snprintf(msg,25,"From Sender 1 = %u\r\n", xReceivedStructure.ucValue);
			  UartPrint(msg);
		  }
		  else
		  {
			  snprintf(msg,25,"From Sender 2 = %u\r\n", xReceivedStructure.ucValue);
			  UartPrint(msg);
		  }
	  }
	  else
	  {

		  UartPrint("Kuyruktan veri okunamadi.\r\n" );
	  }
  }
  /* USER CODE END ReceiveFunction */
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
