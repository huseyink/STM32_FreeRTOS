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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
UART_HandleTypeDef huart1;
osThreadId Task1Handle;
/* USER CODE BEGIN PV */
osThreadId Task2Handle;

const char* Task2_message = "Hello from TASK2 \r\n";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void Task1_Function(void const * argument);
void Task2_Function(void const * argument);

/* USER CODE BEGIN PFP */
void LedToggle(void);
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
  MX_USART1_UART_Init();
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
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Task1 */
  osThreadDef(Task1, Task1_Function, osPriorityNormal, 0, 128);
  Task1Handle = osThreadCreate(osThread(Task1), NULL);

  /* USER CODE BEGIN RTOS_THREADS */

 /* UYGULAMA #1 */

  /*
   2 adet TASK olusturulmustur. TASK'lerin oncelikleri esit olarak ayarlanmistir.
   Task1 icerisinde LedToggle() fonksiyonu cagirilmaktadir. Task2 icerisinde ise UartPrint(message) fonksiyonu ile veri yollanmaktadir.
   Oncelikler esit oldugundan her 2 task de sikintisiz sekilde calismaktadir. TICK_RATE(1000Hz) oldugundan her 2 task'in de calisma suresi
   1ms araliklarladir. Eger herhangi bir TASK icerisindeki gorev 1ms'den fazla suruyorsa islem yarida birakilir ve diger yeniden o TASK isletilmeye
   basladiginda kaldigi yerden devam eder.

   Eger TASK'lerden birinin onceligi arttirilir yada azaltilirsa sadece en yuksek oncelikli TASK calismaya devam edicektir.
   */

//	 osThreadDef(Task2, Task2_Function, osPriorityAboveNormal, 0, 128);
//	 Task2Handle = osThreadCreate(osThread(Task2), (void*)Task2_message);

  /* UYGULAMA #2 */

   /*
    2 adet TASK olusturulmustur. TASK'lerin oncelikleri esit olarak ayarlanmistir.
    Task1 icerisinde LedToggle() fonksiyonu cagirilmaktadir. Task2 icerisinde ise UartPrint(message) fonksiyonu ile veri yollanmaktadir.
    Oncelikler esit oldugundan her 2 task de sikintisiz sekilde calismaktadir. Bu sefer her 2 TASK içerisinde de osDelay() kullanilmistir.
    Bu sebeple TASK'lerin oncelikleri ne olursa olsun 2 TASK'de calismaya devam edecektir.

    Eger Task1 dusuk oncelikli Task2 yuksek oncelikli yapilirsa ve Task1 icerisinde osDelay() olmaz fakat Task2 icerisinde osDelay() olursa,
    yuksek oncelikli Task2 osDelay() sebebiyle "Blocked" moda girecektir. Bu sebeple Task1 calisabilecektir.

    Fakat oncelikleri tam tersi yaparsak Task1 yuksek oncelikli, Task2 dusuk oncelikli olursa; TASK1 yuksek oncelikli oldugundan ve osDelay()
    barindirmadiginda "Blocked" moda gecis yapamayacak devamli "Running" modda calisacagindan Task2 calismayacaktir.

    */

   osThreadDef(Task2, Task2_Function, osPriorityAboveNormal, 0, 128);
   Task2Handle = osThreadCreate(osThread(Task2), (void*)Task2_message);

  /* UYGULAMA #3 */

  /* Eger configUSE_PREEMPTION 0 yapilirsa Kernel co-operative olarak calisacaktir. Co-operative modda TASK onceliklerinin bir onemi yoktur.
   * Eger TASK'ler arasi gecis yapilmak isteniyorsa ya taskYIELD() ile zorla context switch islemi yapilmalidir. Ya da "Running" modunda calisan
   * bir TASK'in "Blocked" moda gecis yapmasi gerekmektedir. Eger du durumlar olusmuyorsa Kernel ilk girdigi TASK'i calistirmaya devam eder.
   *
   * Uygulama sonucunda eger TASK'ler icerisinde osDelay() kullanilmazsa sistem ilk hangi TASK'e girerse o TASK'i devamli calistirir.
   * Fakat herhangi bir TASK'in icerisinde osDelay() kullandigimizda sistem diger TASK'e gecmektedir. Eger o TASK icerisinde de TASK'i "Running"
   * moddan "Blocked" moda gecirecek bir olay yok ise Kernel o TASK'i calistirmaya devam eder.
   *
   */

//     osThreadDef(Task2, Task2_Function, osPriorityNormal, 0, 128);
//     Task2Handle = osThreadCreate(osThread(Task2), (void*)Task2_message);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RED_LED_Pin|GREEN_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_LED_Pin */
  GPIO_InitStruct.Pin = USER_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USER_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RED_LED_Pin GREEN_LED_Pin */
  GPIO_InitStruct.Pin = RED_LED_Pin|GREEN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void Task2_Function(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  /*
	  // UYGULAMA #1
	  UartPrint((char*)argument);
	  */


	  // UYGULAMA #2
	  UartPrint((char*)argument);
	  osDelay(50);


  }
  /* USER CODE END 5 */
}

void LedToggle()
{
	HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
}

void UartPrint(char* message)
{
	HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), 50);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Task1_Function */
/**
  * @brief  Function implementing the Task1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Task1_Function */
void Task1_Function(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  /*
	  // UYGULAMA #1
	  LedToggle();
	  */


	  // UYGULAMA #2
	  LedToggle();
	  osDelay(100);

  }
  /* USER CODE END 5 */
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
