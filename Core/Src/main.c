/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "stm32l4s5i_iot01_tsensor.h"
#include "stm32l4s5i_iot01_hsensor.h"
#include "stm32l4s5i_iot01_psensor.h"
#include "stm32l4s5i_iot01_magneto.h"
#include "stm32l4s5i_iot01_accelero.h"
#include "stm32l4s5i_iot01_gyro.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <math.h>
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MAX_DATA_POINTS 20
#define SAMPLE_RATE_SECONDS 2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

osThreadId SensorReadHandle;
osThreadId EnqueueDataHandle;
osThreadId HandleAnomalyHandle;
osThreadId ProcessingDataHandle;
/* USER CODE BEGIN PV */

#define HIGH_PRESSURE 1022.0
#define LOW_PRESSURE 1000.0
#define HIGH_TEMP 35.0
#define LOW_TEMP 0.0
#define HIGH_HUMIDITY 55.0
#define LOW_HUMIDITY 40.0

uint8_t buttonStatus = 0;
uint32_t C6[38];
TimerInterrupt = 0;

float pressure = 0;
float humidity = 0;
float gyro[3] = {};
float temperature = 0;
float gyroscope = 0;
char buf[150];
int data;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
void SensorReadTask(void const * argument);
void EnqueueDataTask(void const * argument);
void HandleAnomalyTask(void const * argument);
void ProcessingDataTask(void const * argument);

/* USER CODE BEGIN PFP */

void enqueueValue(float value, float *queue, int *front, int *rear);
void dequeueValue(float *queue, int *front, int *rear);
bool isQueueFull(int front, int rear);
bool isQueueEmpty(int front);
bool isRisingTrend(float *queue, int front, int rear);
bool isFallingTrend(float *queue, int front, int rear);
bool isFluctuatingTrend(float *queue, int front, int rear);
float calculateCovariance(const float *data, int n);
void raiseAlarm(const char *message);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int frontPressure = -1, rearPressure = -1;
int frontTemperature = -1, rearTemperature = -1;
int frontHumidity = -1, rearHumidity = -1;
int frontGyroscopeX = -1, rearGyroscopeX = -1;
int frontGyroscopeY = -1, rearGyroscopeY = -1;
int frontGyroscopeZ = -1, rearGyroscopeZ = -1;

float pressureQueue[MAX_DATA_POINTS];
float temperatureQueue[MAX_DATA_POINTS];
float humidityQueue[MAX_DATA_POINTS];
float gyroscopeQueueX[MAX_DATA_POINTS];
float gyroscopeQueueY[MAX_DATA_POINTS];
float gyroscopeQueueZ[MAX_DATA_POINTS];

void C6gen(){
	float n = 38;

	for (int i=0; i<n; i++){
		float mod = (float) (i)/n;
		float pi = 3.1415926535897932;

		float input = arm_sin_f32(2*pi*mod);

		input = input +1;
		input = input * 127.5; // 2/3 of 255/2 range

		uint32_t out;
		out = (uint32_t)input;

		C6[i]=out;

	}
}

void enqueueValue(float value, float *queue, int *front, int *rear) {
    if (isQueueFull(*front, *rear)) {
        dequeueValue(queue, front, rear); // Remove oldest value to make room
    }
    if (isQueueEmpty(*front)) {
        *front = *rear = 0;
    } else {
        *rear = (*rear + 1) % MAX_DATA_POINTS;
    }
    queue[*rear] = value;
}
void dequeueValue(float *queue, int *front, int *rear) {
    if (isQueueEmpty(*front)) {
        printf("Queue underflow! No data to remove.\n");
        return;
    }
    if (*front == *rear) {
        // Single element in the queue
        *front = *rear = -1;
    } else {
        *front = (*front + 1) % MAX_DATA_POINTS;
    }
}
bool isQueueFull(int front, int rear) {
    return (rear + 1) % MAX_DATA_POINTS == front;
}
bool isQueueEmpty(int front) {
    return front == -1;
}
bool isRisingTrend(float *queue, int front, int rear) {
    float fieldData[MAX_DATA_POINTS];
    int count = 0;

    for (int i = front; ; i = (i + 1) % MAX_DATA_POINTS) {
        fieldData[count++] = queue[i];
        if (i == rear) break;
    }
    return calculateCovariance(fieldData, count) > 0.5;
}
bool isFallingTrend(float *queue, int front, int rear) {
    float fieldData[MAX_DATA_POINTS];
    int count = 0;

    for (int i = front; ; i = (i + 1) % MAX_DATA_POINTS) {
        fieldData[count++] = queue[i];
        if (i == rear) break;
    }
    return calculateCovariance(fieldData, count) < -0.5;

}
bool isFluctuatingTrend(float *queue, int front, int rear) {
    float fieldData[MAX_DATA_POINTS];
    int count = 0;

    for (int i = front; ; i = (i + 1) % MAX_DATA_POINTS) {
        fieldData[count++] = queue[i];
        if (i == rear) break;
    }
    float covariance = calculateCovariance(fieldData, count);
    return fabs(covariance) > 0.5;

}
void raiseAlarm(const char *message) {
	memset(buf, '\0', 100);
    	sprintf(buf, "ALARM: %s\n", message);
    	HAL_UART_Transmit(&huart1, (unsigned char *) buf, 100, 100);
    	HAL_UART_Transmit(&huart1, (unsigned char *) "\33[2K\r", 8, 100);
}
float calculateCovariance(const float *data, int size) {
    if (size <= 1) {
        return 0.0f;
    }

    float mean = 0.0f;
    float meanOfSquares = 0.0f;

    for (int i = 0; i < size; i++) {
        mean += data[i];
        meanOfSquares += data[i] * data[i];
    }

    mean /= size;

    return (meanOfSquares / size) - (mean * mean);
}

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
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Init(&huart1);
  HAL_TIM_Base_Start_IT(&htim2);

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
  /* definition and creation of SensorRead */
  osThreadDef(SensorRead, SensorReadTask, osPriorityLow, 0, 256);
  SensorReadHandle = osThreadCreate(osThread(SensorRead), NULL);

  /* definition and creation of EnqueueData */
  osThreadDef(EnqueueData, EnqueueDataTask, osPriorityNormal, 0, 256);
  EnqueueDataHandle = osThreadCreate(osThread(EnqueueData), NULL);

  /* definition and creation of HandleAnomaly */
  osThreadDef(HandleAnomaly, HandleAnomalyTask, osPriorityNormal, 0, 256);
  HandleAnomalyHandle = osThreadCreate(osThread(HandleAnomaly), NULL);

  /* definition and creation of ProcessingData */
  osThreadDef(ProcessingData, ProcessingDataTask, osPriorityHigh, 0, 256);
  ProcessingDataHandle = osThreadCreate(osThread(ProcessingData), NULL);

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_ABOVE_80MHZ;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x30A175AB;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1500;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : myButton_Pin */
  GPIO_InitStruct.Pin = myButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(myButton_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_13){
		  HAL_DAC_Stop_DMA(&hdac1, DAC1_CHANNEL_1);


	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_SensorReadTask */
/**
  * @brief  Function implementing the SensorRead thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_SensorReadTask */
void SensorReadTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	BSP_TSENSOR_Init();
	BSP_PSENSOR_Init();
	BSP_HSENSOR_Init();
	BSP_MAGNETO_Init();
	BSP_ACCELERO_Init();
	BSP_GYRO_Init();
  /* Infinite loop */
  for(;;)
  {
    osDelay(500);
    pressure = BSP_PSENSOR_ReadPressure();
    humidity = BSP_HSENSOR_ReadHumidity();
    BSP_GYRO_GetXYZ(gyro);
    temperature = BSP_TSENSOR_ReadTemp();
    gyroscope = (gyro[0]+gyro[1]+gyro[2])/300;
    data = sprintf(buf, "pressure = %2.d , humidity = %2.d , temperature = %2.d, gyro x= %2.d y = %2.d z = %2.d \r\n", (int)pressure, (int)humidity, (int)temperature, (int)gyro[0], (int)gyro[1], (int)gyro[2]);
	HAL_UART_Transmit(&huart1, (unsigned char *) buf, 150, 100);
	HAL_UART_Transmit(&huart1, (unsigned char *) "\33[2K\r", 8, 100);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_EnqueueDataTask */
/**
* @brief Function implementing the EnqueueData thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_EnqueueDataTask */
void EnqueueDataTask(void const * argument)
{
  /* USER CODE BEGIN EnqueueDataTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(500);
    enqueueValue(pressure, pressureQueue, &frontPressure, &rearPressure);
    enqueueValue(temperature, temperatureQueue, &frontTemperature, &rearTemperature);
    enqueueValue(humidity, humidityQueue, &frontHumidity, &rearHumidity);
    enqueueValue(gyro[0]/100, gyroscopeQueueX, &frontGyroscopeX, &rearGyroscopeX);
    enqueueValue(gyro[1]/100, gyroscopeQueueY, &frontGyroscopeY, &rearGyroscopeY);
    enqueueValue(gyro[2]/100, gyroscopeQueueZ, &frontGyroscopeZ, &rearGyroscopeZ);
  }
  /* USER CODE END EnqueueDataTask */
}

/* USER CODE BEGIN Header_HandleAnomalyTask */
/**
* @brief Function implementing the HandleAnomaly thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HandleAnomalyTask */
void HandleAnomalyTask(void const * argument)
{
  /* USER CODE BEGIN HandleAnomalyTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(500);
    C6gen();
    if (!isQueueEmpty(frontPressure) && !isQueueEmpty(frontHumidity) &&
        !isQueueEmpty(frontTemperature) && !isQueueEmpty(frontGyroscopeX)) {

        bool fallingPressure = isFallingTrend(pressureQueue, frontPressure, rearPressure);
        bool risingHumidity = isRisingTrend(humidityQueue, frontHumidity, rearHumidity);
        bool fluctuatingTemp = isFluctuatingTrend(temperatureQueue, frontTemperature, rearTemperature);
        bool strongWinds = isFluctuatingTrend(gyroscopeQueueX, frontGyroscopeX, rearGyroscopeX)||isFluctuatingTrend(gyroscopeQueueY, frontGyroscopeY, rearGyroscopeY)||isFluctuatingTrend(gyroscopeQueueZ, frontGyroscopeZ, rearGyroscopeZ);



        if (fallingPressure && risingHumidity && fluctuatingTemp) {
            raiseAlarm("Thunderstorm detected!");
			//HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_1, C6, 38, DAC_ALIGN_8B_R);
        } else if (fallingPressure && risingHumidity && strongWinds) {
            raiseAlarm("Hurricane/Cyclone detected!");
			//HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_1, C6, 38, DAC_ALIGN_8B_R);
        } else if (fallingPressure && fluctuatingTemp && strongWinds) {
            raiseAlarm("Tornado detected!");
			//HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_1, C6, 38, DAC_ALIGN_8B_R);
        } else if (fluctuatingTemp && isFluctuatingTrend(pressureQueue, frontPressure, rearPressure) &&
                   isFluctuatingTrend(humidityQueue, frontHumidity, rearHumidity)) {
            raiseAlarm("Atmospheric front detected!");
			//HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_1, C6, 38, DAC_ALIGN_8B_R);
        } else if (risingHumidity && temperatureQueue[rearTemperature] > 15.0) {
            raiseAlarm("Fog detected!");
			//HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_1, C6, 38, DAC_ALIGN_8B_R);
        } else if (risingHumidity && temperatureQueue[rearTemperature] <= 0.0) {
            raiseAlarm("Snow detected!");
			//HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_1, C6, 38, DAC_ALIGN_8B_R);
        } else if (risingHumidity && temperatureQueue[rearTemperature] > 0.0) {
            raiseAlarm("Rain detected!");
			//HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_1, C6, 38, DAC_ALIGN_8B_R);
        } else if (strongWinds) {
            raiseAlarm("Earthquake/Strong winds detected");
			HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_1, C6, 38, DAC_ALIGN_8B_R);
        } else{

        	sprintf(buf, "Nothing to worry about\n");
        	HAL_UART_Transmit(&huart1, (unsigned char *) buf, 150, 100);
        	HAL_UART_Transmit(&huart1, (unsigned char *) "\33[2K\r", 8, 100);
        }
    }
  }
  /* USER CODE END HandleAnomalyTask */
}

/* USER CODE BEGIN Header_ProcessingDataTask */
/**
* @brief Function implementing the ProcessingData thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ProcessingDataTask */
void ProcessingDataTask(void const * argument)
{
  /* USER CODE BEGIN ProcessingDataTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(100);
  }
  /* USER CODE END ProcessingDataTask */
}

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
