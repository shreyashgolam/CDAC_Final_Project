/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : FreeRTOS-based CAN Transmitter for MQ-135 & BME680 Sensors
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "bme68x_necessary_functions.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	uint16_t ppm;
	uint32_t adcValue;
	uint16_t temperature;
	uint16_t humidity;
} CANMessage_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define A 116.6020682
#define B 2.211
#define R_L 10.0
#define R0 76.63

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];
uint32_t TxMailbox;

QueueHandle_t xQueueCAN;

osThreadId defaultTaskHandle, SensorTaskHandle, MQ135TaskHandle, CANTaskHandle;
struct bme68x_data sensorData;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
void StartDefaultTask(void const *argument);

void vSensorTask(void *pvParam);
void vMQ135Task(void *pvParam);
void vCANTask(void *pvParam);
void Error_Handler(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t Read_MQ135_ADC(void) {
	static uint32_t lastValidADCValue = 0;  // Store last valid ADC value

	HAL_ADC_Start(&hadc1);
	for (int retries = 3; retries > 0; retries--) {
		if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
			uint32_t adcValue = HAL_ADC_GetValue(&hadc1);
			HAL_ADC_Stop(&hadc1);
			lastValidADCValue = adcValue;  // Update last valid value
			return adcValue;
		}
		vTaskDelay(pdMS_TO_TICKS(5));  // Retry after 5ms
	}

	return lastValidADCValue;  // Return previous value if ADC fails
}

/* Convert ADC Value to CO2 PPM */
float Convert_ADC_to_Concentration(uint32_t adcValue) {
	float Rs = 0, ratio = 0, concentration = 0;
	if (adcValue != 0) {
		Rs = R_L * ((4095.0 - adcValue) / adcValue);
		ratio = Rs / R0;
		concentration = A * pow(ratio, -B);
	}
	return concentration;
}

/* Sensor Task - Reads BME680 Data */
void vSensorTask(void *pvParam) {

	while (1) {
		CANMessage_t msg;
		if (bme68x_single_measure(&sensorData) == 0) {
			msg.temperature = (uint16_t) (sensorData.temperature * 100);
			msg.humidity = (uint16_t) (sensorData.humidity * 100);

			/* Send Data to Queue */
			if (xQueueSend(xQueueCAN, &msg, portMAX_DELAY) != pdPASS) {
				HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14); // Indicate queue send failure
			}
		}
		vTaskDelay(pdMS_TO_TICKS(2000));
	}
}

void vMQ135Task(void *pvParam) {

	while (1) {
		CANMessage_t msg;
		msg.adcValue = Read_MQ135_ADC();
		msg.ppm = (uint16_t) Convert_ADC_to_Concentration(msg.adcValue);

		/* Send Data to Queue */
		if (xQueueSend(xQueueCAN, &msg, portMAX_DELAY) != pdPASS) {
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14); // Indicate queue send failure
		}
		vTaskDelay(pdMS_TO_TICKS(2500));
	}
}

void vCANTask(void *pvParam) {
	CANMessage_t receivedMsg;

	while (1) {
		if (xQueueReceive(xQueueCAN, &receivedMsg, portMAX_DELAY) == pdPASS) {
			/* Prepare CAN Data */
			TxData[0] = (uint8_t) (receivedMsg.ppm >> 8);
			TxData[1] = (uint8_t) (receivedMsg.ppm & 0xFF);
			TxData[2] = (uint8_t) (receivedMsg.adcValue >> 8);
			TxData[3] = (uint8_t) (receivedMsg.adcValue & 0xFF);
			TxData[4] = (uint8_t) (receivedMsg.temperature >> 8);
			TxData[5] = (uint8_t) (receivedMsg.temperature & 0xFF);
			TxData[6] = (uint8_t) (receivedMsg.humidity >> 8);
			TxData[7] = (uint8_t) (receivedMsg.humidity & 0xFF);

			/* Retry Sending CAN Message */
			uint8_t retry_count = 3;
			HAL_StatusTypeDef canStatus;
			while (retry_count--) {
				canStatus = HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData,
						&TxMailbox);
				if (canStatus == HAL_OK) {
					break;  // Exit loop if successful
				}
				HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14); // Indicate transmission failure
				vTaskDelay(pdMS_TO_TICKS(5));  // Small delay before retry
			}

			/* If message still fails after retries, discard and continue */
			if (retry_count == 0) {
				HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14); // Indicate permanent failure
				continue;  // Drop this message and move on
			}

			/* Check Mailbox Status with Timeout */
			uint32_t timeout = 1000;  // 1 second timeout
			while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0 && timeout > 0) {
				vTaskDelay(pdMS_TO_TICKS(1));
				timeout--;
			}

			if (timeout == 0) {
				HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14); // Indicate transmission timeout
			}
		}
	}
}

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
	MX_I2C1_Init();
	MX_ADC1_Init();
	MX_CAN1_Init();
	/* USER CODE BEGIN 2 */
	if (HAL_CAN_Start(&hcan1) != HAL_OK) {
		Error_Handler();
	}
	TxHeader.DLC = 8;  // Data length (4 bytes)
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = 0x103;  // CAN ID for this message
	TxHeader.ExtId = 0x00;
	TxHeader.TransmitGlobalTime = DISABLE;
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
	/* Create Queue */
	xQueueCAN = xQueueCreate(10, sizeof(CANMessage_t));
	if (xQueueCAN == NULL) {
		Error_Handler();
	}
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of defaultTask */
	osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
	/* Create Tasks */
	xTaskCreate(vMQ135Task, "MQ135Task", configMINIMAL_STACK_SIZE + 256, NULL,
			3, &MQ135TaskHandle);
	xTaskCreate(vSensorTask, "SensorTask", configMINIMAL_STACK_SIZE + 256, NULL,
			4, &SensorTaskHandle);
	xTaskCreate(vCANTask, "CANTask", configMINIMAL_STACK_SIZE + 256, NULL, 5,
			&CANTaskHandle);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
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
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief CAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN1_Init(void) {
	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 21;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.AutoBusOff = DISABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.AutoRetransmission = DISABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;

	if (HAL_CAN_Init(&hcan1) != HAL_OK) {
		Error_Handler();
	}

	/* USER CODE BEGIN CAN1_Init 2 */
	CAN_FilterTypeDef canfilterconfig;

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 0;
	canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	canfilterconfig.FilterIdHigh = 0x123 << 5;
	canfilterconfig.FilterIdLow = 0;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterMaskIdHigh = 0x103 << 5;
	canfilterconfig.FilterMaskIdLow = 0x0000;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilterconfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig) != HAL_OK) {
		Error_Handler();
	}

	/* Start CAN */
	if (HAL_CAN_Start(&hcan1) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* Enable Clocks for GPIO Ports */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/* Configure PD14 as an Output (LED Indicator for Errors) */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/* Configure CAN TX (PB9) & RX (PB8) */
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument) {
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {
		osDelay(1);
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
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
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
		HAL_Delay(500);
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
