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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
//#include <modbus.h>
#include <chintPowerSensor_modbus.h>
#include <sml/sml_parser.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define uSend(x) uartSend((char *)x, sizeof x)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

// UART is used for production and pins are swapped
// -> avoid driving Nucleo UART TX pin and set debug output to 0
// Because of pin swapping, Nucleo UART receives smart meter messages
#define DEBUG_OUTPUT 0
char debug_tx_buf[50];

// **********************
// USART1 is RS485 Modbus
// **********************
#define RS485_RX_BUFFER_SIZE   100
uint8_t rs485_rxDMAbuf[RS485_RX_BUFFER_SIZE] = {0};

volatile bool request_restart_rs485_rx;
volatile bool request_restart_ir_smart_meter_rx;
mbus_t modbus;


// ****************************************
// USART2 is Smart Meter Infrared Interface
// ****************************************
// test in linux:
// stty raw -F /dev/ttyACM0 9600  # without raw, linux adds \r to \n and corrupts data!
// does not work in final version, because tx and rx pin have to be swapped to access rx pin
#define IR_RX_BUFFER_SIZE 1024
uint8_t ir_rxDMAbuf[IR_RX_BUFFER_SIZE] = {0};
uint8_t ir_rxUSERbuf[IR_RX_BUFFER_SIZE] = {0};

volatile bool request_ir_USERbuf_clear = false;
volatile bool sml_parser_started = false;
volatile uint16_t ir_rxUSERbuf_pos = 0;
__IO uint32_t     uwNbReceivedChars;

meter_model_t METER_MODEL = ISKRA_MT175;  // no CRC check; wait for more data -> does not work with MT631
//meter_model_t METER_MODEL = ISKRA_MT631;  //    CRC check; short data

uint16_t MIN_SML_LEN = 200;  // is set to larger value for ISKRA_MT175

#if 0  // init some testvalues, dont use for production to avoid disturbing the PV-inverter
volatile float power_W = 300;
volatile float energy_export_kWh = 4.0;
volatile float energy_import_kWh = 2.0;
#else
volatile float power_W = 0;
volatile float energy_export_kWh = 0;
volatile float energy_import_kWh = 0;
#endif



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */


uint32_t mbus_tickcount()
{
	return HAL_GetTick();
}


void resetWatchdog()
{
	// Watchdog runs at 32/8=4kHz -> ~1sec for 4095
    /* Refresh IWDG: reload counter */
    if(HAL_IWDG_Refresh(&hiwdg) != HAL_OK) {
      /* Refresh Error */
      Error_Handler();
    }
}


void show_sys_error()
{
	  static uint16_t err_cnt = 0;
	  static bool green_led_on;
	  if (green_led_on) {
		  GPIOB->BRR = (1<<3);  // disable green LED and pin D13
		  GPIOA->BRR = (1<<4);  // enable yellow LED (PA4=A3)
		  green_led_on = false;
	  } else {
		  GPIOB->BSRR = (1<<3);  // enable green LED and pin D13
		  GPIOA->BSRR = (1<<4);  // disable yellow LED (PA4=A3)
		  green_led_on = true;
	  }
	  // HAL_Delay(100); does not work without IRQ
	  uint32_t i = 0;
	  while (i<800000) { // ~100ms  with -Ofast
		  i++;
		  asm("nop");
	  }
	  err_cnt++;

	  if (err_cnt >= 600) {  // autoreset after ~60 seconds (-Ofast) if no watchdog overflow
		  NVIC_SystemReset();
	  }
}


void uartSend(char* ptr, int len)
{
#if DEBUG_OUTPUT == 1
	HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 10);
#endif
}


void uSendInt(int value)
{
#if DEBUG_OUTPUT == 1
	itoa(value, debug_tx_buf, 10);
	int tx_len=strlen(debug_tx_buf);
	HAL_UART_Transmit(&huart2, (uint8_t *)debug_tx_buf, tx_len, 10);
#endif
}


int mbus_send(const mbus_t context, const uint8_t* data, const uint16_t size)
{
	// V1: blocking during send
	HAL_Delay(1);
	if (HAL_UART_Transmit( &huart1, (uint8_t*) data, size, 1000) == HAL_OK) {
		return size;
	}

	// V2: use DMA
//	if (HAL_UART_Transmit_DMA( &huart1, (uint8_t*) data, size) == HAL_OK) {
//		return size;
//	}

	return 0;
}

void Start_RS485_Reception(void);
void Start_SmartMeterIR_Reception(void);

void UserDataTreatment(UART_HandleTypeDef *huart, uint8_t* pData, uint16_t Size);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  uint8_t tx_buf[STMODBUS_MAX_RESPONSE_SIZE] = {0};

  Modbus_Conf_t mb_config;
  mb_config.send = &mbus_send;
  mb_config.sendbuf = tx_buf;
  mb_config.sendbuf_sz = STMODBUS_MAX_RESPONSE_SIZE;
  modbus = mbus_powersensor_open(&mb_config);

  resetWatchdog();
  uSend("SM_Adapter  ");
  uSend(__DATE__);
  uSend(" ");
  uSend(__TIME__);
  uSend("\n");

  /*## Check if the system has resumed from IWDG reset ####################*/
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != 0x00u)
  {
    uSend("Watchdog caused reset!");
    uSend("\n");
    show_sys_error();
  }

  uint32_t ticks_last_sml_success = HAL_GetTick();

  if (METER_MODEL == ISKRA_MT175) {
  	MIN_SML_LEN = 264;
  }

  Start_SmartMeterIR_Reception();

  Start_RS485_Reception();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  resetWatchdog();
#if 0  // V1: polling
		uint8_t rx_byte;

		if (HAL_UART_Receive(&huart1, &rx_byte, 1, 1) == HAL_OK) {
			rx_buf[pos++] = rx_byte;
			if (pos == 8) {
				total_msg++;
				if (total_msg==2){
					asm("nop");
				}
				for (int i = 0; i<8; i++) {
					mbus_poll(modbus, rx_buf[i]);
				}
				pos = 0;
			}
		}

#else	// V2: async
	if(request_restart_rs485_rx) {
		request_restart_rs485_rx = false;

		if (HAL_ERROR == HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rs485_rxDMAbuf, RS485_RX_BUFFER_SIZE)) {
			Error_Handler();
		}
	}
#endif

	// **************************
	// * Parse Smart Meter Data *
	// **************************

	if(request_restart_ir_smart_meter_rx) {
		request_restart_ir_smart_meter_rx = false;

		if (HAL_ERROR == HAL_UARTEx_ReceiveToIdle_DMA(&huart2, ir_rxDMAbuf, IR_RX_BUFFER_SIZE)) {
			Error_Handler();
		}
	}

	if (request_ir_USERbuf_clear == false) {
		if (ir_rxUSERbuf_pos > MIN_SML_LEN) {
			sml_parser_started = true;
			uSendInt(ir_rxUSERbuf_pos);
			meterdata_t meterdata;
			if (parse_sml((char*)ir_rxUSERbuf, ir_rxUSERbuf_pos, METER_MODEL, &meterdata) == SML_PARSE_OKAY) {
				power_W = meterdata.power;
				energy_export_kWh = meterdata.e_export_100mWh/10000.0;
				energy_import_kWh = meterdata.e_import_100mWh/10000.0;

				ticks_last_sml_success = HAL_GetTick();
				resetWatchdog();
				uSend("SML_OK\n");

				static bool yellow_led_on;
				if (yellow_led_on) {
					GPIOA->BSRR = (1<<4);  // disable yellow LED (PA4=A3)
					yellow_led_on = false;
				} else {
					GPIOA->BRR = (1<<4);  // enable yellow LED (PA4=A3)
					yellow_led_on = true;
				}

			} else {
				uSend("SML_ERR\n");
			}
			sml_parser_started = false;
			request_ir_USERbuf_clear = true;
		}
	}

	// set power to zero in case of broken link to smart meter
	const uint32_t TIMEOUT_SML_SEC = 10;
	const uint32_t TIMEOUT_SML_REBOOT_SEC = 30;
	uint32_t ticks = HAL_GetTick();
	static uint32_t ticks_prev_check;

	if ( ticks > ticks_last_sml_success+TIMEOUT_SML_SEC*1000) {
		if (ticks < ticks_prev_check) {  // prevent error during overflow
			ticks_last_sml_success = 0;  // set to 0 when ticks overflow
		} else {
			power_W = 0;
			if ( ticks > ticks_last_sml_success+TIMEOUT_SML_REBOOT_SEC*1000) {
				NVIC_SystemReset();
			}
		}
	}
	ticks_prev_check = ticks;

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_8;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
  huart2.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(YL_LED_GPIO_Port, YL_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IR_TX_GPIO_Port, IR_TX_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : YL_LED_Pin IR_TX_Pin */
  GPIO_InitStruct.Pin = YL_LED_Pin|IR_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void Start_SmartMeterIR_Reception(void)
{
  /* Initializes Buffer swap mechanism (used in User callback) :
     - 2 physical buffers aRXBufferA and aRXBufferB (RX_BUFFER_SIZE length)
  */
//  pBufferReadyForReception = aRXBufferA;
//  pBufferReadyForUser      = aRXBufferB;
  uwNbReceivedChars        = 0;

  /* Print user info on PC com port */
  //PrintInfo(&hlpuart1, aTextInfoStart, COUNTOF(aTextInfoStart));

  /* Initializes Rx sequence using Reception To Idle event API.
     As DMA channel associated to UART Rx is configured as Circular,
     reception is endless.
     If reception has to be stopped, call to HAL_UART_AbortReceive() could be used.

     Use of HAL_UARTEx_ReceiveToIdle_DMA service, will generate calls to
     user defined HAL_UARTEx_RxEventCallback callback for each occurrence of
     following events :
     - DMA RX Half Transfer event (HT)
     - DMA RX Transfer Complete event (TC)
     - IDLE event on UART Rx line (indicating a pause is UART reception flow)
  */
  if (HAL_OK != HAL_UARTEx_ReceiveToIdle_DMA(&huart2, ir_rxDMAbuf, IR_RX_BUFFER_SIZE))
  {
    Error_Handler();
  }
}


void Start_RS485_Reception(void)
{
  /* Initializes Rx sequence using Reception To Idle event API.
     As DMA channel associated to UART Rx is configured as Circular,
     reception is endless.
     If reception has to be stopped, call to HAL_UART_AbortReceive() could be used.

     Use of HAL_UARTEx_ReceiveToIdle_DMA service, will generate calls to
     user defined HAL_UARTEx_RxEventCallback callback for each occurrence of
     following events :
     - DMA RX Half Transfer event (HT)
     - DMA RX Transfer Complete event (TC)
     - IDLE event on UART Rx line (indicating a pause is UART reception flow)
  */
  if (HAL_OK != HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rs485_rxDMAbuf, RS485_RX_BUFFER_SIZE))
  {
    Error_Handler();
  }
}


/**
  * @brief  User implementation of the Reception Event Callback
  *         (Rx event notification called after use of advanced reception service).
  * @param  huart UART handle
  * @param  Size  Number of data available in application reception buffer (indicates a position in
  *               reception buffer until which, data are available)
  * @retval None
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart == &huart1) {  // RS485

		static bool green_led_on;
		if (green_led_on) {
			GPIOB->BRR = (1<<3);  // disable green LED and pin D13
			green_led_on = false;
		} else {
			GPIOB->BSRR = (1<<3);  // enable green LED and pin D13
			green_led_on = true;
		}

		for (uint16_t i = 0; i < Size; i++) {
			//pBufferReadyForUser[i] = rs485_rxDMAbuf[i];
			//HAL_UART_Transmit(&huart2, &pBufferReadyForUser[old_pos+i],1, 10);  // debug
			mbus_poll(modbus, rs485_rxDMAbuf[i]);
		}

		request_restart_rs485_rx = true;

	} else {  // Smart Meter Infrared / UART Interface

		static uint32_t ticks_last_rx_ms;
		static uint16_t old_dmabuf_pos = 0;

		if (request_ir_USERbuf_clear) {
			ir_rxUSERbuf_pos = 0;
			request_ir_USERbuf_clear = false;
		}

		if (    sml_parser_started == false   // only write to userbuf if parser is not active
			&& (Size != old_dmabuf_pos)  // check if number of received data in reception buffer has changed
			) {

			if (HAL_GetTick()-ticks_last_rx_ms > 400) { // drop old data after 400ms
				ticks_last_rx_ms = HAL_GetTick();
				ir_rxUSERbuf_pos = 0;
			}

			/* Check if position of index in reception buffer has simply be increased
			   of if end of buffer has been reached */
			if (Size > old_dmabuf_pos) {
				/* Current position is higher than previous one */
				uwNbReceivedChars = Size - old_dmabuf_pos;
				/* Copy received data in "User" buffer for evacuation */
				for (uint16_t i = 0; i < uwNbReceivedChars; i++) {
					ir_rxUSERbuf[ir_rxUSERbuf_pos+i] = ir_rxDMAbuf[old_dmabuf_pos + i];
				}

			} else {
				/* Current position is lower than previous one : end of buffer has been reached */
				/* First copy data from current position till end of buffer */
				uwNbReceivedChars = IR_RX_BUFFER_SIZE - old_dmabuf_pos;
				/* Copy received data in "User" buffer for evacuation */
				for (uint16_t i = 0; i < uwNbReceivedChars; i++)	{
					ir_rxUSERbuf[ir_rxUSERbuf_pos+i] = ir_rxDMAbuf[old_dmabuf_pos + i];
				}
				/* Check and continue with beginning of buffer */
				if (Size > 0) {
					for (uint16_t i = 0; i < Size; i++)	{
						ir_rxUSERbuf[ir_rxUSERbuf_pos+uwNbReceivedChars + i] = ir_rxDMAbuf[i];
					}
					uwNbReceivedChars += Size;
				}
			}

			/* Process received data that has been extracted from Rx User buffer */
			ir_rxUSERbuf_pos += uwNbReceivedChars;

			request_restart_ir_smart_meter_rx = true;
		}

		old_dmabuf_pos = Size;
	}
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
  __disable_irq();
  while (1)
  {
	  show_sys_error();
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
