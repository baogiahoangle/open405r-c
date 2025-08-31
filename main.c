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
#include "cmsis_os.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LCD_Driver.h"
#include "touch.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include <stdio.h>
#include <string.h>
#include "fram_i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define VREFINT          1.21f
#define ADCMAX           4095
#define V25              0.76f
#define AVG_SlOPE        0.0025f
//#define FRAM_ADDR        0xA0

uint8_t temperature_data[2];
uint8_t data_sram[2] = {0};
uint8_t RxData[3];
uint16_t AdcRaw[2];
uint8_t AdcConvCmplt = 0;
double VrefInt;
double VTempSens;
double Temperature;

#define ENABLE_BIT(reg, pos) reg |= (1 << pos)
#define DISABLE_BIT(reg, pos) reg &=~ (1 << pos)

uint16_t x, y;
CAN_TxHeaderTypeDef txHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t txdata[8];
uint32_t txMailbox;

RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define btn_Pin GPIO_PIN_1
#define btn_GPIO_Port GPIOA
#define LOG_DATA_SIZE     8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

osThreadId defaultTaskHandle;
osThreadId LCD_Touch_TaskHandle;
osThreadId LED_TaskHandle;
osThreadId CAN_TaskHandle;
osThreadId Button_FRAM_TasHandle;
osThreadId Display_CAN_DatHandle;
/* USER CODE BEGIN PV */
uint8_t menu_pressed = 0;
uint8_t current_screen = 0;
uint8_t need_redraw_main = 1;
static uint16_t fram_log_address = 0;
uint8_t need_redraw_menu = 1;
uint8_t led_running = 0; // Thêm biến này để theo dõi trạng thái LED (0: tắt, 1: bật)
/*
uint8_t FRAM_Write(uint16_t addr, uint8_t *data, uint16_t len) {
    uint8_t result;
    uint8_t buffer[len + 2];
    buffer[0] = (addr >> 8) & 0xFF;
    buffer[1] = addr & 0xFF;
    memcpy(&buffer[2], data, len);
    result = HAL_I2C_Master_Transmit(&hi2c2, FRAM_ADDR, buffer, len + 2, HAL_MAX_DELAY);
    HAL_Delay(5);
    return (result == HAL_OK) ? 0 : 1;
}

uint8_t FRAM_Read(uint16_t addr, uint8_t *data, uint16_t len) {
    uint8_t result;
    uint8_t buffer[2];

    buffer[0] = (addr >> 8) & 0xFF;
    buffer[1] = addr & 0xFF;

    result = HAL_I2C_Master_Transmit(&hi2c2, FRAM_ADDR, buffer, 2, HAL_MAX_DELAY);
    if (result != HAL_OK) return 1;

    result = HAL_I2C_Master_Receive(&hi2c2, FRAM_ADDR, data, len, HAL_MAX_DELAY);
    return (result == HAL_OK) ? 0 : 1;
}
*/
int8_t write_to_backup_sram(uint8_t *data, uint16_t bytes, uint16_t offset) {
  const uint16_t backup_size = 0x1000;
  uint8_t* base_addr = (uint8_t *) BKPSRAM_BASE;
  uint16_t i;

  if (bytes + offset >= backup_size) {
	  return -1;
  }

  ENABLE_BIT(RCC->AHB1ENR, RCC_AHB1ENR_BKPSRAMEN_Pos);
  ENABLE_BIT(RCC->AHB1ENR, RCC_APB1ENR_PWREN_Pos);
  // ENABLE_BIT(PWR->CR, 8);
  // ENABLE_BIT(PWR->CSR, 9);
  ENABLE_BIT(PWR->CR, PWR_CR_DBP_Pos);

  for (i = 0; i < bytes; i++) {
	  *(base_addr + offset + i) = *(data + i);
  }

  // DISABLE_BIT(PWR->CR, 8);
  DISABLE_BIT(PWR->CR, PWR_CR_DBP_Pos);
  return 0;
}

int8_t read_from_backup_sram(uint8_t *data, uint16_t bytes, uint16_t offset) {
  const uint16_t backup_size = 0x1000;
  uint8_t* base_addr = (uint8_t *) BKPSRAM_BASE;
  uint16_t i;

  if (bytes + offset >= backup_size) {
	  return -1;
  }

  ENABLE_BIT(RCC->AHB1ENR, RCC_APB1ENR_PWREN_Pos);
  // ENABLE_BIT(PWR->CR, 8);
  ENABLE_BIT(RCC->AHB1ENR, RCC_AHB1ENR_BKPSRAMEN_Pos);
  ENABLE_BIT(PWR->CR, PWR_CR_DBP_Pos);

  for (i = 0; i < bytes; i++) {
	  *(data + i) = *(base_addr + offset + i);
  }
  // DISABLE_BIT(PWR->CR, 8);
  DISABLE_BIT(PWR->CR, PWR_CR_DBP_Pos);
  return 0;
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1){
		AdcConvCmplt = 255;

	}
}

void UpdateTemperature(void) {
    if (AdcConvCmplt) {
//        char debug_str[50];
        VrefInt = (VREFINT * ADCMAX) / AdcRaw[0];
        VTempSens = (VrefInt * AdcRaw[1]) / ADCMAX;
        Temperature = (VTempSens - V25) / AVG_SlOPE + 25.0;
//      sprintf(debug_str, "T=%.2f do C", Temperature);
//      lcd_display_string(10, 180, (uint8_t*)debug_str, FONT_1206, BLACK);

        temperature_data[0] = (uint8_t)Temperature;
        temperature_data[1] = (uint8_t)((Temperature - temperature_data[0]) * 100);
        write_to_backup_sram(temperature_data, sizeof(temperature_data), 0);
        read_from_backup_sram(data_sram, sizeof(temperature_data), 0);

        AdcConvCmplt = 0;
        HAL_ADC_Start_DMA(&hadc1, (uint32_t*)AdcRaw, 2);
    }
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_I2C2_Init(void);
static void MX_RTC_Init(void);
void StartDefaultTask(void const * argument);
void LCD_Touch(void const * argument);
void LED(void const * argument);
void CAN(void const * argument);
void Button_FRAM(void const * argument);
void Display_CAN(void const * argument);

/* USER CODE BEGIN PFP */

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
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_I2C2_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan1);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
  ADC->CCR |= ADC_CCR_TSVREFE;
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)AdcRaw, 2);

  txHeader.StdId = 0x123;
  txHeader.ExtId = 0;
  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.DLC = 3;
  txHeader.TransmitGlobalTime = DISABLE;
  lcd_init();
  tp_init();
  tp_adjust();
  tp_dialog();
  if (FRAM_Init(&hi2c2) == HAL_OK) {
    lcd_display_string(10, 260, (uint8_t*)"FRAM OK", FONT_1206, GREEN);
  } else {
    lcd_display_string(10, 260, (uint8_t*)"FRAM Fail!", FONT_1206, RED);
  }
  osDelay(1000); // Để xem thông báo
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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 1024);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  osThreadDef(LCD_Touch_Task, LCD_Touch, osPriorityNormal, 0, 1024);
  LCD_Touch_TaskHandle = osThreadCreate(osThread(LCD_Touch_Task), NULL);

  osThreadDef(LED_Task, LED, osPriorityBelowNormal, 0, 128);
  LED_TaskHandle = osThreadCreate(osThread(LED_Task), NULL);

  osThreadDef(CAN_Task, CAN, osPriorityAboveNormal, 0, 526);
  CAN_TaskHandle = osThreadCreate(osThread(CAN_Task), NULL);

  osThreadDef(Button_FRAM_Tas, Button_FRAM, osPriorityBelowNormal, 0, 128);
  Button_FRAM_TasHandle = osThreadCreate(osThread(Button_FRAM_Tas), NULL);

  osThreadDef(Display_CAN_Dat, Display_CAN, osPriorityLow, 0, 128);
  Display_CAN_DatHandle = osThreadCreate(osThread(Display_CAN_Dat), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  osThreadSuspend(LED_TaskHandle);
  osThreadSuspend(CAN_TaskHandle);
  osThreadSuspend(Button_FRAM_TasHandle);
  osThreadSuspend(Display_CAN_DatHandle);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN ADC1_Init 2 */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
      Error_Handler();
  }
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  // Cấu hình bộ lọc CAN1

   CAN_FilterTypeDef filter;
   filter.FilterIdHigh = 0x0000;
   filter.FilterIdLow = 0x0000;
   filter.FilterMaskIdHigh = 0x0000;
   filter.FilterMaskIdLow = 0x0000;  // Nhận tất cả messages
   filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
   filter.FilterBank = 0;
   filter.FilterMode = CAN_FILTERMODE_IDMASK;
   filter.FilterScale = CAN_FILTERSCALE_32BIT;
   filter.FilterActivation = CAN_FILTER_ENABLE;
   filter.SlaveStartFilterBank = 14; // Số bank bắt đầu cho CAN2

   if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
   {
     Error_Handler();
   }

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 6;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = ENABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  CAN_FilterTypeDef filter2;
  filter2.FilterIdHigh =0x123 << 5;
  filter2.FilterIdLow = 0;
  filter2.FilterMaskIdHigh = 0x7FF << 5;
  filter2.FilterMaskIdLow = 0;
  filter2.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter2.FilterBank = 14;
  filter2.FilterMode = CAN_FILTERMODE_IDMASK;
  filter2.FilterScale = CAN_FILTERSCALE_32BIT;
  filter2.FilterActivation = CAN_FILTER_ENABLE;

  if (HAL_CAN_ConfigFilter(&hcan2, &filter2) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END CAN2_Init 2 */

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
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_Pin|GPIO_PIN_2|LCD_CS_Pin|LCD_RS_Pin
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_PA9_GPIO_Port, LCD_PA9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : btn_Pin */
  GPIO_InitStruct.Pin = btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin PB2 LCD_CS_Pin LCD_RS_Pin
                           PB9 */
  GPIO_InitStruct.Pin = LED_Pin|GPIO_PIN_2|LCD_CS_Pin|LCD_RS_Pin
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_PA9_Pin */
  GPIO_InitStruct.Pin = LCD_PA9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_PA9_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */

  for(;;)
  {
    if (current_screen == 0) {
    	if (need_redraw_main) {
    		lcd_clear_screen(WHITE);
    		lcd_display_string(85, 20, (uint8_t*)"[NHOM 08]", FONT_1608, N);
    		lcd_draw_rect(60, 5, 120, 40, GREEN);
    		lcd_draw_rect(62, 7, 116, 36, YELLOW);
    		lcd_display_string(40, 60, (uint8_t*)"DANH SACH THANH VIEN", FONT_1608, N);
    		lcd_draw_rect(20, 45, 200, 40, BLUE);
       		lcd_draw_rect(22, 47, 196, 36, RED);
    		lcd_display_string(0, 90, (uint8_t*)"22200061 Le Bao Gia Hoang", FONT_1608, N);
    		lcd_display_string(0, 120, (uint8_t*)"22200188 Doan Le Thanh Toan", FONT_1608, N);
    		lcd_display_string(0, 150, (uint8_t*)"21200283 Nguyen Dao Binh Duong", FONT_1608, N);
    		lcd_display_string(0, 180, (uint8_t*)"21200351 Le Minh Thanh", FONT_1608, N);
    		lcd_display_string(0, 210, (uint8_t*)"21200320 Tran Nguyen Nhat", FONT_1608, N);
//    		lcd_draw_rect(60, 190, 120, 40, BLUE);
//    		lcd_draw_rect(62, 192, 116, 36, BLUE);

    		lcd_draw_rect(60, 250, 120, 40, BLUE);
    		lcd_draw_rect(62, 252, 116, 36, RED);

    		// lcd_fill_rect(60, 190, 120, 40, BLUE);
    		lcd_display_string(90, 265, (uint8_t*)"TASK 2 >>", FONT_1608, N);
    		need_redraw_main = 0;
    	}

      while(current_screen == 0) {
    	  uint16_t x, y;
    	  if (tp_get_touch_point(&x, &y)) {
//          char str[30];
//          sprintf(str, "X:%d Y:%d   ", x, y);
//          lcd_display_string(10, 170, (uint8_t*)str, FONT_1206, RED);

          if (x >= 60 && x <= 180 && y >= 250 && y <= 290) {
            // lcd_display_string(50, 230, (uint8_t*)"MENU PRESSED!", FONT_1206, RED);
//            osDelay(500);

            extern uint8_t need_redraw_menu;
            need_redraw_menu = 1;

            current_screen = 1;
            need_redraw_main = 1;
          }
        }
//        osDelay(50);
      }
    }
    osDelay(5);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_LCD_Touch */
/**
* @brief Function implementing the LCD_Touch_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LCD_Touch */
void LCD_Touch(void const * argument)
{
  /* USER CODE BEGIN LCD_Touch */
	/* Infinite loop */
//	static uint8_t need_redraw_menu = 1;
	 for(;;)
	  {
	    if (current_screen == 1) {
	    	 if (need_redraw_menu) {
	    		lcd_clear_screen(N);

				lcd_draw_rect(10, 10, 130, 20, WHITE);
				lcd_draw_rect(11, 11, 128, 18, WHITE);
				//lcd_fill_rect(10, 10, 130, 20, WHITE);
				lcd_display_string(20, 12, (uint8_t*)"Nhom:[Nhom 08]", FONT_1608, WHITE);

			    lcd_draw_rect(20, 50, 90, 70, WHITE);
			    lcd_draw_rect(21, 51, 88, 68, WHITE);

			    //lcd_fill_rect(20, 50, 90, 70, WHITE);
			    lcd_display_string(30, 75, (uint8_t*)"Task 02-1", FONT_1608, WHITE);
			    lcd_display_string(30, 95, (uint8_t*)"LED OFF", FONT_1206, RED);

			    lcd_draw_rect(130, 50, 90, 70, WHITE);
			    lcd_draw_rect(131, 51, 88, 68, WHITE);
			    //lcd_fill_rect(130, 50, 90, 70, WHITE);
			    lcd_display_string(140, 75, (uint8_t*)"Task 02-2", FONT_1608, WHITE);

			    lcd_draw_rect(20, 140, 90, 70, WHITE);
			    lcd_draw_rect(21, 141, 88, 68, WHITE);
			   // lcd_fill_rect(20, 140, 90, 70, WHITE);
			    lcd_display_string(30, 165, (uint8_t*)"Task 02-3", FONT_1608, WHITE);

			    lcd_draw_rect(130, 140, 90, 70, WHITE);
			    lcd_draw_rect(131, 141, 88, 68, WHITE);
			    //lcd_fill_rect(130, 140, 90, 70, WHITE);
			    lcd_display_string(135, 165, (uint8_t*)"Task 02-4", FONT_1608, WHITE);

			    lcd_draw_rect(180, 220, 50, 20, WHITE);
			    lcd_draw_rect(181, 221, 48, 18, WHITE);
			   // lcd_fill_rect(180, 220, 50, 20, WHITE);
			    lcd_display_string(184, 224, (uint8_t*)"<<BACK", FONT_1206, WHITE);

		        need_redraw_menu = 0;
		      }

	      while(current_screen == 1) {
	        uint16_t x, y;
	        if (tp_get_touch_point(&x, &y)) {
	          //char str[30];
	         // sprintf(str, "X:%d Y:%d  ", x, y);
	        //  lcd_display_string(10, 240, (uint8_t*)str, FONT_1206, RED);

	        /*  if (x >= 180 && x <= 230 && y >= 220 && y <= 240) {
	             // lcd_display_string(100, 220, (uint8_t*)"SWITCHING...", FONT_1206, RED);
	              current_screen = 0;
	              need_redraw_menu = 1;
	          }*/
	          if (x >= 180 && x <= 230 && y >= 220 && y <= 240) {
	              // Hiển thị thông báo (tùy chọn)
	             // lcd_display_string(100, 220, (uint8_t*)"SWITCHING...", FONT_1206, RED);

	              // Suspend tất cả các task
	              osThreadSuspend(LED_TaskHandle);
	              osThreadSuspend(CAN_TaskHandle);
	              osThreadSuspend(Button_FRAM_TasHandle);
	              osThreadSuspend(Display_CAN_DatHandle);

	              // Reset biến led_running về 0
	              led_running = 0;

	              // Tắt LED (đảm bảo LED không còn sáng)
	              HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	              // Xóa các thông tin hiển thị trên màn hình
	              //lcd_fill_rect(0, 200, 240, 100, BLACK); // Xóa vùng thông tin nhiệt độ, CAN, v.v.

	              // Chuyển về màn hình chính
	              current_screen = 0;
	              need_redraw_menu = 1;

	              // Đặt biến redraw để vẽ lại màn hình chính hoàn toàn
	              need_redraw_main = 1;

	              osDelay(100); // Delay ngắn để đảm bảo các thay đổi được áp dụng
	          }

	          else if (x >= 20 && x <= 110 && y >= 50 && y <= 120) {
	              if (led_running) {
	                  // Nếu LED đang bật, tắt đi
	            	  lcd_display_string(30, 95, (uint8_t*)"LED ON", FONT_1206, N);
	                  lcd_display_string(30, 95, (uint8_t*)"LED OFF", FONT_1206, RED);
	                  //osDelay(200);
	                  led_running = 0;
	                  osThreadSuspend(LED_TaskHandle);

	                  // Tắt LED (đặt về trạng thái tắt)
	                  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	                  // Vẽ lại nút
	                //  lcd_fill_rect(20, 50, 90, 70, WHITE);
	                //  lcd_display_string(30, 75, (uint8_t*)"Task 02-1", FONT_1206, BLACK);
	              } else {
	                  // Nếu LED đang tắt, bật lên
	            	  lcd_display_string(30, 95, (uint8_t*)"LED OFF", FONT_1206, N);
	                  lcd_display_string(30, 95, (uint8_t*)"LED ON", FONT_1206, GREEN);
	                 // osDelay(200);
	                  led_running = 1;

	                  // Tắt các task khác
	                  osThreadSuspend(CAN_TaskHandle);
	                  osThreadSuspend(Button_FRAM_TasHandle);
	                  osThreadSuspend(Display_CAN_DatHandle);

	                  // Bật LED task
	                  osThreadResume(LED_TaskHandle);

	                  // Vẽ lại nút với đánh dấu đang bật
	                  //lcd_fill_rect(20, 50, 90, 70, CYAN);
	                  //lcd_display_string(30, 75, (uint8_t*)"Task 02-1", FONT_1206, BLACK);
	              }
	          }
		        else if (x >= 130 && x <= 220 && y >= 50 && y <= 120) {
		          osThreadResume(CAN_TaskHandle);
		          osThreadSuspend(LED_TaskHandle);
		          osThreadSuspend(Button_FRAM_TasHandle);
		          osThreadSuspend(Display_CAN_DatHandle);
		        }
		        else if (x >= 20 && x <= 110 && y >= 140 && y <= 210) {
		          osThreadResume(Button_FRAM_TasHandle);
		          osThreadSuspend(LED_TaskHandle);
		          osThreadSuspend(CAN_TaskHandle);
		          osThreadSuspend(Display_CAN_DatHandle);
		        }
		        else if (x >= 130 && x <= 220 && y >= 140 && y <= 210) {
		          osThreadResume(Display_CAN_DatHandle);
		          osThreadSuspend(LED_TaskHandle);
		          osThreadSuspend(CAN_TaskHandle);
		          osThreadSuspend(Button_FRAM_TasHandle);
		        }
	        }
	        osDelay(50);
	      }
	    }
	    osDelay(100);
	  }
  /* USER CODE END LCD_Touch */
}

/* USER CODE BEGIN Header_LED */
/**
* @brief Function implementing the LED_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LED */
void LED(void const * argument)
{
  /* USER CODE BEGIN LED */
  /* Infinite loop */
  for(;;)
  {

	  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  osDelay(400);
  }



  /* USER CODE END LED */
}

/* USER CODE BEGIN Header_CAN */
/**
* @brief Function implementing the CAN_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN */
void CAN(void const * argument)
{
  /* USER CODE BEGIN CAN */
	  for(;;)
	  {

	    UpdateTemperature();

	    txdata[0] = 8;
	    txdata[1] = temperature_data[0];
	    txdata[2] = temperature_data[1];
	    HAL_CAN_AddTxMessage(&hcan1, &txHeader, txdata, &txMailbox);
	    HAL_CAN_RxFifo0MsgPendingCallback(&hcan2);
	    HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
	    char debug_str[50];
	    sprintf(debug_str, "[Nhom %d] + [%d.%d do C]    ",txdata[0], txdata[1], txdata[2]);
	   // lcd_fill_rect(0, 250, 240, 40, BLACK);

	    lcd_display_string(10, 270, (uint8_t*)debug_str, FONT_1206, GREEN);
	    osDelay(500);
	    lcd_display_string(10, 270, (uint8_t*)debug_str, FONT_1206, N);//xóa chữ
	  }

}
  /* USER CODE END CAN */


/* USER CODE BEGIN Header_Button_FRAM */
/**
* @brief Function implementing the Button_FRAM_Tas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Button_FRAM */
void Button_FRAM(void const * argument)
{
	  uint8_t button_state;
	  uint8_t last_button_state = GPIO_PIN_SET;
	  last_button_state = HAL_GPIO_ReadPin(btn_GPIO_Port, btn_Pin);
	  uint8_t log_data[LOG_DATA_SIZE];
	  char lcd_msg[60];
	  for(;;)
	  {

	    button_state = HAL_GPIO_ReadPin(btn_GPIO_Port, btn_Pin);
	    if (last_button_state == GPIO_PIN_SET && button_state == GPIO_PIN_RESET) {
	    	// lcd_fill_rect(0, 220, 240, 40, BLUE);
	    	lcd_display_string(10, 220, (uint8_t*)"Button Pressed!", FONT_1206, WHITE);
	    	AdcConvCmplt = 0;
	    	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)AdcRaw, 2);
	    	uint32_t timeout_adc = HAL_GetTick() + 50;
	    	while(!AdcConvCmplt && HAL_GetTick() < timeout_adc){
	    	  osDelay(1);
	      }
	    	if(AdcConvCmplt){
	    	  UpdateTemperature();
	      } else {
	    	  temperature_data[0] = 0xFF;
          temperature_data[1] = 0xFF;
	    	  lcd_display_string(10, 240, (uint8_t*)"ADC Timeout!", FONT_1206, RED);
	      }
	    	        HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	    	        HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	    	        log_data[0] = temperature_data[0];
	    	        log_data[1] = temperature_data[1];
	    	        log_data[2] = sTime.Hours;
	    	        log_data[3] = sTime.Minutes;
	    	        log_data[4] = sTime.Seconds;
	    	        log_data[5] = sDate.Date;
	    	        log_data[6] = sDate.Month;
	    	        log_data[7] = (uint8_t)sDate.Year;
	    	        if (FRAM_WriteBytes(&hi2c2, fram_log_address, log_data, LOG_DATA_SIZE) == HAL_OK) {
	    	            sprintf(lcd_msg, "Da ghi vao FRAM @0x%04X", fram_log_address);
	    	            lcd_display_string(10, 240, (uint8_t*)lcd_msg, FONT_1206, GREEN);
	    	            fram_log_address += LOG_DATA_SIZE;
	    	            if (fram_log_address > (FRAM_MAX_MEMORY_ADDRESS - LOG_DATA_SIZE + 1) ) {
	    	                fram_log_address = 0;
	    	            }
	    	        } else {
	    	            lcd_display_string(10, 240, (uint8_t*)"Failed!", FONT_1206, RED);
	    	        }
	    	        osDelay(200);
	    	    }
	    	    last_button_state = button_state;

	    	    osDelay(100);
	    	    lcd_display_string(10, 240, (uint8_t*)lcd_msg, FONT_1206, N);//xóa chữ
	    	    lcd_display_string(10, 220, (uint8_t*)"Button Pressed!", FONT_1206, N);//xóa chữ
	    	  }
  /* USER CODE END Button_FRAM */
}

/* USER CODE BEGIN Header_Display_CAN */
/**
* @brief Function implementing the Display_CAN_Dat thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Display_CAN */
void Display_CAN(void const * argument)
{
  /* USER CODE BEGIN Display_CAN */
	for(;;) {
	    char display_str[50];
	    sprintf(display_str, "[Nhom %d] + [%d.%d do C]",RxData[0], RxData[1], RxData[2]);
	    lcd_display_string(10, 300, (uint8_t*)display_str, FONT_1206, GREEN);
	    osDelay(3000);
	    current_screen = 0;
	    need_redraw_main = 1;
	    osThreadSuspend(Display_CAN_DatHandle);

  /* USER CODE END Display_CAN */
}
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
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  //day la commet cac muc duoc them vao 10h
  //sưa doi trong user code begin pv giúp task 2 đóng khi task1 mở lúc mới nạp code
  //  hcan1.Init.AutoRetransmission = ENABLE;
  //
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
