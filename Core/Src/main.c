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

/*
 * Include user alarm functions
 */
#ifndef ALARM_H_
#include "alarm.h"
#endif

/*
 * Include shift register library for quad 7-segment displays
 */
#ifndef SEVSEG_SHIFT_H_
#include "sevSeg_shift.h"
#endif

/*
 * Include board-specific variable mappings
 */
#ifndef BEETHOVENCLOCKVARS_H_
#include "beethovenClockVars.h"
#endif

/*
 * Include cap. touch interface library
 */
#ifndef AT42QT1070_H_
#include "AT42QT1070.h"
#endif

/*
 * Include SPI Flash Memeory Driver
 */
#ifndef W25QXXX_H_
#include "W25Qxxx.h"
#endif

/*
 * Include I2S Amplifier Driver
 */
#ifndef NAU8315YG_H_
#include "NAU8315YG.h"
#endif

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
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s1;
DMA_HandleTypeDef hdma_spi1_tx;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */


/*
 * Timer to be used for PWMing display
 */
TIM_HandleTypeDef *timerPWM = &htim2;

/*
 * Timer to be used for non-blocking delays
 */
TIM_HandleTypeDef *timerDelay = &htim14;

/*
 * Timer to be used for long 10-minute snooze
 */
TIM_HandleTypeDef *timerSnooze = &htim16;

/*
 * RCR value for long 10-minute snooze
 */
const uint32_t timerSnooze_RCR = 100;

/*
 * State bools
 */

// Used to flag alarm set mode
bool alarmSetMode = false;

/*
 * Used to indicate whether the clock is in a
 * state to sound a snoozed alarm again after 10 minutes.
 * false = regular operation, true = 10-minute snooze period
 */
bool secondSnooze = false;

// Used to indicate whether or not we are in alarm beep state
bool beepMode = false;

/*
 * Cap. touch struct
 */
QT1070 capTouch;

/*
 * SPI Flash memory struct
 */
W25Q spiFlash;

/*
 * I2S amplifier struct
 */
NAU8315YG i2sAmp;

/*
 * DMA buffers & flags
 */

// Two buffers to ping-pong in between for SPI RX
uint8_t spiRxBuff[BUFFER_SIZE];

// Single circular output buffer to TX I2S audio data
uint16_t i2sTxBuff[BUFFER_SIZE * 2];
//uint16_t i2sTxBuff[BUFFER_SIZE * 2];

// Variable to keep track of where to read audio data from in memory
uint32_t flashReadAddr = initialMemoryOffset;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S1_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

/*
 * Call to fetch the current time from the RTC and send to the LED display.
 *
 * @param none
 *
 * @ret HAL status object
 */
HAL_StatusTypeDef updateAndDisplayTime(void);

/*
 * Call to fetch the current alarm from the RTC and send to the LED display.
 *
 * @param none
 *
 * @ret HAL status object
 */
HAL_StatusTypeDef updateAndDisplayAlarm(void);

/*
 * Called on interrupt from display button to toggle 7-segment intensity.
 *
 * @param none
 *
 * @ret HAL status object
 */
HAL_StatusTypeDef displayButtonISR(void);

/*
 * Called on interrupt from alarm enable button to toggle user alarm.
 *
 * @param none
 *
 * @ret HAL status object
 */
HAL_StatusTypeDef alarmEnableISR(void);

/*
 * Called on interrupt from alarm set button to enter alarm set loop.
 *
 * @param none
 *
 * @ret HAL status object
 */
HAL_StatusTypeDef alarmSetISR(void);

/*
 * Called on interrupt from hour and minute set buttons.
 * Contain logic to set user alarm hour or minute.
 *
 * @param none
 *
 * @ret HAL status object
 */
HAL_StatusTypeDef hourSetISR(void);
HAL_StatusTypeDef minuteSetISR(void);

/*
 * Called on interrupt from time format switch
 * Contains logic to change RTC time format
 */
HAL_StatusTypeDef timeFormatSwitchISR(void);

/*
 * Hour and minute incrementing functions for alarm time and current time
 */
void alarmHourInc(void);
void currHourInc(void);
void alarmMinuteInc(void);
void currMinuteInc(void);

/*
 * Enters loop to signal user alarm
 */
void userAlarmBeep(void);

/*
 * Displays non-critical error on debug LED and continues functionality
 */
void dispFault();

/*
 * Displays Critical Error on clock and halts all functionality
 */
void dispFailure();

/*
 * Updates RTC Backup Registers with user alarm time
 */
void updateRTCBackupReg(void);

/*
 * Begins double-buffered DMA streams to provide audio output
 */
void startAudioStream(void);

/*
 * Halts double-buffered DMA streams
 */
void stopAudioStream(void);

/*
 * Converts the passed RTC time to the same time expressed in 24 hour (military) time
 */
RTC_TimeTypeDef conv2Mil(RTC_TimeTypeDef *oldTime);

// Fills I2S tx buffer at a given array index offset
void fillTxBuffer(uint16_t offset);

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
  MX_RTC_Init();
  MX_I2C1_Init();
  MX_I2S1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

	  // HAL Status handle for error-checking
	  HAL_StatusTypeDef halRet = HAL_OK;

	  // Determine which time format we will be using
	  	if(HAL_GPIO_ReadPin(timeFormatSwitchPort, timeFormatSwitchPin) == userTimeFormatGPIO_12) {
	  	  userTimeFormat = RTC_HOURFORMAT_12;
	  	}
	  	else {
	  	  userTimeFormat = RTC_HOURFORMAT_24;
	  	}

  	  // Set Smooth Calibration Value
	  halRet = HAL_RTCEx_SetSmoothCalib(&hrtc, RTC_SMOOTHCALIB_PERIOD_8SEC,
									RTC_SMOOTHCALIB_PLUSPULSES_RESET, rtcCalVal);
	  if(halRet != HAL_OK) {
		  // Failure to talk to RTC is a hard failure
		  dispFailure();
	  }

	  // Init the internal RTC alarm time to track the current time
	  halRet = initRTCInternalAlarm(&hrtc, &currTime, &currDate);
	  if(halRet != HAL_OK) {
		  // Failure to initialize RTC alarm is a hard failure
		  dispFailure();
		}

	  // Initialize all GPIOs to be used with 7 segment display
		sevSeg_Init(shiftDataPin, shiftDataClockPin, shiftStoreClockPin,
					shiftOutputEnablePin, shiftMCLRPin,
					GPIOPortArray, timerDelay, timerPWM, tim_PWM_CHANNEL);


		halRet = updateAndDisplayTime();
		if(halRet != HAL_OK) {
		  // Failure to display current time is a hard failure
		  dispFailure();
		}

    /*
     * Initialize capacitive touch sensor
     */

		// Used to separate return initializations into critical and non-critical errors.
		uint8_t initRet = 0;

		initRet = capTouch_Init(&capTouch, &hi2c1, timerDelay,
								&capTouchResetPort, capTouchResetPin, capTouchChannels);
		if( (initRet == 1) || (initRet == 3) || (initRet == 4)) {

			/* Critical Errors:
			 * 1 = Failure to read correct device ID
			 * 2 = Failure to read Keys
			 * 3 = Failure to enable keys
			 */
			dispFailure();
		}
		else if (initRet == 2) {
			/*
			 * Non-critical Errors:
			 * 2 = Failure to Recalibrate
			 */
			dispFault();
		}
		else if(initRet == 0) {
			// initRet = 0 = all is well
			__NOP();
		}

		// Set averaging factor
		uint8_t avgFactors_New[7] = {AVGFact, AVGFact, AVGFact, AVGFact, 0, 0, 0};
		halRet = capTouch_SetAveragingFactor(&capTouch, avgFactors_New);

		if(halRet != HAL_OK) {
			// This is sensitivity-setting and a non-critical error
			dispFault();
		}

		// Set detection integration factors
		uint8_t detIntFactors_New[7] = {DIFact, DIFact, DIFact, DIFact, DIFact, DIFact, DIFact};
		halRet = capTouch_SetDetectionIntegrator(&capTouch, detIntFactors_New);
		if(halRet != HAL_OK) {
			// This is sensitivity-setting and a non-critical error
			dispFault();
		}

		userAlarmToggle = false;			//Default to off


    /*
     * If the bootstrap backup register reads 0x00 (never written to,)
     * initialize the alarm time to a default value.
     *
     * Else, initialize to whatever is stored in backup registers.
     */

		if((uint8_t)HAL_RTCEx_BKUPRead(&hrtc, bootstrapBackupReg) == 0) {

			HAL_RTCEx_BKUPWrite(&hrtc, userAlarmHourBackupReg, 0x01);
			HAL_RTCEx_BKUPWrite(&hrtc, userAlarmMinuteBackupReg, 0x00);
			HAL_RTCEx_BKUPWrite(&hrtc, userAlarmTFBackupReg, RTC_HOURFORMAT12_AM);

			// Write backup register with a non-zero value to signify that it has been initialized before
			HAL_RTCEx_BKUPWrite(&hrtc, bootstrapBackupReg, 0xFFFFFFFF);

		}

		userAlarmTime.Hours = (uint8_t)HAL_RTCEx_BKUPRead(&hrtc, userAlarmHourBackupReg);
		userAlarmTime.Minutes = (uint8_t)HAL_RTCEx_BKUPRead(&hrtc, userAlarmMinuteBackupReg);
		userAlarmTime.TimeFormat = (uint8_t)HAL_RTCEx_BKUPRead(&hrtc, userAlarmTFBackupReg);


		// Init Memory Chip
		initRet = W25Q_Init(&spiFlash, nCSPort, nWPPort, nHOLDPort,
	    		 nCSPin, nWPPin, nHOLDPin, &hspi2, spiFlash_devID, spiFlash_isQuadChip, spiFlash_driveStrength);

		// Enter error loop if there's an error in initialization
		if( (initRet == 1) || (initRet == 7) ) {
			/* Critical Errors:
			* 1 = Failure to release chip from power down
			* 7 = Failure to disable write protection
			*/
			dispFailure();
		}
		else if ( ((initRet >= 2) && (initRet <= 6)) || (initRet == 8) ) {
			/*
			* Non-critical Errors:
			* 2 = Failure to reset chip
			* 3,6,8 = Failure to read status registers
			* 4 = Failure to set driver strength
			* 5 = Failure to read device ID
			*/
			dispFault();
		}
		else if(initRet == 0) {
			// initRet = 0 = all is well
			__NOP();
		}


		// Init i2s amplifier
		NAU8315YG_Init(&i2sAmp, &hi2s1, i2sAmp_enablePort, i2sAmp_enablePin);

//		startAudioStream();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
    /* USER CODE END WHILE */
	  if(beepMode) {
		  userAlarmBeep();
	  }

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

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
  hi2c1.Init.Timing = 0x00C12166;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S1_Init(void)
{

  /* USER CODE BEGIN I2S1_Init 0 */

  /* USER CODE END I2S1_Init 0 */

  /* USER CODE BEGIN I2S1_Init 1 */

  /* USER CODE END I2S1_Init 1 */
  hi2s1.Instance = SPI1;
  hi2s1.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s1.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s1.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s1.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s1.Init.AudioFreq = I2S_AUDIOFREQ_44K;
  hi2s1.Init.CPOL = I2S_CPOL_LOW;
  if (HAL_I2S_Init(&hi2s1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S1_Init 2 */

  /* USER CODE END I2S1_Init 2 */

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

//  RTC_TimeTypeDef sTime = {0};
//  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_12;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
//  sTime.Hours = 0x1;
//  sTime.Minutes = 0x0;
//  sTime.Seconds = 0x0;
//  sTime.SubSeconds = 0x0;
//  sTime.TimeFormat = RTC_HOURFORMAT12_AM;
//  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
//  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
//  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
//  sDate.Month = RTC_MONTH_JANUARY;
//  sDate.Date = 0x1;
//  sDate.Year = 0x0;
//
//  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
//  {
//    Error_Handler();
//  }

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0x1;
  sAlarm.AlarmTime.Minutes = 0x1;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.TimeFormat = RTC_HOURFORMAT12_AM;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY|RTC_ALARMMASK_HOURS
                              |RTC_ALARMMASK_SECONDS;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  // Do not initialize time - pull from whatever is in register
    HAL_RTC_GetTime(&hrtc, &currTime, RTCTimeFormat);

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 800-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 244*4;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 58595;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 10;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|SHIFT_STORE_CLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI_CHIP_SELECT_Pin|MEM_nWP_Pin|MEM_nHOLD_Pin|SHIFT_MCLR_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, AMP_ENABLE_Pin|DEBUG_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SHIFT_DATA_IN_Pin|SHIFT_DATA_CLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAPTOUCH_RESET_GPIO_Port, CAPTOUCH_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : PC13 SHIFT_STORE_CLK_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_13|SHIFT_STORE_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI_CHIP_SELECT_Pin MEM_nWP_Pin MEM_nHOLD_Pin SHIFT_DATA_IN_Pin
                           SHIFT_DATA_CLK_Pin SHIFT_MCLR_Pin */
  GPIO_InitStruct.Pin = SPI_CHIP_SELECT_Pin|MEM_nWP_Pin|MEM_nHOLD_Pin|SHIFT_DATA_IN_Pin
                          |SHIFT_DATA_CLK_Pin|SHIFT_MCLR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : AMP_ENABLE_Pin DEBUG_LED_Pin CAPTOUCH_RESET_Pin */
  GPIO_InitStruct.Pin = AMP_ENABLE_Pin|DEBUG_LED_Pin|CAPTOUCH_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ALARM_SET_BUTTON_EXTI_Pin */
  GPIO_InitStruct.Pin = ALARM_SET_BUTTON_EXTI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ALARM_SET_BUTTON_EXTI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : HOUR_SET_BUTTON_EXTI_Pin MINUTE_SET_BUTTON_EXTI_Pin ALARM_EN_BUTTON_EXTI_Pin DISPLAY_BUTTON_EXTI_Pin */
  GPIO_InitStruct.Pin = HOUR_SET_BUTTON_EXTI_Pin|MINUTE_SET_BUTTON_EXTI_Pin|ALARM_EN_BUTTON_EXTI_Pin|DISPLAY_BUTTON_EXTI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : TIME_SWITCH_EXTI_Pin */
  GPIO_InitStruct.Pin = TIME_SWITCH_EXTI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(TIME_SWITCH_EXTI_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/*
 * Pulls updated time from RTC and send new time to user display
 */
HAL_StatusTypeDef updateAndDisplayTime(void) {

	HAL_StatusTypeDef halRet = HAL_OK;

	getRTCTime(&hrtc, &currTime, &currDate);

	// If the user wants 24-hour time, shift the time accordingly
	if((userTimeFormat == RTC_HOURFORMAT_24)) {

		RTC_TimeTypeDef currTimeMil = conv2Mil(&currTime);

		sevSeg_updateDigits(&currTimeMil, userAlarmToggle);

	}
	else {
		sevSeg_updateDigits(&currTime, userAlarmToggle);
	}

	return halRet;

}

/*
 * Sends current user alarm time to user display
 * This doesn't pull from the RTC because the RTC alarm is not being used for that
 */

HAL_StatusTypeDef updateAndDisplayAlarm(void) {

	HAL_StatusTypeDef halRet = HAL_OK;

	// If the user wants 24-hour time, shift the time accordingly
	if((userTimeFormat == RTC_HOURFORMAT_24)) {

		RTC_TimeTypeDef userAlarmTimeMil = conv2Mil(&userAlarmTime);

		sevSeg_updateDigits(&userAlarmTimeMil, userAlarmToggle);

	}
	else {
		sevSeg_updateDigits(&userAlarmTime, userAlarmToggle);
	}



	return halRet;

}

/*
 * Interrupt callback function for RTC interrupt kickback
 * Occurs every minute increment
 *
 * Pulls alarm time from RTC, increments and sets new alarm time, and updates time.
 * Sets off user alarm if the alarm is enabled.
 */
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc) {

	  RTC_AlarmTypeDef sAlarm = {0};
	  HAL_RTC_GetAlarm(hrtc, &sAlarm, internalAlarm, RTCTimeFormat);


	  if(sAlarm.AlarmTime.Minutes>58) {
		sAlarm.AlarmTime.Minutes=0;
	  } else {
		sAlarm.AlarmTime.Minutes=sAlarm.AlarmTime.Minutes+1;
	  }
		while(HAL_RTC_SetAlarm_IT(hrtc, &sAlarm, FORMAT_BIN)!=HAL_OK){}

	  updateAndDisplayTime();

	  // If alarm is enabled and current time matches user alarm time, set off the alarm.
	  if(userAlarmToggle && userAlarmTime.Hours == currTime.Hours
			  && userAlarmTime.Minutes == currTime.Minutes && userAlarmTime.TimeFormat == currTime.TimeFormat) {
		  beepMode = true;
	  }


}

/*
 * If user alarm is enabled and the RTC time equals the user alarm time, this function is entered.
 *
 * This function blinks the display LEDs and plays sound until the user
 * disables this beeping through cap. touch or the alarm enable button.
 *
 * Functionality depends on whether or not this is the first or second snooze.
 * 		First Snooze: 10-minute timer is started and alarm
 * 		              is beeped again at the end of this 10 minutes.
 * 		Second Snooze: No timer is started and silencing the alarm silences it for good.
 */
void userAlarmBeep() {

	if (secondSnooze) { 		//If the user has already snoozed once,

		// Stop the timer and
		HAL_TIM_Base_Stop_IT(timerSnooze);

		// Reset count to 0
		// only bits 0 - 15 should be changed.
		timerSnooze->Instance->CNT &= 0xFFFF0000;

		// Reset interrupt status register
		timerSnooze->Instance->SR &= 0xFFFC;

		// Re-write RCR with 10
		timerSnooze->Instance->RCR &= 0xFF00;
		timerSnooze->Instance->RCR |= timerSnooze_RCR;

	}

	HAL_TIM_Base_Stop(timerDelay);
	HAL_TIM_Base_Start(timerDelay);						// Begin timer 16 counting (to 500 ms)
	uint32_t timerVal = __HAL_TIM_GET_COUNTER(timerDelay);	// Get initial timer value to compare to
	bool displayBlink = false;

	// Start audio DMA streams
	startAudioStream();

	do {						// Beep buzzer and blink display until snooze button is pressed

		updateAndDisplayTime();				// Update to current time and display

		if(__HAL_TIM_GET_COUNTER(timerDelay) - timerVal >= (65535 / 2)) {		// Use hardware timer to blink/beep display

			sevSeg_setIntensity(sevSeg_intensityDuty[displayBlink]);	// Toggle 0% to 50% duty cycle

			timerVal = __HAL_TIM_GET_COUNTER(timerDelay);				// Update timer value

			displayBlink = !displayBlink;							// Toggle display blink counter

		}


		capTouch_readChannels(&capTouch);

	} while(capTouch.keyStat == 0x00 &&
			(HAL_GPIO_ReadPin(alarmEnableButtonPort, alarmEnableButtonPin) != GPIO_PIN_RESET));

	/*
	 * Stop blinking, turn off audio, set 50% duty cycle, update time
	 */
	HAL_TIM_Base_Stop(timerDelay);

	// Stop audio stream
	stopAudioStream();

	updateAndDisplayTime();				// Update to current time and display
	sevSeg_setIntensity(sevSeg_intensityDuty[1]);	// Set to 50% duty cycle
	displayToggle = 2;								// Set to 2 for future display button ISRs


	// If this is the first snooze,
	if(!secondSnooze) {

		// Start the snooze timer to trigger an interrupt after 10 minutes
		HAL_TIM_Base_Start_IT(timerSnooze);

		// Set flag
		secondSnooze = true;

	} else {

		// Reset flag
		/*
		 * This must be done here because if it's done
		 * in the top conditional, the secondSnooze is always true;
		 */
		secondSnooze = false;

	}

	// Reset beepMode bool
	beepMode = false;

}

/*
 * General Falling-Edge EXTI Callback function
 *
 * Internal conditional determines which function to call
 * based on which button was pressed.
 */
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {

	HAL_StatusTypeDef halRet;					// Flag for printing interrupt status

	if(GPIO_Pin == displayButtonPin) {
		halRet = displayButtonISR();
		if (halRet != HAL_OK) {
			//printf("Error toggling display.\n\r");
		} else {
			//printf("Display intensity toggled.\n\r");
		}
	}
	else if(GPIO_Pin == alarmEnableButtonPin) {
		halRet = alarmEnableISR();
		if (halRet != HAL_OK) {
			//printf("Error toggling user alarm.\n\r");
		} else {
			//printf("User alarm toggled.\n\r");
		}
	}
	else if(GPIO_Pin == alarmSetButtonPin) {
		halRet = alarmSetISR();
		if (halRet != HAL_OK) {
			//printf("Error setting user alarm.\n\r");
		} else {
			//printf("User alarm set.\n\r");
		}
	}
	else if(GPIO_Pin == hourSetButtonPin) {
		halRet = hourSetISR();
		if (halRet != HAL_OK) {
			//printf("Error incrementing hour.\n\r");
		} else {
			//printf("Hour increment ISR success.\n\r");
		}
	}
	else if(GPIO_Pin == minuteSetButtonPin) {
		halRet = minuteSetISR();
		if (halRet != HAL_OK) {
			//printf("Error incrementing minute.\n\r");
		} else {
			//printf("Minute increment ISR success.\n\r");
		}
	}
	else if(GPIO_Pin == timeFormatSwitchPin) {

		halRet = timeFormatSwitchISR();

	}

	else {			//Code should never reach here, but do nothing if it does.
		__NOP();
	}

}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {

	if(GPIO_Pin == timeFormatSwitchPin) {

		timeFormatSwitchISR();

	}

}

/*
 * Used for second snooze functionality
 *
 * This is entered at the end of a 10-minute snooze, at
 * which point the timer kicks back an interrupt.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if((htim == timerSnooze) && (secondSnooze == true)) {

		userAlarmBeep();

	}

}

/*
 * Toggles through user time display brightness settings
 */
HAL_StatusTypeDef displayButtonISR(void) {

	HAL_StatusTypeDef halRet = HAL_OK;

	updateAndDisplayTime();

	sevSeg_setIntensity(sevSeg_intensityDuty[displayToggle]);		//Turn display to proper duty cycle

	if(displayToggle >= 1) {			// Increment display toggle or reset back down to 0;
		displayToggle = 0;
	} else {
		displayToggle++;
	}

	return halRet;				// Return HAL status

}

/*
 * Toggles user alarm enable bool and alarm enable LED.
 */
HAL_StatusTypeDef alarmEnableISR(void) {



	//printf("Entered alarm toggle ISR\n\r");
	HAL_StatusTypeDef halRet = HAL_OK;

	if(!userAlarmToggle) {					// If alarm is disabled, enable it.

		userAlarmToggle = true;								// Toggle internal flag to true

	}
	else if (userAlarmToggle) {				// If alarm is enabled, disable it.

		userAlarmToggle = false;							// Toggle internal flag to false
	}
	else {
		__NOP();							//Code should never reach here.
	}

	updateAndDisplayTime();				// Update display with correct decimal point

	// Reset snooze time
	secondSnooze = false;

	return halRet;

}

/*
 * Delays for 3 seconds, checks if alarm set button is still pressed.
 * If it is, allow user to set the new alarm time
 * and blink display to indicate this
 *
 */
HAL_StatusTypeDef alarmSetISR(void) {



	HAL_StatusTypeDef halRet = HAL_OK;


	/*
	 * Wait for 3 seconds to see if alarm set button is still pressed
	 */

	// Delay for 3 * 1s intervals
	for(uint8_t i = 0; i < 3; i++) {

		HAL_TIM_Base_Stop(timerDelay);
		timerDelay->Instance->CNT = 0;						// Reset timer base
		HAL_TIM_Base_Start(timerDelay);						// Begin timer 16 counting (to 1 s)
	//	timerVal = __HAL_TIM_GET_COUNTER(timerDelay);	// Get initial timer value to compare to

		do {

		} while(__HAL_TIM_GET_COUNTER(timerDelay) < (65535));
	}

	// If button is still pressed, we are in alarm set mode.
	if(HAL_GPIO_ReadPin(alarmSetButtonPort, alarmSetButtonPin) == GPIO_PIN_RESET) {
		alarmSetMode = true;
	}
	else {
		return halRet;
	}

	/*
	 * Then, if we are in alarm set mode, go through the
	 * alarm set process until the button is pressed again
	 */


	// Reset Timer
	HAL_TIM_Base_Stop(timerDelay);
	timerDelay->Instance->CNT = 0;
	HAL_TIM_Base_Start(timerDelay);						// Begin timer 16 counting (to 1 s)
	uint16_t timerVal = __HAL_TIM_GET_COUNTER(timerDelay);	// Get initial timer value to compare to

	bool alarmSetButtonReset = false;

	if(alarmSetMode) {

		bool displayBlink = false;

		do {											// while the alarm set button is not held down, blink display.

			// Check to make sure the user has released the set button from the initial hold
			if(HAL_GPIO_ReadPin(alarmSetButtonPort, alarmSetButtonPin) == GPIO_PIN_SET) {
				alarmSetButtonReset = true;
			}

			updateAndDisplayAlarm();

			if(__HAL_TIM_GET_COUNTER(timerDelay) - timerVal >= (65536 / 2)) {

				sevSeg_setIntensity(sevSeg_intensityDuty[displayBlink]);		// Initialize to whatever duty cycle

				timerVal = __HAL_TIM_GET_COUNTER(timerDelay);
				displayBlink = !displayBlink;

			}

		}while((HAL_GPIO_ReadPin(alarmSetButtonPort, alarmSetButtonPin) != GPIO_PIN_RESET)
				|| !alarmSetButtonReset);

		sevSeg_setIntensity(sevSeg_intensityDuty[1]);			// Turn display back to 50% intensity

		HAL_TIM_Base_Stop(timerDelay);

		updateAndDisplayTime();

	}

	alarmSetMode = false;		// We have exited alarm set mode

	//printf("Current time back to %u:%u:%u.\n\r", currTime.Hours, currTime.Minutes, currTime.Seconds);

	return halRet;

}

/*
 * Increment either user alarm hour value or RTC hour value
 */
HAL_StatusTypeDef hourSetISR(void) {



	HAL_StatusTypeDef halRet = HAL_OK;

	if(alarmSetMode) {	// If the clock is in alarm set mode, change user alarm time hour

		alarmHourInc();

	}
	else {									// Otherwise, change current time hour.

		currHourInc();

		HAL_RTC_SetTime(&hrtc, &currTime, RTCTimeFormat);

		updateAndDisplayTime();

		getRTCTime(&hrtc, &currTime, &currDate);

		//printf("Current time hour incremented to %u:%u:%u.\n\r", currTime.Hours,
				//currTime.Minutes, currTime.Seconds);
	}

	return halRet;

}

/*
 * Increment either user alarm minute value or RTC minute value
 */
HAL_StatusTypeDef minuteSetISR(void) {



	HAL_StatusTypeDef halRet = HAL_OK;

	if(alarmSetMode) {	// If the clock is in alarm set mode, change user alarm time hour

		alarmMinuteInc();

	}
	else {									// Otherwise, change current time hour.

		currMinuteInc();

		HAL_RTC_SetTime(&hrtc, &currTime, RTCTimeFormat);

		/*
		 * Change internal RTC alarm to keep it triggering
		 */

		RTC_AlarmTypeDef sAlarm = {0};
		HAL_RTC_GetAlarm(&hrtc, &sAlarm, internalAlarm, RTCTimeFormat);

		if(sAlarm.AlarmTime.Minutes>58) {
			sAlarm.AlarmTime.Minutes=0;
			//printf("Reset alarm time\n\r");
		} else {
			sAlarm.AlarmTime.Minutes=sAlarm.AlarmTime.Minutes+1;
		}
		while(HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, FORMAT_BIN)!=HAL_OK){}

		updateAndDisplayTime();

		getRTCTime(&hrtc, &currTime, &currDate);

		//printf("Current time minute incremented to %u:%u:%u.\n\r", currTime.Hours,
				//currTime.Minutes, currTime.Seconds);
	}


	return halRet;
}

/*
 * Switch hour format (12 or 24 hr)
 */
HAL_StatusTypeDef timeFormatSwitchISR(void) {

	HAL_StatusTypeDef halRet = HAL_OK;

	// Determine which time format we will be using
	if(HAL_GPIO_ReadPin(timeFormatSwitchPort, timeFormatSwitchPin) == userTimeFormatGPIO_12) {
	  userTimeFormat = RTC_HOURFORMAT_12;
	}
	else {
	  userTimeFormat = RTC_HOURFORMAT_24;
	}

	updateAndDisplayTime();

	return halRet;

}

/*
 * Increment user alarm time hour
 */
void alarmHourInc(void) {

	if(userAlarmTime.Hours >= 12) {
		userAlarmTime.Hours = 1;
	}
	else if(userAlarmTime.Hours == 11) {
		if(userAlarmTime.TimeFormat == RTC_HOURFORMAT12_AM) {
			userAlarmTime.TimeFormat = RTC_HOURFORMAT12_PM;
		}
		else {
			userAlarmTime.TimeFormat = RTC_HOURFORMAT12_AM;
		}
		userAlarmTime.Hours = 12;
	}
	else if(userAlarmTime.Hours < 11) {
		userAlarmTime.Hours = userAlarmTime.Hours + 1;
	}
	else {
		__NOP();
	}

	// Update RTC backup registers with new user alarm time
	updateRTCBackupReg();

}

/*
 * Increment current time hour
 */
void currHourInc(void) {

	getRTCTime(&hrtc, &currTime, &currDate);

	if(currTime.Hours >= 12) {
		currTime.Hours = 1;
	}
	else if(currTime.Hours == 11) {
		if(currTime.TimeFormat == RTC_HOURFORMAT12_AM) {
			currTime.TimeFormat = RTC_HOURFORMAT12_PM;
		}
		else {
			currTime.TimeFormat = RTC_HOURFORMAT12_AM;
		}
		currTime.Hours = 12;
	}
	else if(userAlarmTime.Hours < 11) {
		currTime.Hours = currTime.Hours + 1;
	}
	else {
		__NOP();
	}

	// Reset seconds
	currTime.Seconds = 0;
	currTime.SecondFraction = 0;

}

/*
 * Increment User alarm time minute
 */
void alarmMinuteInc(void) {

	if(userAlarmTime.Minutes >= 59) {
		/*
		 * The below function call is the old version in which
		 * the hour will increment when the minutes roll over.
		 */
		//alarmHourInc();
		userAlarmTime.Minutes = 0;
	}
	else if(userAlarmTime.Minutes < 59) {
		userAlarmTime.Minutes = userAlarmTime.Minutes + 1;
	}
	else {
		__NOP();
	}

	// Update RTC backup registers with new user alarm time
	updateRTCBackupReg();

}

/*
 * Increment current time minute
 */
void currMinuteInc(void) {

	getRTCTime(&hrtc, &currTime, &currDate);

	if(currTime.Minutes >= 59) {

		/*
		 * The below function call is the old version in which
		 * the hour will increment when the minutes roll over.
		 */
		// currHourInc();

		currTime.Minutes = 0;
	}
	else if(currTime.Minutes < 59) {
		currTime.Minutes = currTime.Minutes + 1;
	}
	else {
		__NOP();
	}

	// Reset seconds
	currTime.Seconds = 0;
	currTime.SecondFraction = 0;

}

/*
 * Displays a non-critical fault to indicate reduced functionality
 */
void dispFault(void) {
	HAL_GPIO_WritePin(debugLEDPort, debugLEDPin, GPIO_PIN_SET);
}

/*
 * Displays a critical failure and ceases all operations
 */
void dispFailure(void) {

	HAL_TIM_Base_Stop(timerDelay);
	HAL_TIM_Base_Start(timerDelay);						// Begin timer 16 counting (to 500 ms)
	uint32_t timerVal = __HAL_TIM_GET_COUNTER(timerDelay);	// Get initial timer value to compare to
	bool displayBlink = false;


	do {

		if(__HAL_TIM_GET_COUNTER(timerDelay) - timerVal >= (65535 / 4)) {		// Use hardware timer to blink/beep display

			HAL_GPIO_TogglePin(debugLEDPort, debugLEDPin);

			timerVal = __HAL_TIM_GET_COUNTER(timerDelay);				// Update timer value

			displayBlink = !displayBlink;							// Toggle display blink counter



		}


	} while(1);

}

/*
 * Updates RTC backup register with user alarm time to be later pulled from in the case of a power outage
 */
void updateRTCBackupReg(void) {

	HAL_RTCEx_BKUPWrite(&hrtc, userAlarmHourBackupReg, userAlarmTime.Hours);
	HAL_RTCEx_BKUPWrite(&hrtc, userAlarmMinuteBackupReg, userAlarmTime.Minutes);
	HAL_RTCEx_BKUPWrite(&hrtc, userAlarmTFBackupReg, userAlarmTime.TimeFormat);

}

/*
 * Converts the time in an AM/PM RTC time to 24-hour time
 * Returns this time in an RTC Time Def object
 */
RTC_TimeTypeDef conv2Mil(RTC_TimeTypeDef *oldTime) {

	RTC_TimeTypeDef convertedTime = *oldTime;

	// If we are in PM, increment hours by 12 (but not if it is 12 p.m.)
	if(oldTime->TimeFormat == RTC_HOURFORMAT12_PM && oldTime->Hours != 12) {
		convertedTime.Hours += 12;
	}
	// If we are in AM and the hours are 12, set hours to 0.
	else if(oldTime->TimeFormat == RTC_HOURFORMAT12_AM && oldTime->Hours == 12) {
		convertedTime.Hours = 0;
	}
	else { // No change

	}

	// Make the update and display time function think that it is AM always
	convertedTime.TimeFormat = RTC_HOURFORMAT12_AM;

	return convertedTime;

}

/*
 * Begins DMA streams to pull data from memory, process data, and push to i2s amplifier.
 */
void startAudioStream(void) {

	// Pre-fill TX buffer
	fillTxBuffer(0);
	fillTxBuffer(BUFFER_SIZE);

	// Start TX DMA stream
	HAL_I2S_Transmit_DMA(&hi2s1, i2sTxBuff, BUFFER_SIZE * 2);

	// Enable Amplifier
	NAU8315YG_AmpEnable(&i2sAmp);

	// Interrupts will take care of the rest.

}

/*
 * Halts DMA streams
 */
void stopAudioStream(void) {

	// Disable Amplifier
	NAU8315YG_AmpDisable(&i2sAmp);

	// Stop DMA Stream
	HAL_I2S_DMAStop(&hi2s1);

}

/*
 * DMA completion callbacks
 */

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {

	// Fill first half of i2s TX buffer
	fillTxBuffer(0);


}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {

	// Fill second half of i2s transmit buffer
	fillTxBuffer(BUFFER_SIZE);


}

void fillTxBuffer(uint16_t offset) {

	// Read next chunk of audio data, increment flash read address
	W25Q_readData(&spiFlash, flashReadAddr, BUFFER_SIZE, spiRxBuff);
	flashReadAddr += BUFFER_SIZE;

	// Playing all of a mono file canS-mono-reduced
	 for(uint16_t i = 0; i < BUFFER_SIZE; i += 2) {

		 i2sTxBuff[offset + (i) + 1] = (spiRxBuff[i + 1] << 8) | spiRxBuff[i];

	 }


	// If we have reached the end of the audio clip, reset flash read address
	if(flashReadAddr > audioAddr_END) {
		flashReadAddr = initialMemoryOffset;
	}


}

void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s) {

	//TEST?
	__NOP();


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
