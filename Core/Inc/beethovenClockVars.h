/*
 * vars.h
 *
 *  Created on: Jul 5, 2023
 *      Author: marka
 *
 *  No functions - just variable declarations specifically tailored to sinking clock.
 *  Written for STM32G0xx.
 */

#ifndef INC_BEETHOVENCLOCKVARS_H_
#define INC_BEETHOVENCLOCKVARS_H_

#ifndef STM32G0XX_HAL_H
#include "stm32g0xx_hal.h"
#endif

#ifndef STDIO_H
#include <stdio.h>
#endif

#ifndef STDBOOL_H
#include <stdbool.h>
#endif

#ifndef STRING_H
#include <string.h>
#endif

#define RTCTimeFormat RTC_FORMAT_BIN

/*
 * Declare variables to map button presses to GPIOs
 */
// EXTI Pins
const uint16_t displayButtonPin = GPIO_PIN_3;
GPIO_TypeDef *displayButtonPort = GPIOD;

const uint16_t alarmEnableButtonPin = GPIO_PIN_2;
GPIO_TypeDef *alarmEnableButtonPort = GPIOD;

const uint16_t alarmSetButtonPin = GPIO_PIN_15;
GPIO_TypeDef *alarmSetButtonPort = GPIOA;

const uint16_t hourSetButtonPin = GPIO_PIN_0;
GPIO_TypeDef *hourSetButtonPort = GPIOD;

const uint16_t minuteSetButtonPin = GPIO_PIN_1;
GPIO_TypeDef *minuteSetButtonPort = GPIOD;

const uint16_t timeFormatSwitchPin = GPIO_PIN_4;
GPIO_TypeDef *timeFormatSwitchPort = GPIOB;

// Capacitive Touch Reset Pin
const uint16_t capTouchResetPin = GPIO_PIN_8;
GPIO_TypeDef *capTouchResetPort = GPIOB;

//TODO: Add Cap touch change pin

// Specifies which channels of cap. touch IC to be used
uint8_t capTouchChannels = 0b00001111;

/*
 * Specifies averaging values for each cap. touch IC channel
 * This value will be right-shifted twice and must be a power of 2
 */
uint8_t AVGFact = 4;

// Specifies detection integration values for each cap. touch IC channel
uint8_t DIFact = 0x04;


/*
 * GPIO Pins for shift data
 * Array of ports to map each GPIO onto
 */
const uint16_t shiftDataPin = GPIO_PIN_9;
const uint16_t shiftDataClockPin = GPIO_PIN_10;
const uint16_t shiftStoreClockPin = GPIO_PIN_7;
const uint16_t shiftOutputEnablePin = GPIO_PIN_6;
const uint16_t shiftMCLRPin = GPIO_PIN_11;
GPIO_TypeDef *GPIOPortArray[5] = {GPIOA, GPIOA, GPIOC, GPIOC, GPIOA};

/*
 * Array of all duty cycles used - 0%, 50%, 100%.
 */

const uint8_t sevSeg_intensityDuty[2] = {100, 50};

/*
 * Toggle Variables
 */

// Display Toggle: 0 = 0% display Intensity, 1 = 50% display brightness, 2 = 100% display brightness
uint8_t displayToggle;

// Variable to toggle user alarm on or off. Default to off.
bool userAlarmToggle;

/*
 * RTC Calibration
 */

//const uint16_t RTC_CLK_OUT_Pin = GPIO_PIN_3;
//GPIO_TypeDef *RTC_CLK_OUT_Port = GPIOC;

//const uint16_t RTC_CLK_ADC_IN_Pin = GPIO_PIN_2;
//GPIO_TypeDef *RTC_CLK_ADC_IN_Port = GPIOA;

// Debug LED

const uint16_t debugLEDPin = GPIO_PIN_12;
GPIO_TypeDef *debugLEDPort = GPIOB;


// RTC Calibration Value
uint32_t rtcCalVal = 0x012C;


/*
 * ST Peripheral Handles
 */
uint32_t tim_PWM_CHANNEL = TIM_CHANNEL_3;

/*
 * RTC access objects
 */

RTC_TimeTypeDef currTime = {0};
RTC_DateTypeDef currDate = {0};
RTC_AlarmTypeDef userAlarmObj = {0};
RTC_TimeTypeDef userAlarmTime = {0};

/*
 * User alarm backup register locations
 */

uint32_t userAlarmHourBackupReg = RTC_BKP_DR0;
uint32_t userAlarmMinuteBackupReg = RTC_BKP_DR1;
uint32_t userAlarmTFBackupReg = RTC_BKP_DR2;
uint32_t bootstrapBackupReg = RTC_BKP_DR3;

/*
 * RTC Time Time format (as set by user via switch)
 */
uint32_t userTimeFormat;
GPIO_PinState userTimeFormatGPIO_12 = GPIO_PIN_SET;
GPIO_PinState userTimeFormatGPIO_24  = GPIO_PIN_RESET;

/*
 * SPI Flash memory settings
 */
GPIO_TypeDef *nCSPort = GPIOA;
GPIO_TypeDef *nWPPort = GPIOA;
GPIO_TypeDef *nHOLDPort = GPIOA;

uint32_t nCSPin = GPIO_PIN_5;
uint32_t nWPPin = GPIO_PIN_6;
uint32_t nHOLDPin = GPIO_PIN_7;

uint8_t spiFlash_devID = 0x17;
uint8_t spiFlash_isQuadChip = 1;
uint8_t spiFlash_driveStrength = 1;

/*
 * I2S Amplifier Settings
 */
GPIO_TypeDef *i2sAmp_enablePort = GPIOB;
uint32_t i2sAmp_enablePin = GPIO_PIN_1;

/*
 * Audio/DMA things
 */

#define BUFFER_SIZE 64

// Offset of flash memory (first byte to read)
#define initialMemoryOffset 0

// End of audio data in flash memory
#define audioAddr_END 0x890E0E

/*
 * RCR value for long 10-minute snooze
 */
const uint32_t timerSnooze_RCR = 100;




#endif /* INC_BEETHOVENCLOCKVARS_H_ */
