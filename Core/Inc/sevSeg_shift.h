/*
 * sevSeg_shift.h
 *
 *  Created on: Jul 25, 2023
 *      Author: marka
 */

#ifndef INC_SEVSEG_SHIFT_H_
#define INC_SEVSEG_SHIFT_H_

#ifndef STM32G0XX_HAL_H
#include "stm32g0xx_hal.h"
#endif

#ifndef STDIO_H
#include <stdio.h>
#endif

//typedef struct {
//
//	uint16_t shiftDataPin;
//	uint16_t shiftDataClockPin;
//	uint16_t shiftStoreClockPin;
//	uint16_t shiftMCLRPin;
//	GPIO_TypeDef
//
//} quadSevSeg;

/*
 * - Clears any existing shift register data
 * - Sets intensity to 50%
 * - Flashes an initializing "Hof" symbol
 */

void sevSeg_Init(uint16_t shiftDataPin, uint16_t shiftDataClockPin, uint16_t shiftStoreClockPin,
					uint16_t shiftOutputEnablePin, uint16_t shiftMCLRPin,
					GPIO_TypeDef **GPIOPortArray, TIM_HandleTypeDef *htim, TIM_HandleTypeDef *htim_PWM_pass,
					uint32_t tim_PWM_CHANNEL_pass);


void sevSeg_updateDigits(RTC_TimeTypeDef *updateTime, uint8_t userAlarmEnable);


void sevSeg_setIntensity(uint16_t dutyCycle);


#endif /* INC_SEVSEG_SHIFT_H_ */
