/*
 * alarm.h
 *
 *  Created on: Jul 5, 2023
 *      Author: marka
 *
 *  Drives STM32 RTC
 *  Works with STM32 HAL
 */

#ifndef INC_ALARM_H_
#define INC_ALARM_H_

#ifndef STM32G0XX_HAL_H
#include "stm32g0xx_hal.h"
#endif

#ifndef STDIO_H
#include <stdio.h>
#endif

/*
 * Alarm specification macros
 */
#define internalAlarm 	RTC_ALARM_A
#define RTCTimeFormat	RTC_FORMAT_BIN


/*
 * Declare RTC time and date to defaults and update RTC with values configured in .c file.
 *
 * @param STM32 HAL RTC handling object pointer
 * @param STM32 HAL RTC time object pointer (storing current RTC time.)
 * @param STM32 HAL RTC date object pointer (storing current RTC date.)
 *
 * @ret none
 */
HAL_StatusTypeDef initRTCInternalAlarm(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *currTime, RTC_DateTypeDef *currDate);

/*
 * Gets updated RTC time and stores in access object pointers
 *
 * @param STM32 HAL RTC handling object pointer
 * @param STM32 HAL RTC time object pointer (storing current RTC time.)
 * @param STM32 HAL RTC date object pointer (storing current RTC date.)
 *
 * @ret none
 */
void getRTCTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *currTime, RTC_DateTypeDef *currDate);


#endif /* INC_ALARM_H_ */
