/*
 * alarm.c
 *
 *  Created on: Jul 5, 2023
 *      Author: marka
 */

#include "../Inc/alarm.h"


HAL_StatusTypeDef initRTCInternalAlarm(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *currTime, RTC_DateTypeDef *currDate) {

	HAL_StatusTypeDef halRet = HAL_OK;

	RTC_AlarmTypeDef internalAlarm_init = {0};
	internalAlarm_init.AlarmTime.Hours = currTime->Hours;
	internalAlarm_init.AlarmTime.Minutes = currTime->Minutes + 1;
	internalAlarm_init.AlarmTime.Seconds = currTime->Seconds;
	internalAlarm_init.AlarmTime.SubSeconds = currTime->SubSeconds;
	internalAlarm_init.AlarmTime.TimeFormat = currTime->TimeFormat;
	internalAlarm_init.AlarmTime.TimeFormat = currTime->TimeFormat;
	internalAlarm_init.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	internalAlarm_init.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
	internalAlarm_init.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY|RTC_ALARMMASK_HOURS
								  |RTC_ALARMMASK_SECONDS;
	internalAlarm_init.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
	internalAlarm_init.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
	internalAlarm_init.AlarmDateWeekDay = 0x1;
	internalAlarm_init.Alarm = internalAlarm;

	halRet = HAL_RTC_SetAlarm_IT(hrtc, &internalAlarm_init, RTCTimeFormat);
	if(halRet != HAL_OK)
		return halRet;

	RTC_AlarmTypeDef internalAlarm_initTest;
	halRet = HAL_RTC_GetAlarm(hrtc, &internalAlarm_initTest, internalAlarm, RTCTimeFormat);
	return halRet;

}

void getRTCTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *currTime, RTC_DateTypeDef *currDate) {

	// Store both current time and current date in time and date pointers.
	HAL_RTC_GetTime(hrtc, currTime, RTCTimeFormat);
	HAL_RTC_GetDate(hrtc, currDate, RTCTimeFormat);

}

