/*
 * sevSeg_shift.c
 *
 *  Created on: Jul 23, 2023
 *      Author: marka
 */

#include <sevSeg_shift.h>

/*
 * Digit 0 = LSB (ones place of minutes)
 * .
 * .
 * Digit 3 = MSB (tenths place of hours)
 */

// Binary codes for each digit in the form of a 7-segment display.
// This is reversed - i.e., the activation is active low for Beethoven clock
// Bit 1 in each is set to 1 because it is the decimal point and is changed in the display functions if necessary.
const uint8_t dispDigits[10] = {0b01000010, 	// 0
								0b11011110,		// 1
								0b10000011,		// 2
								0b10001010,		// 3
								0b00011110,		// 4
								0b00101010,		// 5
								0b00100010,		// 6
								0b11001110,		// 7
								0b00000010,		// 8
								0b00001010};	// 9

/*
 * Global variables to be initialized with GPIO assignments
 */
uint16_t shiftData;
uint16_t shiftDataClock;
uint16_t shiftStoreClock;
uint16_t shiftOutputEnable;
uint16_t shiftMCLR;

/*
 * Global variable to be initialized with delay timer
 */
TIM_HandleTypeDef htim_PWM;
uint32_t tim_PWM_CHANNEL_shift;

/*
 * Array of GPIO ports to be initialized for each shift register GPIO
 * GPIO Port A values are placeholders
 * Order:
 * [0] = shift data pin port
 * [1] = shift data clock pin port
 * [2] = shift store clock pin port
 * [3] = shift output enable pin port
 * [4] = shift master clear pin port
 */
GPIO_TypeDef *portArray[5] = {GPIOA, GPIOA, GPIOA, GPIOA, GPIOA};

// Used to avoid conditionals
GPIO_PinState GPIOPinSet[2] = {GPIO_PIN_RESET, GPIO_PIN_SET};


void sevSeg_Init(uint16_t shiftDataPin, uint16_t shiftDataClockPin, uint16_t shiftStoreClockPin,
					uint16_t shiftOutputEnablePin, uint16_t shiftMCLRPin,
					GPIO_TypeDef **GPIOPortArray, TIM_HandleTypeDef *htim, TIM_HandleTypeDef *htim_PWM_pass,
					uint32_t tim_PWM_CHANNEL_pass) {

	shiftData = shiftDataPin;
	shiftDataClock = shiftDataClockPin;
	shiftStoreClock = shiftStoreClockPin;
	shiftOutputEnable = shiftOutputEnablePin;
	shiftMCLR = shiftMCLRPin;

	htim_PWM = *htim_PWM_pass;
	tim_PWM_CHANNEL_shift = tim_PWM_CHANNEL_pass;

	for(int i = 0; i < 5; i++) {
		portArray[i] = GPIOPortArray[i];
	}

	// Clear any existing shift register data
	HAL_GPIO_WritePin(portArray[4], shiftMCLR, GPIOPinSet[0]);
	HAL_GPIO_WritePin(portArray[4], shiftMCLR, GPIOPinSet[1]);

	// Store cleared data and Enable output
	HAL_GPIO_WritePin(portArray[2], shiftStoreClock, GPIOPinSet[1]);
	HAL_GPIO_WritePin(portArray[2], shiftStoreClock, GPIOPinSet[0]);
	HAL_GPIO_WritePin(portArray[3], shiftOutputEnable, GPIOPinSet[0]);

	// Set duty cycle to 50%

	sevSeg_setIntensity(50);

	//Flash an initializing "Hof" symbol
	uint8_t hofSymb[4] = {0b00000000, 0b00110111, 0b00011101, 0b01000111};

	uint8_t sendByte;					// To be used to shift bits

	for(int i = 0; i <= 3; i++) {

		sendByte = hofSymb[i];

		for(int j = 0; j < 8; j++) {

			// Write data pin with LSB of data
			HAL_GPIO_WritePin(portArray[0], shiftData, GPIOPinSet[sendByte & 1]);

			// Toggle clock GPIO to shift bit into register
			HAL_GPIO_WritePin(portArray[1], shiftDataClock, GPIOPinSet[1]);
			HAL_GPIO_WritePin(portArray[1], shiftDataClock, GPIOPinSet[0]);

			// Once data pin has been written and shifted out, shift data right by one bit.
			sendByte >>= 1;

		}
	}

	// Once all data has been shifted out, toggle store clock register to display data.
	HAL_GPIO_WritePin(portArray[2], shiftStoreClock, GPIOPinSet[1]);
	HAL_GPIO_WritePin(portArray[2], shiftStoreClock, GPIOPinSet[0]);

	// Delay for 500 ms using hardware timer
	HAL_TIM_Base_Stop(htim);
	HAL_TIM_Base_Start(htim);							// Begin timer counting
	uint32_t timerVal = __HAL_TIM_GET_COUNTER(htim);	// Get initial timer value to compare to

	//Hang in dead loop until 500 ms
	while(__HAL_TIM_GET_COUNTER(htim) - timerVal <= (65535 / 4)){
//		timerVal = __HAL_TIM_GET_COUNTER(htim);
	}

	HAL_TIM_Base_Stop(htim);

	// Clear any existing shift register data
	HAL_GPIO_WritePin(portArray[4], shiftMCLR, GPIOPinSet[0]);
	HAL_GPIO_WritePin(portArray[4], shiftMCLR, GPIOPinSet[1]);

	// Store cleared data and Enable output
	HAL_GPIO_WritePin(portArray[2], shiftStoreClock, GPIOPinSet[1]);
	HAL_GPIO_WritePin(portArray[2], shiftStoreClock, GPIOPinSet[0]);

}

void sevSeg_updateDigits(RTC_TimeTypeDef *updateTime, uint8_t userAlarmEnable) {

	/*
	 * Determine what time to send to shift registers
	 * digit 3 is a special case - the colons are always on, but the one can be on/off.
	 * Therefore, use array indexing to decide what to send.
	 */
	uint8_t sendTime[4] = {updateTime->Hours / 10, updateTime->Hours % 10,
							updateTime->Minutes / 10, updateTime->Minutes % 10};

	uint8_t sendByte;					// To be used to shift bits

	for(int i = 3; i >= 0; i--) {

		sendByte = dispDigits[sendTime[i]];

		// If tenth's place of hour and zero, disable this segment
		if( (i == 0) && (sendByte == dispDigits[0])) {
			sendByte = 0xFF;
		}

		// If one's place of hour, set decimal point based on AM/PM.
		// OR
		// If one's place of minute, set decimal place based on user alarm enabled
		if( ((i == 3) && (updateTime->TimeFormat == RTC_HOURFORMAT12_PM)) ||
			((i == 1) && userAlarmEnable)	) {
			sendByte = (sendByte & 0b11111101);
		}

		for(int j = 0; j < 8; j++) {

			// Write data pin with LSB of data
			HAL_GPIO_WritePin(portArray[0], shiftData, GPIOPinSet[(sendByte & 1)]);

			// Toggle clock GPIO to shift bit into register
			HAL_GPIO_WritePin(portArray[1], shiftDataClock, GPIOPinSet[1]);
			HAL_GPIO_WritePin(portArray[1], shiftDataClock, GPIOPinSet[0]);

			// Once data pin has been written and shifted out, shift data right by one bit.
			sendByte >>= 1;

		}
	}

	// Once all data has been shifted out, toggle store clock register to display data.

	HAL_GPIO_WritePin(portArray[2], shiftStoreClock, GPIOPinSet[1]);
	HAL_GPIO_WritePin(portArray[2], shiftStoreClock, GPIOPinSet[0]);

	return;

}

void sevSeg_setIntensity(uint16_t dutyCycle) {

	__HAL_TIM_SET_COMPARE(&htim_PWM, tim_PWM_CHANNEL_shift, dutyCycle);
	HAL_TIM_PWM_Start(&htim_PWM, tim_PWM_CHANNEL_shift);

}









