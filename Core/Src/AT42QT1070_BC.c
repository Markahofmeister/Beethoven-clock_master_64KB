/*
 * AT42QT1070.c
 *
 * I2C driver for Atmel AT42QT1070 capacitive touch sensor
 *
 *  Created on: May 5, 2024
 *      Author: Mark Hofmeister
 *
 */

#include "AT42QT1070.h"


uint8_t capTouch_Init(QT1070 *capTouch, I2C_HandleTypeDef *hi2c, TIM_HandleTypeDef *htimDelay,
					GPIO_TypeDef **capTouchResetPort, uint16_t capTouchResetPin, uint8_t keyEnFlags) {

	HAL_StatusTypeDef halRet = HAL_OK;

	// Assign handlers and GPIO pins for specific instance
	capTouch->hi2c = hi2c;

	capTouch->delayTimer = htimDelay;

	capTouch->resetPort = capTouchResetPort;
	capTouch->resetPin = capTouchResetPin;

	// Hardware reset device
	// Assumes active-low hardware configuration
	HAL_GPIO_WritePin(*capTouch->resetPort, capTouch->resetPin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(*capTouch->resetPort, capTouch->resetPin, GPIO_PIN_SET);

	/*
	 *  Delay for 500 ms using hardware timer
	 */
	HAL_TIM_Base_Stop(capTouch->delayTimer);
	HAL_TIM_Base_Start(capTouch->delayTimer);							// Begin timer counting
	uint32_t timerVal = __HAL_TIM_GET_COUNTER(capTouch->delayTimer);	// Get initial timer value to compare to

	//Hang in dead loop until 500 ms
	while(__HAL_TIM_GET_COUNTER(capTouch->delayTimer) - timerVal <= (65535 / 2)){ }

    // Determines the number of times to try each initialization step
	uint8_t numTries = 3;

	// Verify device ID
		uint8_t deviceIDRet = 0x00;

		while(numTries != 0) {
			halRet = capTouch_ReadDeviceID(capTouch, &deviceIDRet);

			if (halRet == HAL_OK)
				break;

			numTries--;
		}
		if(deviceIDRet != DEVICE_ID || numTries == 0) {
			return 1;
		}

		capTouch->deviceID = deviceIDRet;

		numTries = 3;

	// Force Device Recalibration
		while(numTries != 0) {
			halRet = capTouch_Recalibrate(capTouch);

			// Wait until calibration sequence completes
			while(capTouch_checkCal(capTouch)) {}

			if (halRet == HAL_OK)
				break;

			numTries--;

		}
		if(numTries == 0) {
			return 2;
		}

		numTries = 3;

	// Get initial reading of channels
		while(numTries != 0) {

			halRet = capTouch_readChannels(capTouch);

			if (halRet == HAL_OK)
				break;

			numTries--;

		}
		if(numTries == 0) {
			return 3;
		}

		numTries = 3;

	// Enable keys based on passed initialization byte
		while(numTries != 0) {

			halRet = capTouch_enableKeys(capTouch, keyEnFlags);

			if (halRet == HAL_OK)
				break;

			numTries--;

		}
		if(numTries == 0) {
			return 4;
		}

		capTouch->keys = keyEnFlags;

		numTries = 3;

	return 0;

}

HAL_StatusTypeDef capTouch_ReadDeviceID(QT1070 *capTouch, uint8_t *dataBuff) {

	uint8_t deviceIDRet_I2C = 0x00;

	HAL_StatusTypeDef halRet = HAL_OK;

	uint8_t capTouch_DeviceIDReg_TX[1] = {capTouch_DeviceIDReg};

	halRet = HAL_I2C_Master_Transmit(capTouch->hi2c, DEVICE_ADDRESS,
										&(capTouch_DeviceIDReg_TX[0]), 1, HAL_MAX_DELAY);
	if(halRet != HAL_OK)
		return halRet;
	halRet = HAL_I2C_Master_Receive(capTouch->hi2c, DEVICE_ADDRESS, &deviceIDRet_I2C, 1, HAL_MAX_DELAY);

	*dataBuff = deviceIDRet_I2C;

	return halRet;

}

HAL_StatusTypeDef capTouch_Recalibrate(QT1070 *capTouch) {

	HAL_StatusTypeDef halRet = HAL_OK;

	/*
	 * Writing any non-zero value to the calibrate register
	 * will enter a recalibration mode
	 */

	uint8_t deviceCal[2] = {capTouch_CalibrateReg, 0xFF};
	halRet = HAL_I2C_Master_Transmit(capTouch->hi2c, DEVICE_ADDRESS,
									deviceCal, 2, HAL_MAX_DELAY);

	return halRet;

}

uint8_t capTouch_checkCal(QT1070 *capTouch) {

	HAL_StatusTypeDef halRet = HAL_OK;

	uint8_t detectionStatusRet = 0x00;
	uint8_t calibrationFlag;

	uint8_t capTouch_DetectionStatusReg_I2C[1] = {capTouch_DetectionStatusReg};

	halRet = HAL_I2C_Master_Transmit(capTouch->hi2c, DEVICE_ADDRESS,
									&(capTouch_DetectionStatusReg_I2C[0]), 1, HAL_MAX_DELAY);
	if(halRet != HAL_OK)
		return halRet;
	halRet = HAL_I2C_Master_Receive(capTouch->hi2c, DEVICE_ADDRESS, &detectionStatusRet, 1, HAL_MAX_DELAY);

	calibrationFlag = (detectionStatusRet & 0b10000000) >> 7;

	return calibrationFlag;

}

HAL_StatusTypeDef capTouch_readChannels(QT1070 *capTouch) {

	HAL_StatusTypeDef halRet = HAL_OK;

	uint8_t keyStatusRet = 0x00;

	uint8_t keyStatReg[1] = {capTouch_KeyStatusReg};

	halRet = HAL_I2C_Master_Transmit(capTouch->hi2c, DEVICE_ADDRESS,
									&(keyStatReg[0]), 1, HAL_MAX_DELAY);
	if(halRet != HAL_OK)
		return halRet;
	halRet = HAL_I2C_Master_Receive(capTouch->hi2c, DEVICE_ADDRESS, &keyStatusRet, 1, HAL_MAX_DELAY);

	capTouch->keyStat = keyStatusRet & 0b01111111;

	return halRet;

}

HAL_StatusTypeDef capTouch_enableKeys(QT1070 *capTouch, uint8_t dataBuff) {

	HAL_StatusTypeDef halRet = HAL_OK;

	/*
	 *  Read in current averaging values for each key
	 *  The device will automatically increment register addressing for subsequent reads,
	 *  so only one Master Transmit call is required.
	 */
	uint8_t avgRet[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	uint8_t avgRegs[7] = {capTouch_AvgFactor0Reg, capTouch_AvgFactor1Reg, capTouch_AvgFactor2Reg,
			capTouch_AvgFactor3Reg, capTouch_AvgFactor4Reg, capTouch_AvgFactor5Reg, capTouch_AvgFactor6Reg};

	halRet = HAL_I2C_Master_Transmit(capTouch->hi2c, DEVICE_ADDRESS,
								&(avgRegs[0]), 1, HAL_MAX_DELAY);
	if(halRet != HAL_OK)
		return halRet;

	halRet = HAL_I2C_Master_Receive(capTouch->hi2c, DEVICE_ADDRESS, avgRet, 7, HAL_MAX_DELAY);
	if(halRet != HAL_OK)
		return halRet;

	uint8_t i;

	for(i = 0; i <= 6; i++) {

		// If the averaging factor is to be enabled but was previously disabled,
		// set it back to the default value of 8.
//		if ( ((avgRet[i] >> 2) == 0x00) && (((dataBuff >> i) & 0b00000001) == 1) ) {
//			avgRet[i] = avgRet[i] | 0b00100000;
//		}

		// New value to pass only changes bits 2-6.
		// If bit i of dataBuff = 0, set these bits to 0, else leave them.
		avgRet[i] = (avgRet[i] >> 2) * ((dataBuff >> i) & 0b00000001);

	}

	halRet = capTouch_SetAveragingFactor(capTouch, avgRet);

	return halRet;

}

HAL_StatusTypeDef capTouch_SetAveragingFactor(QT1070 *capTouch, uint8_t *dataBuff) {

	HAL_StatusTypeDef halRet = HAL_OK;

	/*
	 *  Read in current averaging values for each key
	 *  The device will automatically increment register addressing for subsequent reads,
	 *  so only one Master Transmit call is required.
	 */
	uint8_t avgRet[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	uint8_t avgRegs[7] = {capTouch_AvgFactor0Reg, capTouch_AvgFactor1Reg, capTouch_AvgFactor2Reg,
			capTouch_AvgFactor3Reg, capTouch_AvgFactor4Reg, capTouch_AvgFactor5Reg, capTouch_AvgFactor6Reg};

	halRet = HAL_I2C_Master_Transmit(capTouch->hi2c, DEVICE_ADDRESS,
								&(avgRegs[0]), 1, HAL_MAX_DELAY);
	if(halRet != HAL_OK)
		return halRet;
	halRet = HAL_I2C_Master_Receive(capTouch->hi2c, DEVICE_ADDRESS, avgRet, 7, HAL_MAX_DELAY);
	if(halRet != HAL_OK)
		return halRet;


	// 2-byte buffer to specify register address and new averaging factor
	uint8_t avgRegNew[2] = {0x00, 0x00};
	uint8_t avgNew = 0x00;
	uint8_t i;

	for(i = 0; i <= 6; i++) {

		/*
		 * New register value should only modify bits 2-6.
		 * Bits 0-1 contain adjacent key suppression information,
		 * which is independent of the averaging factor.
		 */

		// Clear bits 2-6
		avgNew = avgRet[i] & 0b00000011;
		// Set bits 2-6 with new averaging factor
		uint8_t avgMask = ((dataBuff[i]) << 2);
		avgNew = avgNew | avgMask;

		// Throw error if the requested averaging factor is not a power of 2
//		if(!(ceil(log2(avgNew)) == floor(log2(avgNew)))) {
//			halRet = HAL_ERROR;
//			return halRet;
//		}
		// ^^ Kills memory

		avgRegNew[0] = avgRegs[i];
		avgRegNew[1] = avgNew;

		halRet = HAL_I2C_Master_Transmit(capTouch->hi2c, DEVICE_ADDRESS,
									avgRegNew, 2, HAL_MAX_DELAY);
		if(halRet != HAL_OK)
			return halRet;

	}

	// Debug check that avg register values have been set successfully
	halRet = HAL_I2C_Master_Transmit(capTouch->hi2c, DEVICE_ADDRESS,
									&(avgRegs[0]), 1, HAL_MAX_DELAY);
	if(halRet != HAL_OK)
		return halRet;
	halRet = HAL_I2C_Master_Receive(capTouch->hi2c, DEVICE_ADDRESS, avgRet, 7, HAL_MAX_DELAY);

	return halRet;

}

HAL_StatusTypeDef capTouch_SetDetectionIntegrator(QT1070 *capTouch, uint8_t *dataBuff) {

	HAL_StatusTypeDef halRet = HAL_OK;

	uint8_t detIntRegs[7] = {capTouch_DetInteg0Reg, capTouch_DetInteg1Reg, capTouch_DetInteg2Reg,
						capTouch_DetInteg3Reg, capTouch_DetInteg4Reg, capTouch_DetInteg5Reg, capTouch_DetInteg6Reg};

	// 2-byte buffer to specify register address and new averaging factor
	uint8_t detIntRegNew[2] = {0x00, 0x00};
	uint8_t i;

	for(i = 0; i <= 6; i++) {

		detIntRegNew[0] = detIntRegs[i];
		detIntRegNew[1] = dataBuff[i];

		halRet = HAL_I2C_Master_Transmit(capTouch->hi2c, DEVICE_ADDRESS,
										detIntRegNew, 2, HAL_MAX_DELAY);

	}

	// Debug check that avg register values have been set successfully
	uint8_t detIntRet[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	halRet = HAL_I2C_Master_Transmit(capTouch->hi2c, DEVICE_ADDRESS,
									&(detIntRegs[0]), 1, HAL_MAX_DELAY);
	halRet = HAL_I2C_Master_Receive(capTouch->hi2c, DEVICE_ADDRESS, detIntRet, 7, HAL_MAX_DELAY);

	return halRet;

}
