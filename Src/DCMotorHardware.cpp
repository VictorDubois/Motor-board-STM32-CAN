/*
 * DCMotorHardware.cpp
 *
 *  Created on: May 7, 2020
 *      Author: victor
 */

#include "DCMotorHardware.h"
#include "CanStruct/can_structs.h"
extern "C" {
	#include "main.h" // Include pin definitions
}
DCMotorHardware::DCMotorHardware(TIM_TypeDef* a_encoder_right_timer,
		TIM_TypeDef* a_encoder_left_timer,
		TIM_HandleTypeDef* a_motor_right_timer,
		const int32_t a_motor_right_timer_channel,
		TIM_HandleTypeDef* a_motor_left_timer,
		const int32_t a_motor_left_timer_channel,
		FDCAN_HandleTypeDef* a_hcan)
	:hcan(a_hcan){
	dir_right_gpio_bank = DIR_B_GPIO_Port;
	dir_right_gpio = DIR_B_Pin;
	dir_left_gpio_bank = DIR_A_GPIO_Port;
	dir_left_gpio = DIR_A_Pin;
	encoder_right_timer = a_encoder_right_timer;
	encoder_left_timer = a_encoder_left_timer;
	motor_right_timer = a_motor_right_timer;
	motor_right_timer_channel = a_motor_right_timer_channel;
	motor_left_timer = a_motor_left_timer;
	motor_left_timer_channel = a_motor_left_timer_channel;

	HAL_GPIO_WritePin(BRAKE_GPIO_Port, BRAKE_Pin, GPIO_PIN_RESET);//BRAKE
	//HAL_GPIO_WritePin(BRAKE_B_GPIO_Port, BRAKE_Pin, GPIO_PIN_RESET);//BRAKE
}
DCMotorHardware::DCMotorHardware() {}
DCMotorHardware::~DCMotorHardware() {
	HAL_GPIO_WritePin(BRAKE_GPIO_Port, BRAKE_Pin, GPIO_PIN_RESET);//BRAKE
	//HAL_GPIO_WritePin(BRAKE_B_GPIO_Port, BRAKE_Pin, GPIO_PIN_RESET);//BRAKE
}

int16_t DCMotorHardware::getTicks(const uint32_t encoderId) {
	if (encoderId == M_L) {
		return -encoder_left_timer->CNT;
	}
	return encoder_right_timer->CNT;
}

void DCMotorHardware::resetEncodersCounter(){
	encoder_left_timer->CNT = 0;
	encoder_right_timer->CNT = 0;
}

uint32_t DCMotorHardware::getMilliSecondsElapsed() {
	return HAL_GetTick();
}

void DCMotorHardware::setPWM(const int32_t pwm_left, const int32_t pwm_right) {
	// CAN
	FDCAN_TxHeaderTypeDef TxHeader;
	uint8_t TxData[8];

	/* Prepare Tx Header */
	TxHeader.Identifier = 0x321;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;

	TxHeader.Identifier = CAN::can_ids::C620_CURRENT_COMMAND;

	TxData[0] = (pwm_right >> 8) & 0xFF;
	TxData[1] = (pwm_right) & 0xFF;
	TxData[2] = (pwm_left >> 8) & 0xFF;
	TxData[3] = (pwm_left) & 0xFF;
	TxData[4] = 0 & 0xFF;
	TxData[5] = 0 & 0xFF;
	TxData[6] = 0 & 0xFF;
	TxData[7] = 0 & 0xFF;
	if (HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, TxData) != HAL_OK)
	{
		/* Transmission request Error */
		//MotorBoard::getDCMotor().resetMotors();
	}

	// PWM

	// Write dir according to pwm sign
	if (pwm_left > 0) {
		HAL_GPIO_WritePin(dir_left_gpio_bank, dir_left_gpio, GPIO_PIN_SET);
	}
	else {
		HAL_GPIO_WritePin(dir_left_gpio_bank, dir_left_gpio, GPIO_PIN_RESET);
	}

	if (pwm_right > 0) {
		HAL_GPIO_WritePin(dir_right_gpio_bank, dir_right_gpio, GPIO_PIN_SET);
	}
	else {
		HAL_GPIO_WritePin(dir_right_gpio_bank, dir_right_gpio, GPIO_PIN_RESET);
	}

	// Limit to pwm boundaries and write PWM
	int32_t command = 0;
	command = MAX(pwm_left, -pwm_left);
	command = MIN(command, DUTYMAX);
	__HAL_TIM_SET_COMPARE(motor_left_timer, motor_left_timer_channel, command);

	command = MAX(pwm_right, -pwm_right);
	command = MIN(command, DUTYMAX);
	__HAL_TIM_SET_COMPARE(motor_right_timer, motor_right_timer_channel, command);

	if(pwm_left == 0 && pwm_right == 0) {
		//HAL_GPIO_WritePin(BRAKE_B_GPIO_Port, BRAKE_B_Pin, GPIO_PIN_SET);//BRAKE
		HAL_GPIO_WritePin(BRAKE_GPIO_Port, BRAKE_Pin, GPIO_PIN_SET);//BRAKE
	}
	else {
		//HAL_GPIO_WritePin(BRAKE_B_GPIO_Port, BRAKE_B_Pin, GPIO_PIN_RESET);//un-BRAKE
		HAL_GPIO_WritePin(BRAKE_GPIO_Port, BRAKE_Pin, GPIO_PIN_RESET);//un-BRAKE

	}
}
