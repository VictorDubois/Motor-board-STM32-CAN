/*
 * callbacks.cpp
 *
 *  Created on: Jun 8, 2026
 *      Author: victor.dubois@navya.tech
 */

#include <string.h>
#include <cstring>
#include <string>

#include <CanStruct/can_structs.h>
#include <uartBroker.h>


#include "constants.h"

#include "main.h" // for pins definitions
#include "mainpp.h" // for MotorBoard

void motors_cmd_cb(const CAN::MotorBoardCmdInput &motors_cmd_msg)
{
	if (motors_cmd_msg.reset_encoders)
	{
		MotorBoard::set_odom(0, 0, 0);
	}

	MotorBoard::getDCMotor().set_enable_motors((bool)motors_cmd_msg.enable_motors);

	if (!motors_cmd_msg.enable_motors) {
		//MotorBoard::getDCMotor().resetMotors();
		MotorBoard::getDCMotor().resetMotor(M_L);
		MotorBoard::getDCMotor().resetMotor(M_R);

		return;
	}

	if (motors_cmd_msg.override_PWM)
	{
		MotorBoard::getDCMotor().override_PWM(motors_cmd_msg.PWM_override_left, motors_cmd_msg.PWM_override_right);
	}
	else
	{
		MotorBoard::getDCMotor().stop_pwm_override();
	}
}

void motors_cmd_cb(const krabi_msgs::motors_cmd &motors_cmd_msg)
{
	CAN::MotorBoardCmdInput motors_cmd_can;
	motors_cmd_can.PWM_override_left = motors_cmd_msg.PWM_override_left;
	motors_cmd_can.PWM_override_right = motors_cmd_msg.PWM_override_right;
	motors_cmd_can.enable_motors = motors_cmd_msg.enable_motors;
	motors_cmd_can.override_PWM = motors_cmd_msg.override_PWM;
	motors_cmd_can.reset_encoders = motors_cmd_msg.reset_encoders;

	motors_cmd_cb(motors_cmd_can);
}

void digital_outputs_cb(const CAN::DigitalOutputs &digital_outputs_msg)
{
	GPIO_PinState trans_0 = GPIO_PIN_RESET;
	GPIO_PinState trans_1 = GPIO_PIN_RESET;
	GPIO_PinState trans_2 = GPIO_PIN_RESET;
	GPIO_PinState trans_3 = GPIO_PIN_RESET;
	if(digital_outputs_msg.enable_outputs & (1<<9))
	{
		trans_0 = GPIO_PIN_SET;
	}
	if(digital_outputs_msg.enable_outputs & (1<<11))
	{
		trans_1 = GPIO_PIN_SET;
	}
	if(digital_outputs_msg.enable_outputs & (1<<13))
	{
		trans_2 = GPIO_PIN_SET;
	}
	if(digital_outputs_msg.enable_outputs & (1<<15))
	{
		trans_3 = GPIO_PIN_SET;
	}

	HAL_GPIO_WritePin(Trans0_GPIO_Port, Trans0_Pin, trans_0);
	HAL_GPIO_WritePin(Trans1_GPIO_Port, Trans1_Pin, trans_1);
	HAL_GPIO_WritePin(Trans2_GPIO_Port, Trans2_Pin, trans_2);
	HAL_GPIO_WritePin(Trans3_GPIO_Port, Trans3_Pin, trans_3);
}

void parameters_cb(const krabi_msgs::motors_parameters& a_parameters)
{
	MotorBoard::getDCMotor().set_max_current(a_parameters.max_current);
	MotorBoard::getDCMotor().set_max_current(a_parameters.max_current_left, a_parameters.max_current_right);
}

void cmd_vel_cb(const geometry_msgs::Twist& twist)
{
	MotorBoard::getDCMotor().set_speed_order(metersToTicks(twist.linear.x), radsToTicks(-twist.angular.z));
}

void enable_motor_cb(const std_msgs::Bool& enable)
{
	if(!enable.data) {
		MotorBoard::getDCMotor().resetMotors();
	}
}
