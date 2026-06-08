/*
 * uart_broker.h
 *
 *  Created on: Jun 8, 2026
 *      Author: victor.dubois@navya.tech
 */

#ifndef INC_UARTBROKER_H_
#define INC_UARTBROKER_H_

#include "msgs.h"

#include <inttypes.h>
#include "stm32g4xx_hal.h"


#define UART_MSG_SIZE 1+ 8*10 + 1

class uartBroker
{
public:
	float get_float(const uint8_t* a_string, const int a_beginning);


	void motors_cmd_hex_cb(const uint8_t* a_message);
	void parameters_hex_cb(const uint8_t* a_message);

	void cmd_vel_hex_cb(const uint8_t* a_message);
	void enable_motor_hex_cb(const uint8_t* a_message);

	void read_serial();


	void receiveUART(UART_HandleTypeDef *huart);

	void float_to_hex(const float a_value, uint8_t* a_out, const int start_pos);

	void publish_encoders(UART_HandleTypeDef * huart2);

	// Define CRC parameters
	#define CRC_POLYNOMIAL 0x1021
	#define CRC_INITIAL_VALUE 0xFFFF

	// Function to calculate CRC
	uint16_t calculate_crc(const uint8_t *data, int length) ;

	void publish_odom_lighter(UART_HandleTypeDef * huart2);

	void doResetUart(UART_HandleTypeDef * huart2);

	void checkHeartBeat(UART_HandleTypeDef * huart2);

	void initDMA(UART_HandleTypeDef * huart2);

	krabi_msgs::encoders getEncodersMsg() { return encoders_msg;}
	krabi_msgs::motors getMotorsMsg() {return motors_msg;}
	krabi_msgs::motors_parameters getMotorsParametersMsg() {return motors_parameters_msg;}
	krabi_msgs::odom_lighter getOdomLighterMsg() {return odom_lighter_msg;}

	void setEncodersMsg(int16_t a_encoder_left, int16_t a_encoder_right) {
		encoders_msg.encoder_left = a_encoder_left;
		encoders_msg.encoder_right= a_encoder_right;
	}

	void setOdomLighterMsg(float a_X, float a_Y, float a_angleRz, float a_speedVx, float a_speedWz)
	{
		odom_lighter_msg.poseX = a_X;
		odom_lighter_msg.poseY = a_Y;
		odom_lighter_msg.angleRz = a_angleRz;
		odom_lighter_msg.speedVx = a_speedVx;
		odom_lighter_msg.speedWz = a_speedWz;
	}

	void setMotorsMsg(krabi_msgs::encoders a_encoders, uint16_t a_current_left, uint16_t a_current_right, uint32_t a_current_left_accumulated, uint16_t a_current_right_accumulated)
	{
		motors_msg.encoders = a_encoders;
		motors_msg.current_left = a_current_left;
		motors_msg.current_right = a_current_right;
		motors_msg.current_left_accumulated = a_current_left_accumulated;
		motors_msg.current_right_accumulated = a_current_right_accumulated;
	}
private:
	uint8_t rx_line_buffer[UART_MSG_SIZE];
	uint8_t tx_e_line_buffer[UART_MSG_SIZE];
	uint8_t tx_o_line_buffer[UART_MSG_SIZE];

	uint8_t rx_buffer[UART_MSG_SIZE];


	int nb_messages_received = 0;
	float test_message_received = 0;
	unsigned int nb_updates_without_message = 0;

	unsigned int offset_message_already_received = 0;


	krabi_msgs::encoders encoders_msg;
	krabi_msgs::motors motors_msg;
	krabi_msgs::motors_parameters motors_parameters_msg;
	//krabi_msgs::odom_light odom_light_msg;
	krabi_msgs::odom_lighter odom_lighter_msg;

};

#endif /* INC_UARTBROKER_H_ */
