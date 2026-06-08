/*
 * uart_broker.cpp
 *
 *  Created on: Jun 8, 2026
 *      Author: victor.dubois@navya.tech
 */

#include <string.h>
#include <cstring>
#include <string>

#include <CanStruct/can_structs.h>
#include <uartBroker.h>
#include "msgs.h"

#include "callbacks.h"

#include "mainpp.h"// for krabi_msgs. @todo refactor

float uartBroker::get_float(const uint8_t* a_string, const int a_beginning)
{
	char hexValue[9];
	strncpy(hexValue, (const char*)a_string + a_beginning, 8);
	hexValue[8] = '\0'; // Null-terminate the string explicitly
    uint32_t floatHex = strtoul(hexValue, NULL, 16); // Convert hexadecimal string to unsigned long
    float floatValue;
    memcpy(&floatValue, &floatHex, sizeof(floatValue)); // Convert unsigned long to float
    return floatValue;
}

void uartBroker::motors_cmd_hex_cb(const uint8_t* a_message)
{
	krabi_msgs::motors_cmd motors_cmd_msg;
	int i = 0;
    const int offset = 1;
    const int float_msg_size = 8;

    motors_cmd_msg.enable_motors = get_float(a_message, offset + (i++)*float_msg_size) > 0.5;
    motors_cmd_msg.override_PWM = get_float(a_message, offset + (i++)*float_msg_size) > 0.5;
    motors_cmd_msg.PWM_override_left = get_float(a_message, offset + (i++)*float_msg_size);
    motors_cmd_msg.PWM_override_right = get_float(a_message, offset + (i++)*float_msg_size);
    motors_cmd_msg.reset_encoders = get_float(a_message, offset + (i++)*float_msg_size) > 0.5;

    motors_cmd_cb(motors_cmd_msg);
}

void uartBroker::parameters_hex_cb(const uint8_t* a_message)
{
	krabi_msgs::motors_parameters motors_parameters_msg;
    int i = 0;
    const int offset = 1;
    const int float_msg_size = 8;
    motors_parameters_msg.max_current_left = get_float(a_message, offset + (i++)*float_msg_size);
    motors_parameters_msg.max_current_right = get_float(a_message, offset + (i++)*float_msg_size);
    motors_parameters_msg.max_current = get_float(a_message, offset + (i++)*float_msg_size);

    parameters_cb(motors_parameters_msg);
}

void uartBroker::cmd_vel_hex_cb(const uint8_t* a_message)
{
	geometry_msgs::Twist twist_msg;
    int i = 0;
    const int offset = 1;
    const int float_msg_size = 8;
    twist_msg.linear.x = get_float(a_message, offset + (i++)*float_msg_size);
    twist_msg.angular.z = get_float(a_message, offset + (i++)*float_msg_size);

    cmd_vel_cb(twist_msg);
}

void uartBroker::enable_motor_hex_cb(const uint8_t* a_message)
{
	std_msgs::Bool enable_msg;
    int i = 0;
    const int offset = 1;
    const int float_msg_size = 8;
    enable_msg.data = get_float(a_message, offset + (i++)*float_msg_size) > 0.5;

    enable_motor_cb(enable_msg);
}

void uartBroker::read_serial()
{
	//HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, GPIO_PIN_SET); // Turn On LED
	nb_updates_without_message = 0;


	if(rx_line_buffer[0]=='e')
	{
		nb_messages_received++;
		enable_motor_hex_cb(rx_line_buffer);
	}
	if(rx_line_buffer[0]=='v')
	{
		nb_messages_received++;
		cmd_vel_hex_cb(rx_line_buffer);
	}
	if(rx_line_buffer[0]=='p')
	{
		nb_messages_received++;
		parameters_hex_cb(rx_line_buffer);
	}
	if(rx_line_buffer[0]=='c')
	{
		nb_messages_received++;
		motors_cmd_hex_cb(rx_line_buffer);
	}
}


void uartBroker::receiveUART(UART_HandleTypeDef *huart){
	if (huart->Instance == USART2) {
		test_message_received++;
		memcpy(rx_line_buffer, rx_buffer, UART_MSG_SIZE);
		read_serial();
	}
}

void uartBroker::float_to_hex(const float a_value, uint8_t* a_out, const int start_pos)
{
    uint32_t floatHex;
    memcpy(&floatHex, &a_value, sizeof(a_value));
    sprintf((char *)(a_out + start_pos), "%08" PRIX32, floatHex); // Convert to hexadecimal string

}

void uartBroker::publish_encoders(UART_HandleTypeDef * huart2)
{
	int start_pos = 0;
	const int step = 8;

	tx_e_line_buffer[start_pos] = 'e';
	start_pos += 1;
	tx_e_line_buffer[start_pos] = ':';
	start_pos += 1;
	float_to_hex(encoders_msg.encoder_right, tx_e_line_buffer, start_pos);
	start_pos += step;
	float_to_hex(encoders_msg.encoder_left, tx_e_line_buffer, start_pos);
	start_pos += step;

	tx_e_line_buffer[start_pos] = '\n';
	start_pos += 2;

	HAL_UART_Transmit_DMA(huart2, tx_e_line_buffer, start_pos);
}

// Define CRC parameters
#define CRC_POLYNOMIAL 0x1021
#define CRC_INITIAL_VALUE 0xFFFF

// Function to calculate CRC
uint16_t uartBroker::calculate_crc(const uint8_t *data, int length) {
    uint16_t crc = CRC_INITIAL_VALUE;

    for (int i = 0; i < length; i++) {
        crc ^= data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ CRC_POLYNOMIAL;
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

void uartBroker::publish_odom_lighter(UART_HandleTypeDef * huart2)
{
	int start_pos = 0;
	const int step = 8;

	tx_o_line_buffer[start_pos] = 'o';
	start_pos += 1;
	tx_o_line_buffer[start_pos] = ':';
	start_pos += 1;
	float_to_hex(odom_lighter_msg.poseX, tx_o_line_buffer, start_pos);
	start_pos += step;
	float_to_hex(odom_lighter_msg.poseY, tx_o_line_buffer, start_pos);
	start_pos += step;
	float_to_hex(odom_lighter_msg.angleRz, tx_o_line_buffer, start_pos);
	start_pos += step;
	float_to_hex(odom_lighter_msg.speedVx, tx_o_line_buffer, start_pos);
	start_pos += step;
	float_to_hex(odom_lighter_msg.speedWz, tx_o_line_buffer, start_pos);
	start_pos += step;

	// Calculate CRC
	uint16_t crc = calculate_crc(tx_o_line_buffer, start_pos);
	tx_o_line_buffer[start_pos] = crc & 0xFF; // Low byte
	tx_o_line_buffer[start_pos + 1] = crc >> 8; // High byte
	start_pos += 2;

	// Add and of line
	tx_o_line_buffer[start_pos] = '\n';
	start_pos += 1;

	HAL_UART_Transmit_DMA(huart2, tx_o_line_buffer, start_pos);
}


void uartBroker::doResetUart(UART_HandleTypeDef * huart2)
{
	// 1. Disable UART and DMA
	HAL_UART_DMAStop(huart2); // Stop DMA associated with UART2
	HAL_UART_DeInit(huart2); // Deinitialize UART2

	// 2. Reset UART Configuration
	huart2->Instance = USART2;
	huart2->Init.BaudRate = 115200;
	huart2->Init.WordLength = UART_WORDLENGTH_8B;
	huart2->Init.StopBits = UART_STOPBITS_1;
	huart2->Init.Parity = UART_PARITY_NONE;
	huart2->Init.Mode = UART_MODE_TX_RX;
	huart2->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2->Init.OverSampling = UART_OVERSAMPLING_16;
	huart2->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

	while (HAL_UART_Init(huart2) != HAL_OK)
	{
		toggleLed();
		HAL_Delay(100);
	}
	while (HAL_UART_Receive_DMA(huart2, rx_buffer, UART_MSG_SIZE) != HAL_OK)
	{
		toggleLed();
		HAL_Delay(300);
	}
}

void uartBroker::checkHeartBeat(UART_HandleTypeDef * huart2)
{
	nb_updates_without_message++;

	if (test_message_received - nb_messages_received > 4000)
	{
		toggleLed();
		doResetUart(huart2);
		test_message_received = 0;
		nb_messages_received = 0;
	}

	// Check if the heart beat is OK
	if (nb_updates_without_message>1000)
	{
		toggleLed();
		doResetUart(huart2);
		nb_updates_without_message = 0;
	}
}

void uartBroker::initDMA(UART_HandleTypeDef * huart2)
{
	// Make sure that the Uart/DMA is correctly initialized, or signal it
	while (HAL_UART_Receive_DMA(huart2, rx_buffer, UART_MSG_SIZE) != HAL_OK)
	{
		toggleLed();
		HAL_Delay(300);
	}
}
