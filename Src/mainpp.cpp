/*
 * main.cpp
 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */
#include <mainpp.h>
#include <constants.h>
extern "C" {
	#include "main.h"
}
#include <string.h>
#include <string>
#include "math.h"

#define UART_MSG_SIZE 1+ 8*10 + 1
uint8_t rx_buffer[UART_MSG_SIZE];
uint8_t rx_line_buffer[UART_MSG_SIZE];
uint8_t tx_e_line_buffer[UART_MSG_SIZE];
uint8_t tx_o_line_buffer[UART_MSG_SIZE];
unsigned int offset_message_already_received = 0;
int nb_messages_received = 0;
float test_message_received = 0;
unsigned int nb_updates_without_message = 0;


FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];


float get_float(const uint8_t* a_string, const int a_beginning)
{
	char hexValue[9];
	strncpy(hexValue, (const char*)a_string + a_beginning, 8);
	hexValue[8] = '\0'; // Null-terminate the string explicitly
    uint32_t floatHex = strtoul(hexValue, NULL, 16); // Convert hexadecimal string to unsigned long
    float floatValue;
    memcpy(&floatValue, &floatHex, sizeof(floatValue)); // Convert unsigned long to float
    return floatValue;
}

void motors_cmd_hex_cb(const uint8_t* a_message)
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

void parameters_hex_cb(const uint8_t* a_message)
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

void cmd_vel_hex_cb(const uint8_t* a_message)
{
	geometry_msgs::Twist twist_msg;
    int i = 0;
    const int offset = 1;
    const int float_msg_size = 8;
    twist_msg.linear.x = get_float(a_message, offset + (i++)*float_msg_size);
    twist_msg.angular.z = get_float(a_message, offset + (i++)*float_msg_size);

    cmd_vel_cb(twist_msg);
}

void enable_motor_hex_cb(const uint8_t* a_message)
{
	std_msgs::Bool enable_msg;
    int i = 0;
    const int offset = 1;
    const int float_msg_size = 8;
    enable_msg.data = get_float(a_message, offset + (i++)*float_msg_size) > 0.5;

    enable_motor_cb(enable_msg);
}

void read_serial()
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

void motors_cmd_cb(const CAN::MotorBoardCmdInput &motors_cmd_msg)
{
	if (motors_cmd_msg.reset_encoders)
	{
		MotorBoard::set_odom(0, 0, 0);
	}

	MotorBoard::getDCMotor().set_enable_motors((bool)motors_cmd_msg.enable_motors);

	if (!motors_cmd_msg.enable_motors) {
		MotorBoard::getDCMotor().resetMotors();
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

void receiveUART(UART_HandleTypeDef *huart){
	if (huart->Instance == USART2) {
		test_message_received++;
		memcpy(rx_line_buffer, rx_buffer, UART_MSG_SIZE);
		read_serial();

		/*int dma_buffer_offset = 0;

		int i = 0;
		while (i < RX_BUFFER_SIZE)
		{
			rx_line_buffer[offset_message_already_received + i] = rx_buffer[i];

			if (rx_buffer[i] == '\n' || rx_buffer[i] == '\r')
			{
				break;
			}
			else
			{
				i++;
			}
		}

		if (rx_buffer[i] == '\n' || rx_buffer[i] == '\r' ||
				(offset_message_already_received + i > 1 && rx_line_buffer[offset_message_already_received + i-1] == '\\' && (rx_line_buffer[offset_message_already_received + i] == 'n' || rx_line_buffer[offset_message_already_received + i] == 'r' )))
		{
			// Message received !
			read_serial();
			offset_message_already_received = 0;
		}
		else
		{
			offset_message_already_received += i;

			dma_buffer_offset += i;
			// Ensure the buffer does not overflow
			if (offset_message_already_received+RX_BUFFER_SIZE >= sizeof(rx_line_buffer)) {
				// Buffer overflow, handle error or reset the buffer
				offset_message_already_received = 0;
			}
		}*/
	}


}

void float_to_hex(const float a_value, uint8_t* a_out, const int start_pos)
{
    uint32_t floatHex;
    memcpy(&floatHex, &a_value, sizeof(a_value));
	sprintf((char*)(a_out + start_pos), "%08X", floatHex); // Convert to hexadecimal string
}

void publish_encoders(UART_HandleTypeDef * huart2)
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
uint16_t calculate_crc(const uint8_t *data, int length) {
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

void publish_odom_lighter(UART_HandleTypeDef * huart2)
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

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart){
	receiveUART(huart);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	receiveUART(huart);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	if (htim->Instance == TIM15) {
		MotorBoard::getDCMotor().update();

	}
	if (htim->Instance == TIM7) {
	}
}

DCMotorHardware MotorBoard::motorsHardware;
DCMotor MotorBoard::motors;
MCP3002 MotorBoard::currentReader;
volatile long long MotorBoard::message_counter = 0;
volatile long MotorBoard::last_encoder_left = 0;
volatile long MotorBoard::last_encoder_right = 0;
volatile long MotorBoard::int32_t_encoder_left = 0;
volatile long MotorBoard::int32_t_encoder_right = 0;
float MotorBoard::X = 0;
float MotorBoard::Y = 0;
float MotorBoard::theta_offset = 0;

void MotorBoard::set_odom(float a_x, float a_y, float a_theta)
{
	motors.resetEncodersCounter();
	X = a_x;
	Y = a_y;
	int16_t encoder_left = motors.get_encoder_ticks(M_L);
	int16_t encoder_right = motors.get_encoder_ticks(M_R);

	float current_theta = get_orientation_float(encoder_left, encoder_right, 0);
	theta_offset = a_theta - current_theta;

	last_encoder_left = motors.get_encoder_ticks(M_L);
	last_encoder_right = motors.get_encoder_ticks(M_R);

	int32_t_encoder_left = motors.get_encoder_ticks(M_L);
	int32_t_encoder_right = motors.get_encoder_ticks(M_R);
}

MotorBoard::MotorBoard(TIM_HandleTypeDef* a_motorTimHandler, UART_HandleTypeDef * huart2, FDCAN_HandleTypeDef* hcan, ADC_HandleTypeDef* hadc2) :
	huart2(huart2),
	hcan(hcan)
{

	while(false)
	{
		HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, static_cast<GPIO_PinState>(bool(int(HAL_GetTick()/1000)%2))); // Turn On/OFF LED
		HAL_Delay(100);
	}

	HAL_Delay(1);

	motorsHardware = DCMotorHardware(TIM2, TIM1, a_motorTimHandler, TIM_CHANNEL_2, a_motorTimHandler, TIM_CHANNEL_1);

#ifdef USE_MCP3002
	currentReader = MCP3002();
#else
	currentReader = MCP3002(hadc2);
#endif
	motors = DCMotor(&motorsHardware, &currentReader);

	motors.set_max_acceleration(millimetersToTicks(9810));//mm/s/s
	motors.set_max_speed(millimetersToTicks(2000));//mm/s (=1.9rad/s)

	set_odom(0, 0, 0);

	HAL_Delay(100);
}
MotorBoard::MotorBoard() {}
MotorBoard::~MotorBoard() {}

DCMotor& MotorBoard::getDCMotor(void) {
	return motors;
}

/*
    Return the Robot's orientation, in degrees, with respect to the last encoder reset.
*/
float get_orientation_float(int32_t encoder1, int32_t encoder2, float offset)
{


    float absolute_orientation = fmod(ticksToDegrees((encoder2 - encoder1)/2) + offset, 360);

    if (absolute_orientation >= 0)
        return (absolute_orientation);
    else
        return (360.f + absolute_orientation); // reminder: abs_ori is < 0 here
}

constexpr float ticksToMillimeters(int32_t ticks)
{
	return (DIST_MM_PER_WHEEL_REVOLUTION * (float)ticks / TICKS_PER_WHEEL_REVOLUTION);
}

constexpr float ticksToMeters(int32_t ticks)
{
	return ticksToMillimeters(ticks) / 1000.f;
}


constexpr int32_t millimetersToTicks(float millimeters)
{
	return static_cast<int32_t>(millimeters * TICKS_PER_WHEEL_REVOLUTION/DIST_MM_PER_WHEEL_REVOLUTION);
}

constexpr int32_t metersToTicks(float meters)
{
	return millimetersToTicks(meters * 1000);
}



constexpr int32_t degreesToTicks(float degrees)
{
	return degrees * TICKS_PER_ROBOT_DEG;
}
constexpr int32_t radsToTicks(float rads)
{
	return degreesToTicks(rads*180/M_PI);
}

constexpr float ticksToDegrees(int32_t ticks)
{
	return ticks/TICKS_PER_ROBOT_DEG;
}

constexpr float ticksToRads(int32_t ticks)
{
	return ticksToDegrees(ticks) * M_PI/180.f;
}
/*
        Given current value of both encoders
        return the linear dist by approximating it as the average of both wheels' linear distances.
        Static variables are used to keep last value of encoders.
*/
float MotorBoard::compute_linear_dist(const long encoder1, const long encoder2)
{
    float dist1, dist2, dist;
    int diff_encoder1, diff_encoder2;

    // Compute difference in nb of ticks between last measurements and now
    diff_encoder1 = diffWithFixOverflow(encoder1, last_encoder_left);
    diff_encoder2 = diffWithFixOverflow(encoder2, last_encoder_right);

    // Compute each wheel's dist and approximate linear dist as their average
    dist1 = ticksToMillimeters(diff_encoder1);
    dist2 = ticksToMillimeters(diff_encoder2);
    dist = (dist1 + dist2) / 2.0f;

    // Update static variables' values (current encoder values become old ones)
    last_encoder_left = encoder1;
    last_encoder_right = encoder2;

    // Return the computed linear dist
    return dist / 1000.f; // convert to meters
}

void MotorBoard::update_inputs() {

}

void doResetUart(UART_HandleTypeDef * huart2)
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

void MotorBoard::resetUart()
{
	doResetUart(huart2);
}

void MotorBoard::update() {
	nb_updates_without_message++;

	if (test_message_received - nb_messages_received > 4000)
	{
		toggleLed();
		this->resetUart();
		test_message_received = 0;
		nb_messages_received = 0;
	}

	// Check if the heart beat is OK
	if (nb_updates_without_message>1000)
	{
		toggleLed();
		this->resetUart();
		nb_updates_without_message = 0;
	}

	int16_t encoder_left = motors.get_encoder_ticks(M_L);
	int16_t encoder_right = motors.get_encoder_ticks(M_R);

	encoders_msg.encoder_left = encoder_left;
	encoders_msg.encoder_right = encoder_right;

	int32_t_encoder_left = fixOverflow(encoder_left, int32_t_encoder_left);
	int32_t_encoder_right = fixOverflow(encoder_right, int32_t_encoder_right);
	//publish_encoders(huart2); // Currently, it is only possible to transmit one message

	int32_t right_speed = motors.get_speed(M_R);
	int32_t left_speed = motors.get_speed(M_L);

	float linear_dist = compute_linear_dist(encoder_left, encoder_right);
	float current_theta = get_orientation_float(int32_t_encoder_left, int32_t_encoder_right, theta_offset);
	//current_theta += theta_offset;

	float current_theta_rad = current_theta * M_PI / 180.f;

	X += linear_dist * cos(current_theta_rad);
	Y += linear_dist * sin(current_theta_rad);

	// Debug communication
	/*odom_lighter_msg.poseX = MotorBoard::getDCMotor().get_linear_speed_order();//X;
	odom_lighter_msg.poseY = 4.2f;//Y;
	odom_lighter_msg.angleRz = nb_messages_received;//current_theta_rad;
	odom_lighter_msg.speedVx = (float)test_message_received;//ticksToMillimeters((left_speed+right_speed)/2)/1000.f;
	odom_lighter_msg.speedWz = encoder_left;//((right_speed - left_speed)/TICKS_PER_DEG)*M_PI/180; // rad/s*/


	float speedVx = ticksToMeters(left_speed + right_speed)/2;
	float speedWz = ticksToRads(right_speed - left_speed)/2; // rad/s

	odom_lighter_msg.poseX = X;
	odom_lighter_msg.poseY = Y;
	odom_lighter_msg.angleRz = current_theta_rad;
	odom_lighter_msg.speedVx = speedVx;
	odom_lighter_msg.speedWz = speedWz;
	publish_odom_lighter(huart2);

	/* Set the data to be transmitted */
	/*TxHeader.Identifier = CAN::can_ids::ODOMETRY_XY;
    int32_t poseX_mm = X * 1000;
    int32_t poseY_mm = Y * 1000;

    TxData[0] = (poseX_mm >> 24) & 0xFF;
    TxData[1] = (poseX_mm >> 16) & 0xFF;
    TxData[2] = (poseX_mm >> 8) & 0xFF;
    TxData[3] = (poseX_mm) & 0xFF;
    TxData[4] = (poseY_mm >>24) & 0xFF;
    TxData[5] = (poseY_mm >> 16) & 0xFF;
    TxData[6] = (poseY_mm >> 8) & 0xFF;
    TxData[7] = (poseY_mm) & 0xFF;
	if (HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, TxData) != HAL_OK)
	{
		// Transmission request Error
		MotorBoard::getDCMotor().resetMotors();
	}*/

	TxHeader.Identifier = CAN::can_ids::ODOMETRY_XYum;
	int32_t poseX_um = X * 1000000;
	int32_t poseY_um = Y * 1000000;

	TxData[0] = (poseX_um >> 24) & 0xFF;
	TxData[1] = (poseX_um >> 16) & 0xFF;
	TxData[2] = (poseX_um >> 8) & 0xFF;
	TxData[3] = (poseX_um) & 0xFF;
	TxData[4] = (poseY_um >>24) & 0xFF;
	TxData[5] = (poseY_um >> 16) & 0xFF;
	TxData[6] = (poseY_um >> 8) & 0xFF;
	TxData[7] = (poseY_um) & 0xFF;
	if (HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, TxData) != HAL_OK)
	{
		/* Transmission request Error */
		MotorBoard::getDCMotor().resetMotors();
	}

	TxHeader.Identifier = CAN::can_ids::ODOMETRY_THETA;
    int32_t angleRz_centi_deg = current_theta_rad * (100.0f * 180.f / M_PI);
    int16_t currentLeft  = motors.get_accumulated_current(M_L);
    int16_t currentRight = motors.get_accumulated_current(M_R);
    TxData[0] = (angleRz_centi_deg >> 24) & 0xFF;
	TxData[1] = (angleRz_centi_deg >> 16) & 0xFF;
	TxData[2] = (angleRz_centi_deg >> 8) & 0xFF;
	TxData[3] = (angleRz_centi_deg) & 0xFF;
	TxData[4] = (currentLeft >>8) & 0xFF;
	TxData[5] = (currentLeft) & 0xFF;
	TxData[6] = (currentRight >> 8) & 0xFF;
	TxData[7] = (currentRight) & 0xFF;
    if (HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, TxData) != HAL_OK)
	{
		/* Transmission request Error */
		MotorBoard::getDCMotor().resetMotors();
	}

	/*TxHeader.Identifier = CAN::can_ids::ODOMETRY_XY_FLOAT;
	memcpy(TxData, &(X), sizeof(float));
	memcpy(TxData + sizeof(float), &(Y), sizeof(float));
	if (HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, TxData) != HAL_OK)
	{
		// Transmission request Error
		MotorBoard::getDCMotor().resetMotors();
	}

	TxHeader.Identifier = CAN::can_ids::ODOMETRY_THETA;
	memcpy(TxData, &(current_theta_rad), sizeof(float));
	if (HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, TxData) != HAL_OK)
	{
		// Transmission request Error
		MotorBoard::getDCMotor().resetMotors();
	}*/

	TxHeader.Identifier = CAN::can_ids::ODOMETRY_SPEED;
    int32_t speedVx_µm_s = speedVx * 1000000.f;   // 4 bytes
    int32_t speedWz_mrad_s = speedWz * 1000.f; // 4 bytes

	TxData[0] = (speedVx_µm_s >> 24) & 0xFF;
	TxData[1] = (speedVx_µm_s >> 16) & 0xFF;
	TxData[2] = (speedVx_µm_s >> 8) & 0xFF;
	TxData[3] = (speedVx_µm_s ) & 0xFF;
	TxData[4] = (speedWz_mrad_s >> 24) & 0xFF;
	TxData[5] = (speedWz_mrad_s >> 16) & 0xFF;
	TxData[6] = (speedWz_mrad_s >> 8) & 0xFF;
	TxData[7] = (speedWz_mrad_s) & 0xFF;

	if (HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, TxData) != HAL_OK)
	{
		/* Transmission request Error */
		MotorBoard::getDCMotor().resetMotors();
	}

	TxHeader.Identifier = CAN::can_ids::CURRENT_LIMIT;
	uint16_t left_current_mA = motors.get_accumulated_current(M_L) /ONE_AMP; // 2 bytes
	uint16_t right_current_mA = motors.get_accumulated_current(M_R) /ONE_AMP; // 2 bytes
	uint16_t left_wheel_unstalled_in_ms = motors.get_remaining_time_stopped(M_L); // 2 bytes Nb of ms until the robot's left wheel is allowed to move again
	uint16_t right_wheel_unstalled_in_ms = motors.get_remaining_time_stopped(M_R);

	TxData[0] = (left_current_mA >> 8) & 0xFF;
	TxData[1] = (left_current_mA ) & 0xFF;
	TxData[2] = (right_current_mA >> 8) & 0xFF;
	TxData[3] = (right_current_mA ) & 0xFF;
	TxData[4] = (left_wheel_unstalled_in_ms >> 8) & 0xFF;
	TxData[5] = (left_wheel_unstalled_in_ms ) & 0xFF;
	TxData[6] = (right_wheel_unstalled_in_ms >> 8) & 0xFF;
	TxData[7] = (right_wheel_unstalled_in_ms) & 0xFF;

	if (HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, TxData) != HAL_OK)
	{
		/* Transmission request Error */
		//MotorBoard::getDCMotor().resetMotors();
	}


	/*TxHeader.Identifier = CAN::can_ids::ODOMETRY_SPEED_FLOAT;
	memcpy(TxData, &(speedVx), sizeof(float));
	memcpy(TxData+ sizeof(float), &(speedWz), sizeof(float));
	if (HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, TxData) != HAL_OK)
	{
		/* Transmission request Error *
		MotorBoard::getDCMotor().resetMotors();
	}*/


	if (false && message_counter%100 == 0)
	{
		motors_msg.current_left = motors.get_current(M_L);
		motors_msg.current_right = motors.get_current(M_R);

		motors_msg.current_left_accumulated = motors.get_accumulated_current(M_L);
		motors_msg.current_right_accumulated = motors.get_accumulated_current(M_R);

		//motors_pub.publish(&motors_msg);
	}
}

void setup()
{
  HAL_GPIO_WritePin(DIR_A_GPIO_Port, DIR_A_Pin, GPIO_PIN_RESET);//DIR_A
  HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, GPIO_PIN_RESET);//DIR_B
}

void toggleLed()
{
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // Toggle LED on GPIOA Pin 5
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    /* Retrieve Rx messages from RX FIFO0 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
    	Error_Handler();
    }

    if ((RxHeader.Identifier == CAN::can_ids::CMD_VEL_FLOAT) && (RxHeader.IdType == FDCAN_STANDARD_ID) && (RxHeader.DataLength == FDCAN_DLC_BYTES_8))
    {
    	CAN::CmdVelFloat l_cmd_vel;
        memcpy(&(l_cmd_vel.linear_x_m), RxData, sizeof(float));
        memcpy(&(l_cmd_vel.angular_z_rad), RxData + sizeof(float), sizeof(float));

        MotorBoard::getDCMotor().set_speed_order(metersToTicks(l_cmd_vel.linear_x_m), radsToTicks(-l_cmd_vel.angular_z_rad));
    }

    if ((RxHeader.Identifier == CAN::can_ids::CMD_VEL) && (RxHeader.IdType == FDCAN_STANDARD_ID) && (RxHeader.DataLength == FDCAN_DLC_BYTES_8))
        {
        	CAN::CmdVelFloat l_cmd_vel;

            int32_t linear_x_µm_s = 0;
            int32_t angular_z_µrad_s = 0;
            linear_x_µm_s |= RxData[0] << 24;
            linear_x_µm_s |= RxData[1] << 16;
            linear_x_µm_s |= RxData[2] << 8;
            linear_x_µm_s |= RxData[3] ;

            angular_z_µrad_s |= RxData[4] << 24;
            angular_z_µrad_s |= RxData[5] << 16;
            angular_z_µrad_s |= RxData[6] << 8;
            angular_z_µrad_s |= RxData[7] ;

            //l_cmd_vel.linear_x_m = linear_x_µm_s/(10000000.0f); // before fix
            //l_cmd_vel.angular_z_rad = angular_z_µrad_s/(10000000.0f); // before fix
            l_cmd_vel.linear_x_m = linear_x_µm_s/(1000000.0f);
            l_cmd_vel.angular_z_rad = angular_z_µrad_s/(1000000.0f);

            //l_cmd_vel.angular_z_rad /= 2.4;// Magic factor :'(
        	//@todo decode
            //memcpy(&(l_cmd_vel.linear_x_m), RxData, sizeof(float));
            //memcpy(&(l_cmd_vel.angular_z_rad), RxData + sizeof(float), sizeof(float));

            MotorBoard::getDCMotor().set_speed_order(metersToTicks(l_cmd_vel.linear_x_m), radsToTicks(-l_cmd_vel.angular_z_rad));
        }

    if ((RxHeader.Identifier == CAN::can_ids::MOTOR_BOARD_CMD_INPUT) && (RxHeader.IdType == FDCAN_STANDARD_ID) && (RxHeader.DataLength == FDCAN_DLC_BYTES_8))
	{
		CAN::MotorBoardCmdInput l_cmd_inputs;

		//memcpy(&(l_cmd_inputs), RxData, 8*sizeof(uint8_t));

		l_cmd_inputs.enable_motors = RxData[0];
		l_cmd_inputs.override_PWM = RxData[1];
		l_cmd_inputs.PWM_override_left = RxData[3] | (RxData[2] << 8);
		l_cmd_inputs.PWM_override_right = RxData[5] | (RxData[4] << 8);
		l_cmd_inputs.reset_encoders = RxData[6];

		motors_cmd_cb(l_cmd_inputs);
	}
    if ((RxHeader.Identifier == CAN::can_ids::MOTOR_BOARD_CURRENT_INPUT) && (RxHeader.IdType == FDCAN_STANDARD_ID) && (RxHeader.DataLength == FDCAN_DLC_BYTES_8))
	{
		CAN::MotorBoardCurrentInput l_cmd_current_inputs;

		//memcpy(&(l_cmd_current_inputs), RxData, 8*sizeof(uint8_t));
		l_cmd_current_inputs.max_current_left_mA = RxData[1] | (RxData[0] << 8);
		l_cmd_current_inputs.max_current_right_mA = RxData[3] | (RxData[2] << 8);
		l_cmd_current_inputs.max_current_mA = RxData[5] | (RxData[4] << 8);

		MotorBoard::getDCMotor().set_max_current(l_cmd_current_inputs.max_current_mA/1000.0f);
		MotorBoard::getDCMotor().set_max_current(l_cmd_current_inputs.max_current_left_mA/1000.0f, l_cmd_current_inputs.max_current_right_mA/1000.0f);
	}

    if ((RxHeader.Identifier == CAN::can_ids::MOTOR_BOARD_ENABLE) && (RxHeader.IdType == FDCAN_STANDARD_ID))
	{
		bool l_motor_enable = RxData[0];

		if (!l_motor_enable)
		{
			MotorBoard::getDCMotor().resetMotors();
		}
	}
  }
}

/**
  * @brief  Configures the FDCAN.
  *   None
  * @retval None
  */
static void FDCAN_Config(FDCAN_HandleTypeDef* hcan)
{
  FDCAN_FilterTypeDef sFilterConfig;

  /* Configure Rx filter */
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 42;
  sFilterConfig.FilterID2 = 0x7FF;
  if (HAL_FDCAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* Start the FDCAN module */
  if (HAL_FDCAN_Start(hcan) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }

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
}

void loop(TIM_HandleTypeDef* a_motorTimHandler, TIM_HandleTypeDef* a_loopTimHandler, UART_HandleTypeDef * huart2, FDCAN_HandleTypeDef* hcan, ADC_HandleTypeDef* hadc2)
{
	MotorBoard myboard = MotorBoard(a_motorTimHandler, huart2, hcan, hadc2);

	__HAL_UART_CLEAR_OREFLAG(huart2); // Not sure if actually needed



	// CAN sandbox, from https://community.st.com/t5/stm32-mcus/how-to-use-fdcan-to-create-a-simple-communication-with-a-basic/ta-p/671766

	FDCAN_Config(hcan);


	/* Start the Transmission process */
	if (HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, TxData) != HAL_OK)
	{
		/* Transmission request Error */
		MotorBoard::getDCMotor().resetMotors();
	}

	HAL_TIM_Base_Start_IT(a_loopTimHandler);
	uint32_t waiting_time = 5;

	// Make sure that the Uart/DMA is correctly initialized, or signal it
	while (HAL_UART_Receive_DMA(huart2, rx_buffer, UART_MSG_SIZE) != HAL_OK)
	{
		toggleLed();
		HAL_Delay(300);
	}

	while(true) {
		myboard.update();



		HAL_Delay(waiting_time);
	}
}
