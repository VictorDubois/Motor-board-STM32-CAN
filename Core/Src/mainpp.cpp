/*
 * main.cpp
 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */
#include <mainpp.h>
#include "canManager.h"

#include <constants.h>
extern "C" {
	#include "main.h"
}

#include "math.h"

#include <uartBroker.h>
#include <cstring>

#include "callbacks.h"

FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];

uartBroker s_uart_broker;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart){
	s_uart_broker.receiveUART(huart);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	s_uart_broker.receiveUART(huart);
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
CurrentReaderCan MotorBoard::currentReader;
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

	motorsHardware = DCMotorHardware(TIM2, TIM1, a_motorTimHandler, TIM_CHANNEL_2, a_motorTimHandler, TIM_CHANNEL_1, hcan);

#ifdef USE_MCP3002
	currentReader = CurrentReaderMCP3002();
#else
	//currentReader = CurrentReaderAdc(hadc2);
	currentReader = CurrentReaderCan();
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

void MotorBoard::updateCurrent()
{
	TxHeader.Identifier = CAN::can_ids::CURRENT_LIMIT;
	int16_t left_current_mA = motors.get_accumulated_current(M_L) / currentReader.getOneMilliAmp(); // 2 bytes
	int16_t right_current_mA = motors.get_accumulated_current(M_R) / currentReader.getOneMilliAmp(); // 2 bytes
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


	CAN_Enqueue(&TxHeader, TxData);


	int32_t right_speed = motors.get_speed(M_R);
	int32_t left_speed = motors.get_speed(M_L);

	float speedVx = ticksToMeters(left_speed + right_speed)/2;
	float speedWz = ticksToRads(right_speed - left_speed)/2; // rad/s

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

	CAN_Enqueue(&TxHeader, TxData);
}

void MotorBoard::update() {
	s_uart_broker.checkHeartBeat(huart2);

	int16_t encoder_left = motors.get_encoder_ticks(M_L);
	int16_t encoder_right = motors.get_encoder_ticks(M_R);

	s_uart_broker.setEncodersMsg(encoder_left, encoder_right);

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

	s_uart_broker.setOdomLighterMsg(X, Y, current_theta_rad, speedVx, speedWz);

	s_uart_broker.publish_odom_lighter(huart2);

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
	CAN_Enqueue(&TxHeader, TxData);*/

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
	CAN_Enqueue(&TxHeader, TxData);


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
	CAN_Enqueue(&TxHeader, TxData);


	/*TxHeader.Identifier = CAN::can_ids::ODOMETRY_XY_FLOAT;
	memcpy(TxData, &(X), sizeof(float));
	memcpy(TxData + sizeof(float), &(Y), sizeof(float));
	CAN_Enqueue(&TxHeader, TxData);

	TxHeader.Identifier = CAN::can_ids::ODOMETRY_THETA;
	memcpy(TxData, &(current_theta_rad), sizeof(float));
	CAN_Enqueue(&TxHeader, TxData);*/


	/*TxHeader.Identifier = CAN::can_ids::ODOMETRY_SPEED_FLOAT;
	memcpy(TxData, &(speedVx), sizeof(float));
	memcpy(TxData+ sizeof(float), &(speedWz), sizeof(float));
	CAN_Enqueue(&TxHeader, TxData);*/



	if (false && message_counter%100 == 0)
	{
		s_uart_broker.setMotorsMsg(s_uart_broker.getEncodersMsg(), motors.get_current(M_L), motors.get_current(M_R), motors.get_accumulated_current(M_L), motors.get_accumulated_current(M_R));

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

    else if ((RxHeader.Identifier == CAN::can_ids::C620_OUTPUT_1) && (RxHeader.IdType == FDCAN_STANDARD_ID) && (RxHeader.DataLength == FDCAN_DLC_BYTES_8))
	{
		uint16_t l_mechanical_angle_8192_ticks = 0;
		int16_t l_speed_rpm = 0;
		int16_t l_torque = 0;

		l_mechanical_angle_8192_ticks |= RxData[0] << 8;
		l_mechanical_angle_8192_ticks |= RxData[1];
		l_speed_rpm |= RxData[2] << 8;
		l_speed_rpm |= RxData[3] ;
		l_torque |= RxData[4] << 8;
		l_torque |= RxData[5] ;


#ifdef USE_CAN_SPEED_ODOMETRY
		float l_speed_meter_s = l_speed_rpm * MOTOR_RPM_TO_WHEEL_M_S;
		MotorBoard::getDCMotor().set_speed(M_L, metersToTicks(l_speed_meter_s));
		MotorBoard::getDCMotor().set_ticks(M_L, l_mechanical_angle_8192_ticks);
#endif

#ifdef USE_C620_CURRENT
		MotorBoard::getDCMotor().set_current(M_L, l_torque);
#endif
	}

    else if ((RxHeader.Identifier == CAN::can_ids::C620_OUTPUT_2) && (RxHeader.IdType == FDCAN_STANDARD_ID) && (RxHeader.DataLength == FDCAN_DLC_BYTES_8))
	{
		uint16_t l_mechanical_angle_8192_ticks = 0;
		int16_t l_speed_rpm = 0;
		int16_t l_torque = 0;

		l_mechanical_angle_8192_ticks |= RxData[0] << 8;
		l_mechanical_angle_8192_ticks |= RxData[1];
		l_speed_rpm |= RxData[2] << 8;
		l_speed_rpm |= RxData[3] ;
		l_torque |= RxData[4] << 8;
		l_torque |= RxData[5] ;


#ifdef USE_CAN_SPEED_ODOMETRY
		float l_speed_meter_s = l_speed_rpm * MOTOR_RPM_TO_WHEEL_M_S;
		MotorBoard::getDCMotor().set_speed(M_R, metersToTicks(l_speed_meter_s));
		MotorBoard::getDCMotor().set_ticks(M_R, l_mechanical_angle_8192_ticks);
#endif

#ifdef USE_C620_CURRENT
		MotorBoard::getDCMotor().set_current(M_R, l_torque);
#endif
	}

    else if ((RxHeader.Identifier == CAN::can_ids::CMD_VEL) && (RxHeader.IdType == FDCAN_STANDARD_ID) && (RxHeader.DataLength == FDCAN_DLC_BYTES_8))
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

    else if ((RxHeader.Identifier == CAN::can_ids::MOTOR_BOARD_CMD_INPUT) && (RxHeader.IdType == FDCAN_STANDARD_ID) && (RxHeader.DataLength == FDCAN_DLC_BYTES_8))
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
    else if ((RxHeader.Identifier == CAN::can_ids::DIGITAL_OUTPUTS) && (RxHeader.IdType == FDCAN_STANDARD_ID) && (RxHeader.DataLength == FDCAN_DLC_BYTES_8))
    {
		CAN::DigitalOutputs l_digital_outputs;

		l_digital_outputs.enable_outputs = RxData[1] | (RxData[0] << 8);
		l_digital_outputs.enable_power = RxData[2];

		digital_outputs_cb(l_digital_outputs);
	}
    else if ((RxHeader.Identifier == CAN::can_ids::MOTOR_BOARD_CURRENT_INPUT) && (RxHeader.IdType == FDCAN_STANDARD_ID) && (RxHeader.DataLength == FDCAN_DLC_BYTES_8))
	{
		CAN::MotorBoardCurrentInput l_cmd_current_inputs;

		//memcpy(&(l_cmd_current_inputs), RxData, 8*sizeof(uint8_t));
		l_cmd_current_inputs.max_current_left_mA = RxData[1] | (RxData[0] << 8);
		l_cmd_current_inputs.max_current_right_mA = RxData[3] | (RxData[2] << 8);
		l_cmd_current_inputs.max_current_mA = RxData[5] | (RxData[4] << 8);

		MotorBoard::getDCMotor().set_max_current(l_cmd_current_inputs.max_current_mA/1000.0f);
		MotorBoard::getDCMotor().set_max_current(l_cmd_current_inputs.max_current_left_mA/1000.0f, l_cmd_current_inputs.max_current_right_mA/1000.0f);
	}

    else if ((RxHeader.Identifier == CAN::can_ids::MOTOR_BOARD_ENABLE) && (RxHeader.IdType == FDCAN_STANDARD_ID))
	{
		bool l_motor_enable = RxData[0];

		if (!l_motor_enable)
		{
			MotorBoard::getDCMotor().resetMotor(M_L);
			MotorBoard::getDCMotor().resetMotor(M_R);
		}

	}
    else if ((RxHeader.Identifier == CAN::can_ids::MOTOR_BOARD_LINEAR_PI_SET) && (RxHeader.IdType == FDCAN_STANDARD_ID) && (RxHeader.DataLength == FDCAN_DLC_BYTES_8))
    {
        CAN::MotorBoardPiSet msg;
        memcpy(&msg, RxData, sizeof(msg));
        MotorBoard::getDCMotor().set_linear_pi(msg.p, msg.i);
    }
    else if ((RxHeader.Identifier == CAN::can_ids::MOTOR_BOARD_ANGULAR_PI_SET) && (RxHeader.IdType == FDCAN_STANDARD_ID) && (RxHeader.DataLength == FDCAN_DLC_BYTES_8))
    {
        CAN::MotorBoardPiSet msg;
        memcpy(&msg, RxData, sizeof(msg));
        MotorBoard::getDCMotor().set_angular_pi(msg.p, msg.i);
    }
    else if ((RxHeader.Identifier == CAN::can_ids::MOTOR_BOARD_DERIVATIVE_SET) && (RxHeader.IdType == FDCAN_STANDARD_ID) && (RxHeader.DataLength == FDCAN_DLC_BYTES_8))
    {
        CAN::MotorBoardDerivativeSet msg;
        memcpy(&msg, RxData, sizeof(msg));
        MotorBoard::getDCMotor().set_derivative(msg.linear_d, msg.angular_d);
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
	uint32_t waiting_time = 5; // ms

	s_uart_broker.initDMA(huart2);

	CAN::DigitalOutputs init_digital_outputs;
	init_digital_outputs.enable_outputs = 0;
	init_digital_outputs.enable_power= 0;
	digital_outputs_cb(init_digital_outputs);

	while(true) {

		myboard.update();
		//HAL_Delay(1); // ms

		myboard.updateCurrent();
		CAN_ProcessTxQueue(hcan);

		HAL_Delay(waiting_time - 1); // ms
	}
}
