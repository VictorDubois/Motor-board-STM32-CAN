//add constant from libmd25 use in main
#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

#include <stdint.h>
#include <math.h>

/*****************************
 *         PARAMETERS        *
 *****************************/

// At the moment, we keep floats as we are not (yet) in optimization mode
#define KV2
// #define USE_CAN_SPEED_ODOMETRY

#ifdef USE_CAN_ODOMETRY
	#define TICKS_OVERFLOW 8192.0
	#define TICKS_half_OVERFLOW TICKS_OVERFLOW/2.0
#else
	// dedicated odo
	#define TICKS_OVERFLOW 65536.0
	#define TICKS_half_OVERFLOW TICKS_OVERFLOW/2.0
#endif

#ifdef KV1
	#define TICKS_PER_REVOLUTION 4096 	// Nb ticks per wheel revolution
	#define WHEEL_DIAM 68               	// Diameter of the wheel (mm) UNUSED !!!
	#define DIST_PER_REVOLUTION 216.2695  	// Distance traveled for a full wheel revolution (in mm)
	#define TICKS_PER_DEG 91.472       	// Nb of diff tick (enc1 - enc2) it takes to rotate 1 deg (old value: 4.7)
#endif

#ifdef KV2
	constexpr float M2006_REDUCTION_RATIO = 36;
	constexpr float MOTOR_WHEEL_RADIUS_MM = 30;

	#ifdef USE_CAN_ODOMETRY
		constexpr float TICKS_PER_WHEEL_REVOLUTION = M2006_REDUCTION_RATIO * 8096.0; 	// Nb ticks per wheel revolution
		constexpr float WHEEL_RADIUS_MM = MOTOR_WHEEL_RADIUS_MM;
		constexpr float VOIE_MM = 160.0; //@todo set
	#else
		// KV2 dedicated odo
		constexpr float TICKS_PER_WHEEL_REVOLUTION = 20480.0; 	// Nb ticks per wheel revolution
		constexpr float WHEEL_RADIUS_MM = 19.91;
		constexpr float VOIE_MM = 205.10;
	#endif

	constexpr float MOTOR_RPM_TO_WHEEL_MM_S = MOTOR_WHEEL_RADIUS_MM * M_PI/(60.0 * M2006_REDUCTION_RATIO);
	constexpr float MOTOR_RPM_TO_WHEEL_M_S = MOTOR_RPM_TO_WHEEL_MM_S/1000;

	constexpr float DIST_MM_PER_WHEEL_REVOLUTION_THEORY = (WHEEL_RADIUS_MM*2*M_PI); // Distance traveled for a full wheel revolution (in mm)
	constexpr float DIST_MM_PER_WHEEL_REVOLUTION = (DIST_MM_PER_WHEEL_REVOLUTION_THEORY);///2.494362831); // Distance traveled for a full wheel revolution (in mm)
	constexpr float DIST_PER_ROBOT_REVOLUTION_THEORY = (VOIE_MM * M_PI);
	constexpr float TICKS_PER_ROBOT_REVOLUTION_THEORY = ((DIST_PER_ROBOT_REVOLUTION_THEORY * TICKS_PER_WHEEL_REVOLUTION)/(DIST_MM_PER_WHEEL_REVOLUTION_THEORY));
	constexpr float TICKS_PER_ROBOT_DEG_THEORY = (TICKS_PER_ROBOT_REVOLUTION_THEORY/360.0); // Nb of diff tick (enc1 - enc2) it takes to rotate 1 deg
	constexpr float TICKS_PER_ROBOT_DEG = TICKS_PER_ROBOT_DEG_THEORY;//* 0.96007158213; // Nb of diff tick (enc1 - enc2) it takes to rotate 1 deg
#endif


constexpr float ticksToMillimeters(int32_t ticks);
constexpr float ticksToMeters(int32_t ticks);
constexpr int32_t millimetersToTicks(float millimeters);
constexpr int32_t metersToTicks(float meters);
constexpr int32_t degreesToTicks(float degrees);
constexpr int32_t radsToTicks(float rads);
constexpr float ticksToDegrees(int32_t ticks);
constexpr float ticksToRads(int32_t ticks);

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

#endif
