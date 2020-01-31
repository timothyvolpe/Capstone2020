#pragma once

/**
* @file kinetics.h
* @brief Motion sensor code for analyzing vehicle motion.
*
* @authors Timothy Volpe
*
* @date 1/30/2020
*/

/**
* @brief Handles inertial motion such a gyroscope, accelerometer, and magnetometer.
* @details This class takes input from an intertial motion unit (IMU) or individual sensors
*	action as an IMU. Orientation is calculated from gyroscopic input, acceleration from an
*	accelerometer, and cardinal direction.
*
* @author Timothy Volpe
* @date 1/30/2020
*/
class CInertialMotionSensors
{
public:
	CInertialMotionSensors();
	~CInertialMotionSensors();
};

/**
* @brief Takes input from a GPS unit.
* @details This class handles setting up and managing a GPS unit, as well as taking and 
*	cleaning up its output to provide global position.
*
* @author Timothy Volpe
* @date 1/30/2020
*/
class CGlobalPositioning
{
public:
	CGlobalPositioning();
	~CGlobalPositioning();
};