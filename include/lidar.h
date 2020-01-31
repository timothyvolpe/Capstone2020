#pragma once

/**
* @file lidar.h
* @brief LIDAR sensor for obstance detection above the waters surface.
*
* @authors Timothy Volpe
*
* @date 1/30/2020
*/

/**
* @brief Handles the RPLIDAR sensor.
* @details This class will handle a single RPLIDAR S1 sensors. Presumably there will only ever
*	be one sensor on the boat, but multiple can be added if necessary.
*
* @author Timothy Volpe
* @date 1/30/2020
*/
class CLIDARSensor
{
public:
	/** Default constructor */
	CLIDARSensor();
	/** Destructor */
	~CLIDARSensor();
};