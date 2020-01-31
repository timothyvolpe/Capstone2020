#pragma once

/**
* @file sensors.h
* @brief Contains code for managing all the vehicle sensors.
*
* @authors Timothy Volpe
*
* @date 1/29/2020
*/

/** The number of ultrasonic sensors to look for */
#define ULTRASONIC_SENSOR_COUNT 4

class CUltrasonicSensor;
class CLIDARSensor;
class CInertialMotionSensors;
class CGlobalPositioning;

/**
* @brief Class for managing all vehicle sensors.
* @details All vehicle sensors should be handled by this class. The CVehicle class should have an object
*	of this class.
*
* @author Timothy Volpe
* @date 1/29/2020
*/
class CSensorManager
{
private:
	CUltrasonicSensor* m_pUltrasonicSensors[ULTRASONIC_SENSOR_COUNT];
	CLIDARSensor* m_pLIDARSensor;
	CInertialMotionSensors* m_pInertialMotionSensors;
	CGlobalPositioning* m_pGPS;
public:
	/** Default constructor. */
	CSensorManager();
	/** Destructor. */
	~CSensorManager();

	/**
	* @brief Initialize all the sensors but dont start acquisition.
	* @details This will begin commmunication with all the sensors and set their default configs,
	*	however it will not begin acquiring.
	* @returns Returns #ERR_OK, or an appropriate error code if a failure occured.
	*/
	int initSensors();

	/**
	* @brief This class tells all sensors to begin acquiring data.
	*/
	void startSensors();
};