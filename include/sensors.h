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
class CI2CBus;

/** How often to update the sensors, in Hz */
#define SENSOR_UPDATE_FREQUENCY 0.5

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
	CI2CBus *m_pI2cBus;

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
	* @returns Returns #ERR_OK, or an appropriate error code if a failure occurred.
	*/
	int initSensors();

	/**
	* @brief This class tells all sensors to begin acquiring data.
	*/
	void startSensors();
	
	/**
	* @brief Update the sensor manager, check for errors on the comm threads
	* @details This will check for errors on the comm threads.
	* @returns Returns #ERR_OK if there were no errors, or an appropriate eror code if a failure occurred. 
	*/
	int update();
};
