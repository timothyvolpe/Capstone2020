#pragma once

/**
* @file sensors.h
* @brief Contains code for managing all the vehicle sensors.
*
* @authors Timothy Volpe
*
* @date 1/29/2020
*/

/** The number of ultrasonic sensors to look for, maximum. Can be reduced in config.ini for testing. */
#define ULTRASONIC_MAX_SENSOR_COUNT 4

class CUltrasonicSensor;
class CLIDARSensor;
class CInertialMotionSensors;
class CGlobalPositioning;
class CI2CBus;

/** How often to update the sensors, in Hz */
#define SENSOR_UPDATE_FREQUENCY 2

/** The amount of time, in ms, to wait for the i2c write messages to flush at the end of an update frame. */
#define I2C_FLUSH_TIMEOUT_MS 500
/** The number of consecutive i2c flush failures before a fatal error occurs. */
#define I2C_FLUSH_FAILURE_LIMIT 2

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
	int m_consecFlushFailures;

	CUltrasonicSensor* m_pUltrasonicSensors[ULTRASONIC_MAX_SENSOR_COUNT];
	int m_actualUltraSensorCount;
	
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
	
	/**
	* @brief Prints new range readings from each ultrasonic sensor.
	* @details This will command the sensors to take new readings and report them to the console.
	* This is blocking
	* @returns Returns true if no error, or false if there was an error.
	* @warning Blocking.
	*/
	bool printUltrasonicReadings();
	
	CI2CBus* getI2CBus();
};
