#pragma once
#include <chrono>

/** The factory default address of the sensors, or the address when forced to default. */
#define ULTRASONIC_DEFAULT_ADDRESS 224
/** The first part of the address unlock command. */
#define ULTRASONIC_ADDR_UNLOCK_1 170
/** The second part of the address unlock command. */
#define ULTRASONIC_ADDR_UNLOCK_2 165

/** The minimum wait time between taking a reading and getting the value, per datasheet (ms) */
#define ULTRASONIC_READ_TIME_MS 80
/** The minimum time between consecutive readings, per datasheet (ms) */
#define ULTRASONIC_OFF_TIME_MS 100
/** Time to wait for device to restart when address is changed, per datasheet (ms) */
#define ULTRASONIC_RESET_DELAY_MS 125

/**
* @file ultrasonic.h
* @brief Ultrasonic sensor management for underwater obstacle avoidance.
*
* @authors Timothy Volpe
*
* @date 1/29/2020
*/

class CI2CBus;

/**
* @brief Handles an ultrasonic sensor.
* @details This class handles a single ultrasonic sensor. Multiple ultrasonic sensors can be
*	used will multiple instances of this class.
*
* @author Timothy Volpe
* @date 1/29/2020
*/
class CUltrasonicSensor
{
private:
	CI2CBus *m_pI2CBus;
	
	unsigned char m_sensorAddress;
	
	std::chrono::steady_clock::time_point m_lastReadingTaken;
	
	/**
	* @brief Polls every possible Ultrasonic sensor address. Used for debugging
	* @details This checks each address one at a time. This will take a few seconds, and is blocking.
	* Only use for debugging.
	* @returns Returns #ERR_OK on success, or an appropriate error code if there was a failure. If no sensors were found but there were no comm errors, #ERR_OK is still returned.
	* @warning Only use for debugging
	*/
	int pollAllAddress();
public:
	/** Default constructor */
	CUltrasonicSensor( CI2CBus *pI2CBus, unsigned char address );
	/** Destructor */
	~CUltrasonicSensor();
	
	/**
	* @brief Initialize the sensor are verify it is responding.
	* @details This will take a reading to verify that a valid sensor exists at the address. In only this case,
	* it will be blocking and cause about a ~#ULTRASONIC_READ_TIME_MS delay.
	* @returns Returns #ERR_OK if successful, or appropriate error code on failure.
	* @warning Blocking
	*/
	int initialize();
	
	/**
	* @brief Commands the sensor to start taking a reading.
	* @details This does not get the reading, as there is a delay between when the reading is taken
	* and when the range is available. The minimum value of this is #ULTRASONIC_READ_TIME_MS, so it is not necessary
	* to check until this is elapsed. After this interval, the status pin must be polled until it is low, meaning the device
	* is done taking a range.
	* 
	* This will fail if #ULTRASONIC_OFF_TIME_MS has not elapsed since the last takeReading command.
	* @returns Returns #ERR_OK if the command was sent successfully, or an appropriate error code
	*/
	int takeReading();
	
	/**
	* @brief Gets the lasted available reading from the sensor
	* @details This will fail if #ULTRASONIC_READ_TIME_MS has not passed since the last takeReading command.
	*/
	int getReading( uint16_t rangeReading );
	
	/**
	* @brief This will change the devices address.
	* @details This will update the address stored by the class to avoid losing communication. This should not be done repeatedly, as it will
	* * wear out the EEPROM (source: datasheet).
	* 
	* After this command is executed, we must wait #ULTRASONIC_RESET_DELAY_MS for the value to save and device to reset. This will be blocking
	* @warning Blocking for #ULTRASONIC_RESET_DELAY_MS plus time to take reading, ~#ULTRASONIC_READ_TIME_MS
	*/
	int setAddress( uint8_t newAddress );
};
