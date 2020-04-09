#pragma once
#include <chrono>

/** The factory default address of the sensors, or the address when forced to default. */
#define ULTRASONIC_DEFAULT_ADDRESS 112
/** The first part of the address unlock command. */
#define ULTRASONIC_ADDR_UNLOCK_1 170
/** The second part of the address unlock command. */
#define ULTRASONIC_ADDR_UNLOCK_2 165

/** Command to take a range reading. */
#define ULTRASONIC_COMMAND_RANGE 81
/** Command to report last range reading. */
#define ULTRASONIC_COMMAND_READ 255
/** Command to change the sensor address */
#define ULTRASONIC_COMMAND_CHANGE_ADDRESS 224
/** The first part of the address unlock command. */
#define ULTRASONIC_COMMAND_ADDR_UNLOCK_1 170
/** The second part of the address unlock command. */
#define ULTRASONIC_COMMAND_ADDR_UNLOCK_2 165

/** The minimum wait time between taking a reading and getting the value, per datasheet (ms) */
#define ULTRASONIC_READ_TIME_MS 80
/** The minimum time between consecutive readings, per datasheet (ms) */
#define ULTRASONIC_OFF_TIME_MS 100
/** Time to wait for device to restart when address is changed, per datasheet (ms) */
#define ULTRASONIC_RESET_DELAY_MS 125

/** The number of consecutive missed readings until a fatal error occurs. */
#define ULTRASONIC_MISSED_READING_LIMIT 3
/** The maximum age of a reading before a fatal error is thrown. This needs to be long enough to account for startup. */
#define ULTRASONIC_READING_MAX_AGE_MS 5000

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
* The process for taking continous readings without blocking is as follows:
* 1. On update, instruct sensor to take a reading, do not wait.
* 2. On next update, check to make sure sensor acknowledged (ACK) range command. If not, go back to 1.
* 3. On consecutive updates, check until #ULTRASONIC_READ_TIME_MS has passed since range command ACK. When
* this occurs, instruct the device to return the range reading.
* 4. On next update, check to make sure sensor acknowledged (ACK) the read command. If not, go back to 3. If this did occur,
* save the reading.
* 5. On consecutive updates, check until #ULTRASONIC_OFF_TIME_MS has passed since read command ACK. When this occurs, go back to 1.
* 
* If a reading is missed, meaning ACK was not received, then the missed counter is incremented. When it reaches #ULTRASONIC_MISSED_READING_LIMIT, a fatal
* error will occur. This margin of error is allowed incase the program freezes up or the i2c command is delayed or not sent for some reason.
*
* @author Timothy Volpe
* @date 1/29/2020
*/
class CUltrasonicSensor
{
private:
	enum UltrasonicSensorState
	{
		STATE_TAKE_READING,
		STATE_VERIFY_TAKE_COMMAND,
		STATE_READ_READING,
		STATE_VERIFY_READ_COMMAND
	};

	CI2CBus *m_pI2CBus;
	
	unsigned char m_sensorAddress;
	
	UltrasonicSensorState m_sensorState;
	int m_consecMissedReadings;
	
	uint16_t m_lastRangeValue;
	
	// Time since last range command was ACK
	std::chrono::steady_clock::time_point m_lastRange;
	// Time since last reading was successfully received
	std::chrono::steady_clock::time_point m_lastReadingTaken;
	
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
	* @brief Command the sensor to read the last range reading.
	* @details This will fail if #ULTRASONIC_READ_TIME_MS has not passed since the last takeReading command.
	* @returns Returns #ERR_OK if the command was sent successfully, or an appropriate error code
	*/
	int getReading();
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
	* @brief Update the sensor. See update procedure in the class description.
	* @returns Returns #ERR_OK if successful, or appropriate error code on failure.
	*/
	int update();
	
	/**
	* @brief This will change the devices address.
	* @details This will update the address stored by the class to avoid losing communication. This should not be done repeatedly, as it will
	* wear out the EEPROM (source: datasheet).
	* 
	* After this command is executed, we must wait #ULTRASONIC_RESET_DELAY_MS for the value to save and device to reset. This will be blocking
	* @returns Returns #ERR_OK if the command was carried out successfully, or an appropriate error code
	* @warning Blocking for #ULTRASONIC_RESET_DELAY_MS plus time to take reading, ~#ULTRASONIC_READ_TIME_MS
	*/
	int setAddress( uint8_t newAddress );
	
	/**
	* @brief Prints the last reading from the sensor.
	* @details This reading will be younger than #ULTRASONIC_READING_MAX_AGE_MS.
	* @returns Last range reading from ultrasonic sensor.
	*/
	uint16_t getLastReading();
};
