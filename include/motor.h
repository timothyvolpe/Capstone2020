#pragma once

#include <stdint.h>
#include <string>

/**
* @file motor.h
* @brief Contains code that handles the interfacing with the motor controllers.
*
* @authors Timothy Volpe
* @date 2/19/2020
*/

/** The minimum valid address. */
#define ROBOCLAW_ADDRESS_MIN 0x80
/** The maximum valid address. There are 8 possible addresses. */
#define ROBOCLAW_ADDRESS_MAX 0x87

/** How often to update the motor controllers, in Hz */
#define MOTOR_UPDATE_FREQUENCY 1

enum RoboClawCommand : unsigned char
{
	MOTOR1_FORWARD			= 0,
	MOTOR1_REVERSE			= 1,
	MOTOR1_DRIVE			= 6,
	MOTOR2_FORWARD			= 4,
	MOTOR2_REVERSE			= 5,
	MOTOR2_DRIVE			= 7,
	
	READ_FIRMWARE			= 21,
	READ_VOLTAGE_MAIN		= 24,
	READ_VOLTAGE_LOGICB		= 25,
	
	READ_MOTOR_PWM			= 48,
	READ_MOTOR_CURRENT		= 49,
	
	READ_SETTINGS_MAIN_VOLTAGE	= 59,
	READ_SETTINGS_LOGIC_VOLTAGE	= 60,
	
	READ_TEMPERATURE		= 82,
	READ_TEMPERATURE2		= 83,
	
	READ_STATUS				= 90,
	
	SET_VOLTAGE_LOGICMIN	= 26,
	SET_VOLTAGE_LOGICMAX	= 27,
	SET_VOLTAGE_MAIN		= 57,
	SET_VOLTAGE_LOGIC		= 58
};

class CUARTChannel;

/**
* @brief The available channels on a Roboclaw motor controller
*/
enum RoboClawChannels
{
	CHANNEL1,
	CHANNEL2
};

/**
* @brief The binary packet data sent over the UART
*/
struct RoboClawPacket
{
	unsigned char 	address;
	unsigned char 	command;
	std::vector<unsigned char> valueBytes;
	unsigned char 	crc[2];
};

/**
* @brief Generates the CRC16 checksum used for RoboClaw communication, using a packet struct.
* @details The entire packet is checksummed, including address and command.
* @param[in]	packet	The packet to checksum
* @returns A two-byte checksum of the packet data.
*/
uint16_t roboclaw_crc16( RoboClawPacket packet );

/**
* @brief Generates the CRC16 checksum used for RoboClaw communication, from bytes.
* @details Primarily used to verify responses are correct.
* @param[in]	packet	The bytes to checksum.
* @returns A two-byte checksum of the byte data.
*/
uint16_t roboclaw_crc16( const std::vector<unsigned char> &buffer );

/**
* @brief RoboClaw Basicmicro 2 channel motor controller interface.
* @details This class can communicate via UART with a RoboClaw motor controller.
*
* @author Timothy Volpe
* @date 2/19/2020
*/
class CMotorController
{
private:
	/** Do not delete this, it should be a reference to one owned by CVehicle. */
	CUARTChannel *m_pMotorUARTReference;
	
	unsigned char m_motorAddress;
	
	char m_lastForwardSpeedSet;
	char m_lastReverseSpeedSet;
	
	/**
	* @brief Sends a blocking command to the controller with arbitrary value.
	* @details Used by the other specific sized commands. Commands with no value will not be checksummed, as per manual.
	* @param[in]	command			The command value.
	* @param[in]	responseLength	The expected response length, it will read up to this many bytes, but possibly less.
	* @param[out]	response		The response if the command, if there was any.
	* @returns Returns #ERR_OK if success, error code if otherwise.
	* @warning This flushes the UART channel.
	*/
	int sendCommandARBBlocking( unsigned char command, const std::vector<unsigned char> &valueBytes, size_t responseLength, std::vector<unsigned char> &response );
	
	/**
	* @brief Sends a blocking command to the controller with no value.
	* @param[in]	command			The command value.
	* @param[in]	responseLength	The expected response length, it will read up to this many bytes, but possibly less.
	* @param[out]	response		The response if the command, if there was any.
	* @returns Returns #ERR_OK if success, error code if otherwise.
	* @warning This flushes the UART channel.
	*/
	int sendCommandBlocking( unsigned char command, size_t responseLength, std::vector<unsigned char> &response );
	
	/**
	* @brief Sends a blocking command to the controller with 8-bit argument.
	* @param[in]	command			The command value.
	* @param[in]	value			The 8-bit command argument.
	* @param[in]	responseLength	The expected response length, it will read up to this many bytes, but possibly less.
	* @param[out]	response		The response if the command, if there was any.
	* @returns Returns #ERR_OK if success, error code if otherwise.
	* @warning This flushes the UART channel.
	*/
	int sendCommand8Blocking( unsigned char command, uint8_t value, size_t responseLength, std::vector<unsigned char> &response );
	
	/**
	* @brief Sends a blocking command to the controller with 16-bit argument.
	* @param[in]	command			The command value.
	* @param[in]	value			The 16-bit command argument.
	* * @param[in]	responseLength	The expected response length, it will read up to this many bytes, but possibly less.
	* @param[out]	response		The response if the command, if there was any.
	* @returns Returns #ERR_OK if success, error code if otherwise.
	* @warning This flushes the UART channel.
	*/
	int sendCommand16Blocking( unsigned char command, uint16_t value, size_t responseLength, std::vector<unsigned char> &response );
	
	/**
	* @brief Sends a blocking command to the controller with 32-bit argument.
	* @param[in]	command			The command value.
	* @param[in]	value			The 32-bit command argument.
	* @param[in]	responseLength	The expected response length, it will read up to this many bytes, but possibly less.
	* @param[out]	response		The response if the command, if there was any.
	* @returns Returns #ERR_OK if success, error code if otherwise.
	* @warning This flushes the UART channel.
	*/
	int sendCommand32Blocking( unsigned char command, uint32_t value, size_t responseLength, std::vector<unsigned char> &response );
	
	/**
	* @brief Converts packet to serial binary data.
	* @details Generates CRC and converts packet to binary buffer for sending over UART.
	* @param[in]	packet	The packet to serialize. Sets the CRC value.
	* @return Buffer containing packet binary data.
	*/
	std::vector<unsigned char> serializePacket( RoboClawPacket &packet );
	
	/**
	* @brief Checks if the response contains a valid checksum for the data or acknowledgement byte.
	* @details The response can either contain a single byte, 0xFF, acknowledging the command, or if multiple bytes, a 2-byte checksum
	* at the end of the reponse that verifies its contents.
	* @param[in]	command		The command used to get the response.
	* @param[in]	response	The expected response from the UART channel.
	* @return Returns true if the response was validated, false if otherwise.
	*/
	bool verifyResponse( unsigned char command, const std::vector<unsigned char> &response );
public:
	CMotorController( CUARTChannel *pUART, unsigned char address );
	~CMotorController();

	int init();
	void shutdown();
	
	/**
	* @brief Gets the motor controller information.
	* @details Retrieves the firmware version, can be used to check validity of communication.
	* @param[out]	versionStr	A string containing version info from the device
	* @returns Returns #ERR_OK if successfully read firmware version, or an appropriate error code if a failure occured.
	*/
	int getControllerInfo( std::string &versionStr );
	
	/**
	* @brief Gets the motor controller's status code.
	* @details Retrives two bytes from the motor controller representing the status. See the Roboclaw User Manual
	* for a description of the codes.
	* @param[out]	pStatus		The retrieved statsu controller is stored here.
	* @returns Returns #ERR_OK if successfully read controller status, or an appropriate error code if a failure occured.
	*/
	int getControllerStatus( uint16_t *pStatus );
	
	/**
	* @brief Sets the selected channel to a forward speed
	* @details The Roboclaw has two motor channels, identified by #RoboClawChannels.
	* 	The speed can be any value between 0 and 127, with 0 being full stop, and 64 being half power.
	* 	Values outside this range will be assumed to be 0 for safety reasons.
	* @param[in]	channelId	The channel to modify, from #RoboClawChannels
	* @param[in]	speed		The speed to set the channel to, 0-127
	* @returns Returns #ERR_OK if successfully modified motor channel, or an appropriate error code if a failure occured.
	*/
	int forward( RoboClawChannels channelId, char speed );
	
	/**
	* @brief Sets the selected channel to a reverse speed
	* @details The Roboclaw has two motor channels, identified by #RoboClawChannels.
	* 	The speed can be any value between 0 and 127, with 0 being full stop, and 64 being half power.
	* 	Values outside this range will be assumed to be 0 for safety reasons.
	* @param[in]	channelId	The channel to modify, from #RoboClawChannels
	* @param[in]	speed		The speed to set the channel to, 0-127
	* @returns Returns #ERR_OK if successfully modified motor channel, or an appropriate error code if a failure occured.
	*/
	int reverse( RoboClawChannels channelId, char speed );
};
