#pragma once

#include <stdint.h>

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
	char 			value;
	unsigned char 	crc[2];
} __attribute__((packed));

/**
* @brief Generates the CRC16 checksum used for RoboClaw communication.
* @details The entire packet is checksummed, including address and command.
* @param[in]	packet	The packet to checksum
* @returns A two-byte checksum of the packet data.
*/
uint16_t robowclaw_crc16( RoboClawPacket packet );

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
public:
	CMotorController( CUARTChannel *pUART, unsigned char address );
	~CMotorController();

	int init();
	void shutdown();
	
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
