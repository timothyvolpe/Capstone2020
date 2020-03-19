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

/** How long to wait for data to be returned from the UART before timing out */
#define MOTOR_UART_WAIT_MS 10
/** The number of times to try contacting the controller again after no response */
#define MOTOR_UART_TRIES 3

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
	READ_VOLTAGE_LOGIC		= 25,
	
	READ_MOTOR_PWMS			= 48,
	READ_MOTOR_CURRENTS		= 49,
	
	SET_MAIN_VOLTAGES		= 57,
	SET_LOGIC_VOLTAGES		= 58,
	
	READ_MAIN_VOLTAGES		= 59,
	READ_LOGIC_VOLTAGES		= 60,
	
	READ_TEMPERATURE		= 82,
	READ_TEMPERATURE2		= 83,
	
	READ_STATUS				= 90,
	
	READ_STANDARD_CONFIG	= 99,
};

enum RoboClawSettings : uint16_t
{
	RC_MODE				= 0x0000,
	ANALOG_MODE			= 0x0001,
	SIMPLE_SERIAL_MODE	= 0x0002,
	PACKET_SERIAL_MODE	= 0x0003,
	BATTERY_MODE_OFF	= 0x0000,
	BATTERY_MODE_AUTO	= 0x0004,
	BATTERY_MODE_2CELL	= 0x0008,
	BATTERY_MODE_3CELL	= 0x000C,
	BATTERY_MODE_4CELL	= 0x0010,
	BATTERY_MODE_5CELL	= 0x0014,
	BATTERY_MODE_6CELL	= 0x0018,
	BATTERY_MODE_7CELL	= 0x001C,
	MIXING				= 0x0020,
	EXPONENTIAL			= 0x0040,
	MCU					= 0x0080,
	BAUDRATE_2400		= 0x0000,
	BAUDRATE_9600		= 0x0020,
	BAUDRATE_19200		= 0x0040,
	BAUDRATE_38400		= 0x0060,
	BAUDRATE_57600		= 0x0080,
	BAUDRATE_115200		= 0x00A0,
	BAUDRATE_230400		= 0x00C0,
	BAUDRATE_460800		= 0x00E0,
	FLIPSWITCH			= 0x0100,
	PACKET_ADDRESS_80	= 0x0000,
	PACKET_ADDRESS_81	= 0x0100,
	PACKET_ADDRESS_82	= 0x0200,
	PACKET_ADDRESS_83	= 0x0300,
	PACKET_ADDRESS_84	= 0x0400,
	PACKET_ADDRESS_85	= 0x0500,
	PACKET_ADDRESS_86	= 0x0600,
	PACKET_ADDRESS_87	= 0x0700,
	SLAVE_MODE			= 0x0800,
	RELAY_MODE			= 0x1000,
	SWAP_ENCODERS		= 0x2000,
	SWAP_BUTTONS		= 0x4000,
	MULTI_UNIT_MODE		= 0x8000
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
	* @param[out]	response		The response of the command, if there was any.
	* @returns Returns #ERR_OK if success, error code if otherwise.
	* @warning This flushes the UART channel.
	*/
	int sendCommandBlocking( unsigned char command, size_t responseLength, std::vector<unsigned char> &response );
	
	/**
	* @brief Sends a blocking command to the controller with 8-bit argument.
	* @param[in]	command			The command value.
	* @param[in]	value			The 8-bit command argument.
	* @param[in]	responseLength	The expected response length, it will read up to this many bytes, but possibly less.
	* @param[out]	response		The response of the command, if there was any.
	* @returns Returns #ERR_OK if success, error code if otherwise.
	* @warning This flushes the UART channel.
	*/
	int sendCommand8Blocking( unsigned char command, uint8_t value, size_t responseLength, std::vector<unsigned char> &response );
	
	/**
	* @brief Sends a blocking command to the controller with 16-bit argument.
	* @param[in]	command			The command value.
	* @param[in]	value			The 16-bit command argument.
	* * @param[in]	responseLength	The expected response length, it will read up to this many bytes, but possibly less.
	* @param[out]	response		The response of the command, if there was any.
	* @returns Returns #ERR_OK if success, error code if otherwise.
	* @warning This flushes the UART channel.
	*/
	int sendCommand16Blocking( unsigned char command, uint16_t value, size_t responseLength, std::vector<unsigned char> &response );
	
	/**
	* @brief Sends a blocking command to the controller with two 16-bit arguments.
	* @param[in]	command			The command value.
	* @param[in]	value1			The 1st 16-bit command argument.
	* @param[in]	value2			The 2nd 16-bit command argument.
	* * @param[in]	responseLength	The expected response length, it will read up to this many bytes, but possibly less.
	* @param[out]	response		The response of the command, if there was any.
	* @returns Returns #ERR_OK if success, error code if otherwise.
	* @warning This flushes the UART channel.
	*/
	int sendCommand1616Blocking( unsigned char command, uint16_t value1, uint16_t value2, size_t responseLength, std::vector<unsigned char> &response );
	
	/**
	* @brief Sends a blocking command to the controller with 32-bit argument.
	* @param[in]	command			The command value.
	* @param[in]	value			The 32-bit command argument.
	* @param[in]	responseLength	The expected response length, it will read up to this many bytes, but possibly less.
	* @param[out]	response		The response of the command, if there was any.
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
	* @brief Reads the two temperatures available from the controller.
	* @details Temperature 1 is always available, while temperature 2 is only on supported units. Only the first
	* argument is required
	* @param[out]	pTemp1	The value of board temperature 1, in degrees C.
	* @param[out]	pTemp2	The value of board temperature 2, in degrees C. Can be NULL.
	* @returns Returns #ERR_OK if successfully read controller temperature, or an appropriate error code if a failure occured.
	*/
	int getTemperature( float *pTemp1, float *pTemp2 );
	
	/**
	* @brief Reads the main battery voltage, in volts.
	* @param[out]	pVoltage	The voltage of the main battery
	* @returns Returns #ERR_OK if successfully read main battery voltage, or an appropriate error code if a failure occured.
	*/
	int getMainBatteryVoltage( float *pVoltage );
	
	/**
	* @brief Reads the logic battery voltage, in volts.
	* @param[out]	pVoltage	The voltage of the logic battery
	* @returns Returns #ERR_OK if successfully read logic battery voltage, or an appropriate error code if a failure occured.
	*/
	int getLogicBatteryVoltage( float *pVoltage );
	
	/**
	* @brief Read config bits for standard settings.
	* @details See the RoboClaw user main for description of the response.
	* @param[out]	pConfigSettings		A two-byte integer representing the current settings.
	* @returns Returns #ERR_OK if successfully read controller settings, or an appropriate error code if a failure occured.
	*/
	int getConfigSettings( uint16_t *pConfigSettings );
	
	/**
	* @brief Gets the current of each motor channel, in A.
	* @details The resolution is 10 mA.
	* @param[out]	pCurrentM1	The current of motor 1, in A.
	* @param[out]	pCurrentM2	The current of motor 2, in A.
	* @returns Returns #ERR_OK if successfully read motor currents, or an appropriate error code if a failure occured.
	*/
	int getMotorCurrents( float *pCurrentM1, float *pCurrentM2 );
	
	/**
	* @brief Gets the PWM duty cycle of each motor channel.
	* @details The values range from 0, representing 0%, to 100, representing 100%.
	* @param[out]	pDuty1	The PWM duty cycle of motor 1.
	* @param[out]	pDuty2	The PWM duty cycle of motor 2.
	* @returns Returns #ERR_OK if successfully read motor duty cycles, or an appropriate error code if a failure occured.
	*/
	int getMotorDutyCycles( float *pDuty1, float *pDuty2 );
	
	/**
	* @brief Sets the minimum and maximum main battery voltage levels.
	* @details If the main voltage level is outside this range, it will cause a shutdown. Accurate to the 10th of a volt.
	* @param[in]	mainMin		The minimum logic voltage level. 
	* @param[in]	mainMax		The maximum logic voltage level.
	* @warning If minimum > maximum, the controller we be constantly in an error state.
	* @returns Returns #ERR_OK if successfully set main battery voltage level limits, or an appropriate error code if a failure occured.
	*/
	int setMainVoltageLevels( float mainMin, float mainMax );
	/**
	* @brief Sets the minimum and maximum logic battery voltage levels.
	* @details If the logic voltage level is outside this range, it will cause a shutdown. Accurate to the 10th of a volt.
	* @param[in]	logicMin	The minimum logic voltage level. 
	* @param[in]	logicMax	The maximum logic voltage level.
	* @warning If minimum > maximum, the controller we be constantly in an error state.
	* @returns Returns #ERR_OK if successfully set logic battery voltage level limits, or an appropriate error code if a failure occured.
	*/
	int setLogicVoltageLevels( float logicMin, float logicMax );
	
	/**
	* @brief Gets the minimum and maximum main battery voltage levels. 
	* @param[out]	pMainMin	The minimum allowable main voltage level.
	* @param[out]	pMainMax	The maximum allowable main voltage level.
	* @returns Returns #ERR_OK if successfully got the main battery voltage level limits, or an appropriate error code if a failure occured.
	*/
	int getMainVoltageLevels( float *pMainMin, float *pMainMax );
	
	/**
	* @brief Gets the minimum and maximum logic battery voltage levels. 
	* @param[out]	pLogicMin	The minimum allowable logic voltage level.
	* @param[out]	pLogicMax	The maximum allowable logic voltage level.
	* @returns Returns #ERR_OK if successfully got the logic battery voltage level limits, or an appropriate error code if a failure occured.
	*/
	int getLogicVoltageLevels( float *pLogicMin, float *pLogicMax );
	
	/**
	* @brief Sets the selected channel to a forward speed
	* @details The Roboclaw has two motor channels, identified by #RoboClawChannels.
	* 	The speed can be any value between 0 and 127, with 0 being full stop, and 64 being half power.
	* 	Values outside this range will be assumed to be 0 for safety reasons.
	* @param[in]	channelId	The channel to modify, from #RoboClawChannels
	* @param[in]	speed		The speed to set the channel to, 0-127
	* @returns Returns #ERR_OK if successfully modified motor channel, or an appropriate error code if a failure occured.
	*/
	int forward( RoboClawChannels channelId, int8_t speed );
	
	/**
	* @brief Sets the selected channel to a reverse speed
	* @details The Roboclaw has two motor channels, identified by #RoboClawChannels.
	* 	The speed can be any value between 0 and 127, with 0 being full stop, and 64 being half power.
	* 	Values outside this range will be assumed to be 0 for safety reasons.
	* @param[in]	channelId	The channel to modify, from #RoboClawChannels
	* @param[in]	speed		The speed to set the channel to, 0-127
	* @returns Returns #ERR_OK if successfully modified motor channel, or an appropriate error code if a failure occured.
	*/
	int reverse( RoboClawChannels channelId, int8_t speed );
};

/**
* @brief Manages all the motors to move the vehicle.
* @details This class handles all the motor controllers and allows the vehicle to move as one.
*
* @author Timothy Volpe
* @date 3/15/2020
*/
class CMotionManager
{
public:
	CMotionManager();
	~CMotionManager();
};
