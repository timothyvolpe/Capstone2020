#pragma once

#include <string>
#ifdef __linux__
#include <linux/i2c-dev.h>
//#include <i2c/smbus.h>
#endif

/**
* @file wire_protocols.h
* @brief Contains handlers for common wire communication protocols (I2C, UART, SPI).
* @warning This file does nothing on Windows.
*
* @authors Timothy Volpe
*
* @date 2/27/2020
*/

/**
* @brief Handles an I2C bus.
* @details This class can handle a single I2C bus with multiple devices connected to it.
*
* @author Timothy Volpe
* @date 2/27/2020
*/
class CI2CBus
{
private:
	int m_hBusHandle;
public:
	/* Default constructor */
	CI2CBus();
	/**
	* @brief Default destructor. Closes bus handle.
	* @details Although the destructor automatically closes the bus handle, it is good practice
	*	to use the CI2CBus::close function.
	*/
	~CI2CBus();

	/**
	* @brief Opens a i2c bus with the given name
	* @param[in]	busPath		A path to the i2c bus to open.
	* @returns Returns #ERR_OK if successfully opened handle to i2c bus, or an appropriate error code if a failure occured.
	*/
	int open( std::string busPath );

	/**
	* @brief Close the i2c bus handle.
	*/
	void close();
};


/**
* @brief Handles a single UART channel.
* @details This can handle one device connected to the UART channel.
*
* @author Timothy Volpe
* @date 2/27/2020
*/
class CUARTChannel
{
private:
	int m_hChannelHandle;

#ifdef __linux__
	termios m_uartOptions;
#endif

public:
	/* Default constructor */
	CUARTChannel();
	/**
	* @brief Default destructor. Closes channel handle.
	* @details Although the destructor automatically closes the channel handle, it is good practice
	*	to use the CUARTChannel::close function.
	*/
	~CUARTChannel();

	/**
	* @brief Opens a UART channel with the given name
	* @param[in]	channelPath		A path to the UART channel to open.
	* @returns Returns #ERR_OK if successfully opened handle to UART bus, or an appropriate error code if a failure occured.
	*/
	int open( std::string channelPath );
	/**
	* @brief Close the UART channel handle.
	*/
	void close();
};