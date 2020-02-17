#pragma once

#include <string>
#ifdef __linux__
#include <linux/i2c-dev.h>
//#include <i2c/smbus.h>
#endif

/**
* @file wire_protocols.h
* @brief Contains handlers for common wire communication protocols (I2C and SPI).
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
	* @details Althought the destructor automatically closes the bus handle, it is good practice
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
