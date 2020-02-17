#pragma once

/**
* @file def.h
* @brief Contains definitions used by entire program.
* @details This includes error codes, error message lookup tables, and message loop message identifiers.
*
* @authors Timothy Volpe
*
* @date 1/30/2020
*/

/////////////////
// Error Codes //
/////////////////


/** Operation completed successfully */
#define ERR_OK							0x0000

/** All initialization errors start at this value. */
#define ERR_INIT_BASE					0x1000
/** Ultrasonic sensor failure to initialize. */
#define ERR_SENSOR_INIT_ULTRASONIC		ERR_INIT_BASE + 0x1
/** LIDAR sensor failed to initialize. */
#define ERR_SENSOR_INIT_LIDAR			ERR_INIT_BASE + 0x2
/** One or more IMU sensors failed to initialize. */
#define ERR_SENSOR_INIT_IMU				ERR_INIT_BASE + 0x3
/** GPS unit failed to initialize. */
#define ERR_SENSOR_INIT_GPS				ERR_INIT_BASE + 0x4

/** All I2C errors start at this value */
#define ERR_I2C_BASE					0x2000
/** The system could not access the I2C bus for an unknown reason */
#define ERR_I2C_OPEN_FAILED				ERR_I2C_BASE + 0x1
/** Access was denied to the I2C bus */
#define ERR_I2C_ACCESS_DENIED			ERR_I2C_BASE + 0x2
/** The given I2C path did not point to a valid bus. */
#define ERR_I2C_INVALID_PATH			ERR_I2C_BASE + 0x3

/** All SPI errors start at this value */
#define ERR_SPI_BASE					0x3000

/**
* @brief Combines error code and error message for lookup
*/
typedef struct {
	/** The error code */
	int code;
	/** The error message */
	const char* message;
} error_message_t;

static const error_message_t ErrorMessageTable[] = {
	{ ERR_OK, "SUCCESS" },
	{ ERR_SENSOR_INIT_ULTRASONIC, "Ultrasonic sensor initialization failure" },
	{ ERR_SENSOR_INIT_LIDAR, "LIDAR sensor initialization failure" },
	{ ERR_SENSOR_INIT_IMU, "IMU sensor initialization failure" },
	{ ERR_SENSOR_INIT_GPS, "GPS unit initialization failure" },

	{ ERR_I2C_OPEN_FAILED, "The system could not access the I2C bus for an unknown reason" },
	{ ERR_I2C_ACCESS_DENIED, "Access was denied to the I2C bus" },
	{ ERR_I2C_INVALID_PATH, "The given I2C path did not point to a valid bus" }
};

#define ERRMSG_TABLE_LEN  sizeof(ErrorMessageTable)/sizeof(ErrorMessageTable[0])

/**
* @brief Returns an erorr string associated with the code
* @param[in]	errCode	The error code to convert to string literal.	
* @returns An error code associated with errCode
* @warning Passing an invalid code will return a non-descript error string.
*/
const char* GetErrorString( int errCode );