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
};

#define ERRMSG_TABLE_LEN  sizeof(ErrorMessageTable)/sizeof(ErrorMessageTable[0])

/**
* @brief Returns an erorr string associated with the code
* @param[in]	errCode	The error code to convert to string literal.	
* @returns An error code associated with errCode
* @warning Passing an invalid code will return a non-descript error string.
*/
const char* GetErrorString( int errCode );