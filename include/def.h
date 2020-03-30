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

/** All terminal errors start at this value */
#define ERR_TERMINAL_BASE				0x1000
/** Failed to get executable directory */
#define ERR_TERMINAL_EXEC_DIR			ERR_TERMINAL_BASE + 0x1
/** Failed to open the terminal output file for writing */
#define ERR_TERMINAL_OUTPUT_FILE		ERR_TERMINAL_BASE + 0x2

/** All initialization errors start at this value. */
#define ERR_INIT_BASE					0x2000
/** Ultrasonic sensor failure to initialize. */
#define ERR_SENSOR_INIT_ULTRASONIC		ERR_INIT_BASE + 0x1
/** LIDAR sensor failed to initialize. */
#define ERR_SENSOR_INIT_LIDAR			ERR_INIT_BASE + 0x2
/** One or more IMU sensors failed to initialize. */
#define ERR_SENSOR_INIT_IMU				ERR_INIT_BASE + 0x3
/** GPS unit failed to initialize. */
#define ERR_SENSOR_INIT_GPS				ERR_INIT_BASE + 0x4

/** All comm errors start at this value */
#define ERR_COMM_BASE					0x3000
/** The system could not access the COMM channel for an unknown reason */
#define ERR_COMM_OPEN_FAILED			ERR_COMM_BASE + 0x1
/** Access was denied to the COMM channel */
#define ERR_COMM_ACCESS_DENIED			ERR_COMM_BASE + 0x2
/** The given COMM path did not point to a valid bus. */
#define ERR_COMM_INVALID_PATH			ERR_COMM_BASE + 0x3
/** The given baud rate is unsupported. */
#define ERR_COMM_INVALID_BAUD			ERR_COMM_BASE + 0x4
/** Failed to set the COMM attributes. */
#define ERR_COMM_SET_ATTRIB				ERR_COMM_BASE + 0x5
/** Failed to flush the COMM channel. */
#define ERR_COMM_FLUSH_CHANNEL			ERR_COMM_BASE + 0x6
/** Failed to set the COMM input flags. */
#define ERR_COMM_SET_IFLAG				ERR_COMM_BASE + 0x7
/** Failed to set the COMM output flags. */
#define ERR_COMM_SET_OFLAG				ERR_COMM_BASE + 0x8
/** Failed to set the COMM control flags. */
#define ERR_COMM_SET_CFLAG				ERR_COMM_BASE + 0x8
/** Failed to set the COMM read timetout. */
#define ERR_COMM_SET_READ_TIMEOUT		ERR_COMM_BASE + 0x9
/** The poll() operation failed. */
#define ERR_COMM_POLL					ERR_COMM_BASE + 0xB
/** An error occured while trying to read from the COMM channel. */
#define ERR_COMM_READ					ERR_COMM_BASE + 0xC
/** The device connected to the COMM did not respond. */
#define ERR_COMM_NO_RESPONSE			ERR_COMM_BASE + 0xD
/** Failed to write to COMM. */
#define ERR_COMM_WRITE					ERR_COMM_BASE + 0xE
/** The response from the COMM device was invalid. */
#define ERR_COMM_INVALID_RESPONSE		ERR_COMM_BASE + 0xF

/** All I2C errors start at this value */
#define ERR_I2C_BASE					0x4000
/** The I2C channel is already open. */
#define ERR_I2C_ALREADY_OPEN			ERR_I2C_BASE + 0x1

/** All SPI errors start at this value */
#define ERR_SPI_BASE					0x5000

/** All UART errors start at this value */
#define ERR_UART_BASE					0x6000
/** The UART channel is already open. */
#define ERR_UART_ALREADY_OPEN			ERR_UART_BASE + 0x1

/** All motor controller errors start at this value. */
#define ERR_MOTOR_BASE					0x7000
/** Unexpected change in motor controller settings */
#define ERR_MOTOR_VERIFY_FAILED				ERR_MOTOR_BASE + 0x1
/** The motor controller version is not supported */
#define ERR_MOTOR_VERSION_MISMATCH			ERR_MOTOR_BASE + 0x2

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
	{ ERR_OK, "No error." },

	{ ERR_TERMINAL_EXEC_DIR, "Failed to get executable directory." },
	{ ERR_TERMINAL_OUTPUT_FILE, "Failed to open the terminal output file for writing." },

	{ ERR_SENSOR_INIT_ULTRASONIC, "Ultrasonic sensor initialization failure." },
	{ ERR_SENSOR_INIT_LIDAR, "LIDAR sensor initialization failure." },
	{ ERR_SENSOR_INIT_IMU, "IMU sensor initialization failure." },
	{ ERR_SENSOR_INIT_GPS, "GPS unit initialization failure." },

	{ ERR_COMM_OPEN_FAILED, "The system could not access the comm channel for an unknown reason." },
	{ ERR_COMM_ACCESS_DENIED, "Access was denied to the channel." },
	{ ERR_COMM_INVALID_PATH, "The given port path did not point to a valid channel." },
	{ ERR_COMM_INVALID_BAUD, "The given baud rate is unsupported." },
	{ ERR_COMM_SET_ATTRIB, "Failed to set the port attributes." },
	{ ERR_COMM_FLUSH_CHANNEL, "Failed to flush the channel." },
	{ ERR_COMM_SET_IFLAG, "Failed to set the port input flags." },
	{ ERR_COMM_SET_OFLAG, "Failed to set the port output flags." },
	{ ERR_COMM_SET_CFLAG, "Failed to set the port control flags." },
	{ ERR_COMM_SET_READ_TIMEOUT, "Failed to set the port read timetout." },
	{ ERR_COMM_POLL, "The poll() operation failed." },
	{ ERR_COMM_READ, "An error occured while trying to read from the channel." },
	{ ERR_COMM_NO_RESPONSE, "The device connected to the port did not respond." },
	{ ERR_COMM_WRITE, "Failed to write to port." },
	{ ERR_COMM_INVALID_RESPONSE, "The response from the device was invalid." },

	{ ERR_I2C_ALREADY_OPEN, "The I2C channel is already open." },

	{ ERR_UART_ALREADY_OPEN, "The COMM channel is already open." },
	
	{ ERR_MOTOR_VERIFY_FAILED, "Unexpected change in motor controller settings" },
	
	{ ERR_MOTOR_VERSION_MISMATCH, "The motor controller version is not supported" }
};

#define ERRMSG_TABLE_LEN  sizeof(ErrorMessageTable)/sizeof(ErrorMessageTable[0])

/**
* @brief Returns an erorr string associated with the code
* @param[in]	errCode	The error code to convert to string literal.	
* @returns An error code associated with errCode
* @warning Passing an invalid code will return a non-descript error string.
*/
const char* GetErrorString( int errCode );
