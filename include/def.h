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

/** All I2C errors start at this value */
#define ERR_I2C_BASE					0x4000
/** The I2C channel is already open. */
#define ERR_I2C_ALREADY_OPEN			ERR_I2C_BASE + 0x1
/** The given port does not appear to support I2C communication. */
#define ERR_I2C_NO_SUPPORT				ERR_I2C_BASE + 0x2
/** Failed to write to the i2c port. */
#define ERR_I2C_WRITE_TO_PORT			ERR_I2C_BASE + 0x3

/** All SPI errors start at this value */
#define ERR_SPI_BASE					0x5000

/** All UART errors start at this value */
#define ERR_UART_BASE					0x6000
/** The UART channel is already open. */
#define ERR_UART_ALREADY_OPEN			ERR_UART_BASE + 0x1
/** The given baud rate is unsupported. */
#define ERR_UART_INVALID_BAUD			ERR_UART_BASE + 0x2
/** Failed to set the UART attributes. */
#define ERR_UART_SET_ATTRIB				ERR_UART_BASE + 0x3
/** Failed to flush the UART channel. */
#define ERR_UART_FLUSH_CHANNEL			ERR_UART_BASE + 0x4
/** Failed to set the UART input flags. */
#define ERR_UART_SET_IFLAG				ERR_UART_BASE + 0x5
/** Failed to set the UART output flags. */
#define ERR_UART_SET_OFLAG				ERR_UART_BASE + 0x6
/** Failed to set the UART control flags. */
#define ERR_UART_SET_CFLAG				ERR_UART_BASE + 0x7
/** Failed to set the UART read timetout. */
#define ERR_UART_SET_READ_TIMEOUT		ERR_UART_BASE + 0x8
/** The poll() operation failed. */
#define ERR_UART_POLL					ERR_UART_BASE + 0x9
/** An error occured while trying to read from the UART channel. */
#define ERR_UART_READ					ERR_UART_BASE + 0xA
/** The device connected to the UART channel did not respond. */
#define ERR_UART_NO_RESPONSE			ERR_UART_BASE + 0xB
/** Failed to write to UART channel. */
#define ERR_UART_WRITE					ERR_UART_BASE + 0xC
/** The response from the UART device was invalid. */
#define ERR_UART_INVALID_RESPONSE		ERR_UART_BASE + 0xD

/** All motor controller errors start at this value. */
#define ERR_MOTOR_BASE					0x7000
/** Unexpected change in motor controller settings */
#define ERR_MOTOR_VERIFY_FAILED			ERR_MOTOR_BASE + 0x1
/** The motor controller version is not supported */
#define ERR_MOTOR_VERSION_MISMATCH		ERR_MOTOR_BASE + 0x2

/** All config errors start at this value */
#define ERR_CONFIG_BASE					0x8000
/** Failed to read the config file. */
#define ERR_CONFIG_FILE					ERR_CONFIG_BASE + 0x1
/** There was a syntax error in the config file. */
#define ERR_CONFIG_SYNTAX				ERR_CONFIG_BASE + 0x2

/** All ultrasonic sensor erors start at this value */
#define ERR_ULTRASONIC_BASE				0x9000
/** The ultrasonic sensor is not ready to take the next reading */
#define ERR_ULTRASONIC_NOT_READY		ERR_ULTRASONIC_BASE + 0x1
/** There was a failure taking an ultrasonic range reading. */
#define ERR_ULTRASONIC_RANGE			ERR_ULTRASONIC_BASE + 0x2

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

	{ ERR_I2C_ALREADY_OPEN, "The I2C channel is already open." },
	{ ERR_I2C_NO_SUPPORT, "The given port does not appear to support I2C communication." },
	{ ERR_I2C_WRITE_TO_PORT, "Failed to write to the i2c port." },

	{ ERR_UART_ALREADY_OPEN, "The UART channel is already open." },
	{ ERR_UART_INVALID_BAUD, "The given baud rate is unsupported." },
	{ ERR_UART_SET_ATTRIB, "Failed to set the UART attributes." },
	{ ERR_UART_FLUSH_CHANNEL, "Failed to flush the UART channel." },
	{ ERR_UART_SET_IFLAG, "Failed to set the UART input flags." },
	{ ERR_UART_SET_OFLAG, "Failed to set the UART output flags." },
	{ ERR_UART_SET_CFLAG, "Failed to set the UART control flags." },
	{ ERR_UART_SET_READ_TIMEOUT, "Failed to set the UART read timetout." },
	{ ERR_UART_POLL, "The poll() operation failed." },
	{ ERR_UART_READ, "An error occured while trying to read from the UART channel." },
	{ ERR_UART_NO_RESPONSE, "The device connected to the UART did not respond." },
	{ ERR_UART_WRITE, "Failed to write to UART." },
	{ ERR_UART_INVALID_RESPONSE, "The response from the UART device was invalid." },
	
	{ ERR_MOTOR_VERIFY_FAILED, "Unexpected change in motor controller settings." },
	{ ERR_MOTOR_VERSION_MISMATCH, "The motor controller version is not supported." },
	
	{ ERR_CONFIG_FILE, "Failed to read the config file." },
	{ ERR_CONFIG_SYNTAX, "There was a syntax error in the config file." },
	
	{ ERR_ULTRASONIC_NOT_READY, "The ultrasonic sensor is not ready to take the next reading." },
	{ ERR_ULTRASONIC_RANGE, "There was a failure taking an ultrasonic range reading." }
};

#define ERRMSG_TABLE_LEN  sizeof(ErrorMessageTable)/sizeof(ErrorMessageTable[0])

/**
* @brief Returns an erorr string associated with the code
* @param[in]	errCode	The error code to convert to string literal.	
* @returns An error code associated with errCode
* @warning Passing an invalid code will return a non-descript error string.
*/
const char* GetErrorString( int errCode );
