#pragma once

#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <queue>
#ifdef __linux__
#include <linux/i2c-dev.h>
#include <termios.h>
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
* 	The device will poll for UART events in a seperate thread, so that it is non-blocking.
*
* @author Timothy Volpe
* @date 2/27/2020
*/
class CUARTChannel
{
private:
	typedef std::queue<std::vector<unsigned char>> WriteQueue;

	int m_hChannelHandle;
	
	std::thread m_uartThread;
	std::atomic<bool> m_threadRunning;

	std::mutex m_writeMutex;
	WriteQueue m_writeBuffer;
	std::mutex m_readMutex;
	std::queue<unsigned char> m_readBuffer;

#ifdef __linux__
	termios m_uartOptions;
	
	bool setAttributes();
#endif

	void uartThreadMain();
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
	* @param[in]	enableReceiver	Enable the RX pin.
	* @param[in]	twoStopBits		Use two stop bits instead of one.
	* @param[in]	parity			Use parity error checking.
	* @param[in]	rtscts			Use RTS/CTS.
	* @returns Returns #ERR_OK if successfully opened handle to UART bus, or an appropriate error code if a failure occured.
	*/
	int open( std::string channelPath, bool enableReceiver, bool twoStopBits, bool parity, bool rtscts );
	/**
	* @brief Close the UART channel handle.
	*/
	void close();
	
	/**
	* @brief Returns true if a channel is open.
	* @returns True if the channel is open.
	*/
	bool isOpen();

	/**
	* @brief Write to the UART channel.
	* @details This will add buffer to the write buffer, to be sent as soon as the channel is avaliable.
	* @param[in]	buffer	Data to be added to the write buffer.
	* @returns Returns true if successfully added to queue, false if channel was not open.
	*/
	bool write( std::vector<unsigned char> buffer );

	/**
	* @brief Reads a certain number of bytes from the read buffer.
	* @details This will read count number of bytes from the read buffer and return them. If there are not count
	*	bytes in the buffer, the entire contents of the read buffer will be returned. The read buffer is filled continuously
	*	as data is read from the port. 
	* @param[in]	count	The maximum number of bytes to be read.
	* @returns A buffer containing up to count number of bytes, however an empty buffer may be returned if no bytes were available.
	*/
	std::vector<unsigned char> read( size_t count );

#ifdef __linux__
	/**
	* @brief Flushes the input and output, clears buffers.
	* @returns Returns #ERR_OK if successfully flushed the channel, or an appropriate error code if a failure occured.
	*/
	int flush();

	/**
	* @brief Sets the baud rate of the UART.
	* @details Sets the speed to the input and output lines. This will flush the channel first.
	* @param[in]	baud	The baud rate constant
	* @warning Flushes the input and output.
	* @returns Returns #ERR_OK if successfully set baud rate, or an appropriate error code if a failure occured.
	*/
	int setBaudRate( speed_t baud );
	
	/**
	* @brief Sets the read timeout of the UART.
	* @details This will flush the channel first.
	* @param[in]	deciseconds		The read timeout in deciseconds.
	* @warning Flushes the input and output.
	* @returns Returns #ERR_OK if successfully set read timeout, or an appropriate error code if a failure occured.
	*/
	int setReadTimeout( cc_t deciseconds ); 
	
	/**
	* @brief Sets the attribute iflag.
	* @details This will flush the channel first.
	* @param[in]	iflag	The iflags to set.
	* @warning Flushes the input and output.
	* @returns Returns #ERR_OK if successfully set iflag, or an appropriate error code if a failure occured. 
	*/
	int setiFlag( tcflag_t iflag );
	/**
	* @brief Returns the attribute iflag.
	* @returns The attribute iflag.
	*/
	tcflag_t getiFlag();
	
	/**
	* @brief Sets the attribute oflag.
	* @details This will flush the channel first.
	* @param[in]	oflag	The oflags to set.
	* @warning Flushes the input and output.
	* @returns Returns #ERR_OK if successfully set oflag, or an appropriate error code if a failure occured. 
	*/
	int setoFlag( tcflag_t oflag );
	/**
	* @brief Returns the attribute oflag.
	* @returns The attribute oflag.
	*/
	tcflag_t getoFlag();
	
	/**
	* @brief Sets the attribute cflag.
	* @details This will flush the channel first.
	* @param[in]	cflag	The cflags to set.
	* @warning Flushes the input and output.
	* @returns Returns #ERR_OK if successfully set cflag, or an appropriate error code if a failure occured. 
	*/
	int setcFlag( tcflag_t cflag );
	/**
	* @brief Returns the attribute cflag.
	* @returns The attribute cflag.
	*/
	tcflag_t getcFlag();
#endif
};
