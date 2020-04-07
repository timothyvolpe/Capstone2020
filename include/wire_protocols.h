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

/** The number of MS to wait between write calls when attempting to write a block of data*/
#define UART_WRITE_WAIT_MS 2
/** The number of consecutive write attempts to make before failing out. */
#define UART_WRITE_ATTEMPS 10 

/**
* @brief The base class for wire protocols
* @details This class contains all the shared operations between the classes that send
* binary data over wire from the Pi. These classes all use a file handle to write data.
* 
* @author Timothy Volpe
* @date 3/15/2020
*/
class CWireProtocol
{
private:
	std::string m_portName;
	
	std::thread m_thread;
	std::atomic<bool> m_threadRunning;
	
	void threadMain();
protected:
	int m_hPortHandle;
	
	std::atomic<int> m_threadError;
	
	std::mutex m_flushMutex;
	std::mutex m_writeMutex;
	std::mutex m_readMutex;
	
	/**
	* @brief Start the comm thread
	*/
	void startThread();
	
	/**
	* @brief Attempt to stop the comm thread
	*/
	void stopThread();
	
	/**
	* @brief Open the data port.
	* @details This opens the data port but does not start the thread.
	* @param[in]	channelPath		The path to the channel file
	* @param[in]	flags			The flags to open the channel with, see linux open command
	* @returns Returns #ERR_OK if successful, or appropriate erro code if a failure occured.
	*/
	int openPort( std::string channelPath, int flags );

	virtual void threadLoop() = 0;
	
	/**
	* @brief Called before a thread is stopped by the main thread.
	* @details If the thread is stopped by itself, this will not be called.
	*/
	virtual void preStopThread() = 0;
public:
	CWireProtocol( std::string portName );
	~CWireProtocol();

	/**
	* @brief Write to the UART channel.
	* @details This will add buffer to the write buffer, to be sent as soon as the channel is avaliable.
	* @param[in]	buffer	Data to be added to the write buffer.
	* @returns Returns true if successfully added to queue, false if channel was not open.
	*/
	virtual bool write( std::vector<unsigned char> buffer ) = 0;
	/**
	* @brief Reads a certain number of bytes from the read buffer.
	* @details This will read count number of bytes from the read buffer and return them. If there are not count
	*	bytes in the buffer, the entire contents of the read buffer will be returned. The read buffer is filled continuously
	*	as data is read from the port. This will pop the bytes from the buffer, so the cannot be read twice.
	* @param[in]	count	The maximum number of bytes to be read. Pass 0 to read all bytes.
	* @returns A buffer containing up to count number of bytes, however an empty buffer may be returned if no bytes were available.
	*/
	virtual std::vector<unsigned char> read( size_t count ) = 0;
	
	/**
	* @brief Flushes the input and output, clears buffers.
	* @returns Returns #ERR_OK if successfully flushed the channel, or an appropriate error code if a failure occured.
	*/
	virtual int flush() = 0;
	
	/**
	* @brief Checks if there is data available in the read buffer.
	* @returns True if there is data, false if the buffer is empty.
	*/
	virtual bool dataAvailable() = 0;
	
	/**
	* @brief Returns true if a channel is open.
	* @details If the channel is open, but the comm thread is not running, this will still return true.
	* @returns True if the channel is open.
	*/
	virtual bool isOpen() = 0;
	
	/**
	* @brief Returns true if the comm thread is still running
	* @returns True if comm thread is running.
	*/
	bool isRunning();
	
	/**
	* @brief Gets the arbitrary port name for debug purposes
	* @returns The name of the port
	*/
	std::string getPortName();
	
	/**
	* @brief Gets current thread error code
	* @returns Thread error code, which will be #ERR_OK if no error occured.
	*/
	const int getThreadError();
};

/**
* @brief Handles an I2C bus.
* @details This class can handle a single I2C bus with multiple devices connected to it.
*
* @author Timothy Volpe
* @date 2/27/2020
*/
class CI2CBus : public CWireProtocol
{
private:
	struct I2CPacket
	{
		unsigned char address;
		std::vector<unsigned char> payload;
	};
	
	std::queue<I2CPacket> m_writeBuffer;
protected:
	void threadLoop();
	void preStopThread();
public:
	/* Default constructor */
	CI2CBus( std::string portName );
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
	
	/**
	* @brief Writes an i2c packet to the bus.
	* @details This method forms the packet automatically and sends it to the comm thread
	* to be transmitted.
	* @param[in]	address		The slave address to send the payload to.
	* @param[in]	writeBit	If true, the write bit will be set to one. If false, it will be forced to 0.
	* @param[in]	payload		The data payload to deliver to the slave at the specified address.
	* @returns Returns true if successfully posted the message to the comm thread, false if otherwise.
	*/
	bool write_i2c( unsigned char address, bool writeBit, std::vector<unsigned char> payload );
	
	/**
	* @brief Write a single byte to an i2c device
	* @details This writes one device to an i2c device. Posts it to the comm thread for non-blocking write.
	* @param[in]	address		The address to write to. The R/W bit will be set.
	* @param[in]	data		The byte to write to the device.
	* @returns Returns ture if successfully posted to the comm thread, false if otherwise.
	*/
	bool write_i2c_byte( unsigned char address, unsigned char data );
	
	bool write( std::vector<unsigned char> buffer );
	std::vector<unsigned char> read( size_t count );

	int flush();
	
	bool dataAvailable();
	
	bool isOpen();
};

/**
* @brief Handles a single UART channel.
* @details This can handle one device connected to the UART channel.
* 	The device will poll for UART events in a seperate thread, so that it is non-blocking.
*
* @author Timothy Volpe
* @date 2/27/2020
*/
class CUARTChannel : public CWireProtocol
{
private:
	typedef std::queue<std::vector<unsigned char>> WriteQueue;

	termios m_portOptions;
	
	WriteQueue m_writeBuffer;
	std::queue<unsigned char> m_readBuffer;
	
	/**
	* @brief Takes the attribute changes stored in this class and flushes them to the port.
	* @warning This flushes the read and write buffers of the serial port.
	*/
	bool flushAttributes();
protected:
	void threadLoop();
	void preStopThread();
public:
	/* Default constructor */
	CUARTChannel( std::string portName );
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
	* @brief Write to the UART channel.
	* @details This will add buffer to the write buffer, to be sent as soon as the channel is avaliable.
	* @param[in]	buffer	Data to be added to the write buffer.
	* @returns Returns true if successfully added to queue, false if channel was not open.
	*/
	bool write( std::vector<unsigned char> buffer );
	std::vector<unsigned char> read( size_t count );

	int flush();
	
	bool dataAvailable();
	
	bool isOpen();
	
	/**
	* @brief Sets the baud rate of the port.
	* @details Sets the speed to the input and output lines. This will flush the channel first.
	* @param[in]	baud	The baud rate constant
	* @warning Flushes the input and output.
	* @returns Returns #ERR_OK if successfully set baud rate, or an appropriate error code if a failure occured.
	*/
	int setBaudRate( speed_t baud );
	
	/**
	* @brief Sets the read timeout of the port.
	* @details This will flush the channel first.
	* @param[in]	bytesNeeded		The number of bytes to block until, set to 0 for nonblocking.
	* @param[in]	deciseconds		The read timeout in deciseconds.
	* @warning Flushes the input and output.
	* @returns Returns #ERR_OK if successfully set read timeout, or an appropriate error code if a failure occured.
	*/
	int setReadTimeout( cc_t bytesNeeded, cc_t deciseconds ); 
	
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
};
