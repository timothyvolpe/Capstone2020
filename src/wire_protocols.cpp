#ifdef __linux__
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#endif
#include <assert.h>
#include <cstring>
#include <sstream>
#include "def.h"
#include "wire_protocols.h"
#include "vehicle.h"
#include "terminal.h"

CWireProtocol::CWireProtocol( std::string portName ) : m_portName( portName )
{
	m_hPortHandle = -1;
	
	m_threadRunning = false;
}
CWireProtocol::~CWireProtocol()
{
}

void CWireProtocol::threadMain()
{
	DebugMessage( "%s THREAD ENTER\n", m_portName.c_str() );
	
	m_threadRunning = true;
	m_threadStarted.notify_all();
	while( m_threadRunning )
		this->threadLoop();
		
	// Need to add a thread heartbeat
	
	DebugMessage( "%s THREAD EXIT\n", m_portName.c_str() );
}

bool CWireProtocol::isRunning() {
	return m_threadRunning;
}

int CWireProtocol::openPort( std::string channelPath, int flags )
{
	assert( m_hPortHandle == -1 );
	assert( !m_threadRunning );
	
	// Open the channel
	m_hPortHandle = ::open( channelPath.c_str(), flags );
	if( m_hPortHandle == -1 )
	{
		if( errno == EACCES )
			return ERR_COMM_ACCESS_DENIED;
		else if( errno == ENOENT || errno == ENOSPC || errno == ENOTDIR || errno == EINVAL )
			return ERR_COMM_INVALID_PATH;
		else
			return ERR_COMM_OPEN_FAILED;
	}
	
	return ERR_OK;
}

void CWireProtocol::startThread()
{
	m_threadError = 0;
	m_thread = std::thread( &CWireProtocol::threadMain, this );
	
	// Wait until thread reports running
	std::mutex m;
	std::unique_lock<std::mutex> lock( m );
	m_threadStarted.wait( lock );
}
void CWireProtocol::stopThread()
{
	// If it was called from the thread itself, just do this
	if( std::this_thread::get_id() == m_thread.get_id() ) {
		m_threadRunning = false;
		return;
	}
	
	this->preStopThread();
	
	// Stop thread
	m_threadRunning = false;
	if( m_thread.joinable() )
		m_thread.join();
		
	// Close channel
	if( m_hPortHandle >= 0 ) {
		::close( m_hPortHandle );
	}
	m_hPortHandle = -1;
}

std::string CWireProtocol::getPortName() {
	return m_portName;
}

const int CWireProtocol::getThreadError() {
	return m_threadError;
}


/////////////
// CI2CBus //
/////////////

CI2CBus::CI2CBus( std::string portName ) : CWireProtocol( portName ) {
}
CI2CBus::~CI2CBus() {
	this->close();
}

void CI2CBus::threadLoop()
{
	assert( this->isOpen() );
	
	if( !this->isRunning() )
		return;
		
	try
	{
		CI2CBus::I2CPacket packet;
		bool potentiallyEmpty = false;
		packet.address = 0;
		
		// Try to write an i2c command
		std::unique_lock<std::mutex> lock( m_writeMutex );
		if( !m_writeBuffer.empty() ) {
			packet = m_writeBuffer.front();
			//Terminal()->printImportant( "Read is %s, and resp len is %d\n", (m_writeBuffer.front().readFromPort ? "True" : "False"), m_writeBuffer.front().respLen );
			m_writeBuffer.pop();
		}
		if( m_writeBuffer.empty() )
			potentiallyEmpty = true;
		lock.unlock();
		
		// If we have a packet to write
		if( packet.address != 0 && packet.payload.size() > 0 )
		{
			// Form the buffer
			std::vector<i2c_msg> messages;
			i2c_rdwr_ioctl_data i2cpayload;
			CI2CBus::I2CPacket responsePacket;
			
			responsePacket.address = packet.address;

			// Write message, always exists
			i2c_msg writeMsg;
			writeMsg.addr = packet.address;
			writeMsg.buf = &packet.payload[0];
			writeMsg.len = packet.payload.size();
			writeMsg.flags = 0;
			messages.push_back( writeMsg );
			
			// Read message, only exists if read is true
			if( packet.readFromPort )
			{
				i2c_msg readMsg;
				readMsg.addr = packet.address;
				readMsg.len = packet.respLen;
				readMsg.flags = I2C_M_RD;
				responsePacket.payload.resize( packet.respLen );
				readMsg.buf = &responsePacket.payload[0];
				messages.push_back( readMsg );
			}
	
			i2cpayload.msgs = &messages[0];
			i2cpayload.nmsgs = messages.size();
			
			// Send i2c message and check if device responds
			if( ioctl( m_hPortHandle, I2C_RDWR, &i2cpayload ) == -1 )
			{
				// No response
				if( errno == EREMOTEIO ) {
					responsePacket.noResponse = true;
				}
				// Some other error
				else {
					DebugMessage( "Thread error: %d (address: 0x%02x, port: %d)\n", errno, (int)packet.address, (int)m_hPortHandle );
					m_threadError = ERR_I2C_WRITE_TO_PORT;
					this->stopThread();
					return;
				}
			}
			// Response be in responsePacket.payload
			else {
				responsePacket.noResponse = false;
			}
			
			// Add to response buffer
			std::unique_lock<std::mutex> lock2( m_readMutex );
			m_responseBuffer.push_back( responsePacket );
			lock2.unlock();
			
			// Signal write buffer MIGHT be empty (something could have been added
			// between now and the lock above).
			if( potentiallyEmpty )
				m_waitingToSend.notify_all();
		}
		else
			m_waitingToSend.notify_all();
	}
	catch(const std::exception &e) {
		Terminal()->printImportant( "Communication thread encountered an exception and had to stop: %s\n", e.what() );
		this->stopThread();
	}
	catch(...) {
		Terminal()->printImportant( "Communication thread encountered an unknown exception and had to stop.\n" );
		this->stopThread();
	}
}
void CI2CBus::preStopThread() {
	
}

int CI2CBus::open( std::string busPath )
{
	int errCode;
	tcflag_t cflag;
	unsigned long i2cfuncs;
	
	if( this->isOpen() )
		return ERR_I2C_ALREADY_OPEN;
		
	// Open port
	if( (errCode = this->openPort( busPath, O_RDWR | O_NOCTTY | O_NONBLOCK )) != ERR_OK )
		return errCode;
		
	// Check for i2c support on channel
	if( ioctl( m_hPortHandle, I2C_FUNCS, &i2cfuncs ) == -1 )
		return ERR_I2C_NO_SUPPORT;
    if( !(i2cfuncs & I2C_FUNC_I2C) )
		return ERR_I2C_NO_SUPPORT;
	
	this->startThread();

	return ERR_OK;
}

void CI2CBus::close() {
	this->stopThread();
}

bool CI2CBus::write_i2c( uint8_t address, std::vector<unsigned char> payload, bool read, size_t respLen )
{
	// If read is true, respLen must be > 0
	assert( (!read) ^ (respLen > 0) );
	
	if( !this->isOpen() || !this->isRunning() ) 
		return false;

	// Form the packet
	CI2CBus::I2CPacket packet;
	packet.address = address;
	packet.payload = payload;
	packet.readFromPort = read;
	packet.respLen = respLen;
	
	std::lock_guard<std::mutex> lock( m_writeMutex );
	m_writeBuffer.push( packet );

	return true;
}
bool CI2CBus::write_i2c_byte( uint8_t address, unsigned char data, bool read, size_t respLen ) {
	std::vector<unsigned char> buffer(1);
	buffer[0] = data;
	return this->write_i2c( address, buffer, read, respLen );
}

bool CI2CBus::write( std::vector<unsigned char> buffer )
{
	assert( false ); // Don't use this for now
	
	return true;
}

bool CI2CBus::read_i2c( uint8_t address, int timeoutMS, std::vector<unsigned char> &response )
{	
	CI2CBus::I2CPacket packet;
	bool found = false;
	
	std::chrono::steady_clock::time_point starttime = std::chrono::steady_clock::now();
	
	// Look for address in vector. First found will a be oldest
	// Try until timeout has expired
	while( (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - starttime).count() < timeoutMS) || timeoutMS == 0 ) 
	{
		std::unique_lock<std::mutex> lock( m_readMutex );
		for( auto it = m_responseBuffer.begin(); it != m_responseBuffer.end(); it++ )
		{
			if( (*it).address == address ) {
				packet = (*it);
				m_responseBuffer.erase( it );
				found = true;
				break;
			}
		}
		lock.unlock();

		// If we found a response
		if( found )
		{
			if( packet.noResponse ) {
				return false;
			}
			response = packet.payload;
			return true;
		}
		
		if( timeoutMS == 0 )
			break;
	}
	// No response found apparently
	return false;
}

std::vector<unsigned char> CI2CBus::read( size_t count )
{	
	assert( false ); // Don't use this for now
	
	return std::vector<unsigned char>();
}

int CI2CBus::flush()
{
	// Flush write and response buffers
	std::lock_guard<std::mutex> lock1( m_readMutex );
	std::lock_guard<std::mutex> lock2( m_writeMutex );
	m_responseBuffer.clear();
	m_writeBuffer = std::queue<I2CPacket>();
	
	return ERR_OK;
}

bool CI2CBus::flushWriteBlocking( int timeoutMS )
{
	std::mutex m;
	std::unique_lock<std::mutex> lock(m);
	
	// Wait for it to be set to false by the thread, or timeout
	return (m_waitingToSend.wait_until(lock, std::chrono::steady_clock::now() + std::chrono::milliseconds( timeoutMS ) ) == std::cv_status::no_timeout );
}

bool CI2CBus::dataAvailable() {
	return true;
}

bool CI2CBus::isOpen() {
	return (m_hPortHandle >= 0);
}

bool CI2CBus::checkAddress( uint8_t addr )
{
	this->flush();
	// Send dummy command and block until it is transmitted
	this->write_i2c_byte( addr, 0, false );
	if( !this->flushWriteBlocking( 1000 ) ) {
		Terminal()->printImportant( "Timeout expired while checking i2c address\n" );
		return false;
	}
	// Check for response
	std::vector<unsigned char> response;
	if( !this->read_i2c( addr, 0, response ) )
		return false;
	return true;
}

bool CI2CBus::pollAllAddress()
{
	std::vector<uint8_t> respondingAddresses;
	std::stringstream addressListStr;
	
	Terminal()->print( "Polling all 7-bit I2C addresses...\n" );
	
	this->flush();
	
	// Check every possible i2c sensor address
	for( uint8_t addr = 0x01; addr <= 0x7F; addr++ )
	{
		// Write a dummy command to the port
		if( !this->write_i2c_byte( addr, 0, false ) ) {
			Terminal()->printImportant( "There was an internal i2c error\n" );
			return false;
		}
	}
	
	if( !this->flushWriteBlocking( 1000 ) ) {
		Terminal()->printImportant( "Timed out waiting for I2C port to flush!\n" );
		return false;
	}
	
	// Check for responses
	for( uint8_t addr = 0x01; addr <= 0x7F; addr++ )
	{
		std::vector<unsigned char> response;
		if( this->read_i2c( addr, 0, response ) )
			respondingAddresses.push_back( addr );
	}
	
	Terminal()->print( "Finished polling 7-bit I2C addresses!\n" );
	
	// Print responding addresses
	if( !respondingAddresses.empty() )
	{
		for( auto it = respondingAddresses.begin(); it != respondingAddresses.end(); it++ ) 
			addressListStr << "0x" << std::hex << (int)(*it) << " ";
		Terminal()->printImportant( "The following addresses responded: %s\n", addressListStr.str().c_str() );
	}
	else
		Terminal()->printImportant( "No devices responded.\n" );
	
	this->flush();
	
	return true;
}

//////////////////
// CUARTChannel //
//////////////////

CUARTChannel::CUARTChannel( std::string portName ) : CWireProtocol( portName )
{
	std::memset( &m_portOptions, 0, sizeof( m_portOptions ) );
}
CUARTChannel::~CUARTChannel() {
	this->close();
}

void CUARTChannel::threadLoop()
{
	int events;
	pollfd fileDesc;
	bool bytesAtPort, writeAvail;
	
	assert( this->isOpen() );
	
	if( !this->isRunning() )
		return;
	
	
	fileDesc.fd = m_hPortHandle;
	fileDesc.events = POLLIN | POLLOUT | POLLERR;
	fileDesc.revents = 0;
	
	bytesAtPort = false;
	writeAvail = false;
	
	try
	{	
		// Check for serial events
		events = poll( &fileDesc, 1, 0 );
		if( events == -1 )
		{
			if( errno == EFAULT || errno == EINVAL || errno == ENOMEM )
			{
				// Fatal error
				m_threadError = ERR_UART_POLL;
				this->stopThread();
				return;
			}
		}
		if( events > 0 )
		{
			// Read some bytes
			if( fileDesc.revents & POLLIN )
				bytesAtPort = true;
			// Write some bytes
			if( fileDesc.revents & POLLOUT )
				writeAvail = true;
			// Error
			if( fileDesc.revents & POLLERR ) {
				m_threadError = ERR_UART_POLL;
				this->stopThread();
				return;
			}
		}
			
		if( bytesAtPort )
		{
			std::unique_lock<std::mutex> lock( m_flushMutex );
			
			// Check the number of bytes at the port
			unsigned long byteCount;
			if( ioctl( m_hPortHandle, FIONREAD, &byteCount ) == -1 ) {
				m_threadError = ERR_UART_READ;
				this->stopThread();
				return;
			}
		
			if( byteCount <= 0 )
				bytesAtPort = false;
			else
			{
				std::vector<unsigned char> buffer;
				ssize_t actualBytesRead;
				buffer.resize( byteCount );
				actualBytesRead = ::read( m_hPortHandle, &buffer[0], byteCount );
				if( actualBytesRead == -1 )
				{
					if( errno != EINTR ) {
						m_threadError = ERR_UART_READ;
						this->stopThread();
						return;
					}
				}
				else if( actualBytesRead > 0 ) {
					buffer.resize( actualBytesRead );
					for( auto it = buffer.begin(); it != buffer.end(); it++ )
						m_readBuffer.push( (*it) );
				}
				if( byteCount - actualBytesRead <= 0 )
					bytesAtPort = false;
			}
			
			lock.unlock();
		}
		if( writeAvail )
		{
			std::vector<unsigned char> buffer; 
			
			// Retrieve one entry from the buffer
			std::unique_lock<std::mutex> lock2( m_writeMutex );
			if( m_writeBuffer.size() > 0 ) {
				buffer = m_writeBuffer.front();
				m_writeBuffer.pop();
			}
			lock2.unlock();
			
			if( !buffer.empty() )
			{
				size_t bufferSize = buffer.size();
				ssize_t bytesWritten = 0;
				int attempts = 0;
				
				// Try to write all the bytes
				std::unique_lock<std::mutex> lock( m_flushMutex );
				do
				{
					ssize_t curBytesWritten;
					curBytesWritten = ::write( m_hPortHandle, &buffer[0], buffer.size() );
					if( curBytesWritten <= 0 )
					{	
						attempts++;
						if( attempts >= UART_WRITE_ATTEMPS )
							break;
						if( errno != EINTR | errno != EAGAIN ) {
							m_threadError = ERR_UART_READ;
							this->stopThread();
							return;
						}
					}
					else {
						
						bytesWritten += curBytesWritten;
						if( bytesWritten < bufferSize )
							std::this_thread::sleep_for( std::chrono::milliseconds(UART_WRITE_WAIT_MS) ); 
					}
				} while( bytesWritten < bufferSize );
				lock.unlock();
				
				writeAvail = false;
			}
		}
	}
	catch(const std::exception &e) {
		Terminal()->printImportant( "Communication thread encountered an exception and had to stop: %s\n", e.what() );
		this->stopThread();
	}
	catch(...) {
		Terminal()->printImportant( "Communication thread encountered an unknown exception and had to stop.\n" );
		this->stopThread();
	}
}
void CUARTChannel::preStopThread() {
	
}

int CUARTChannel::open( std::string channelPath, bool enableReceiver, bool twoStopBits, bool parity, bool rtscts )
{
	int errCode;
	tcflag_t cflag;
	
	if( this->isOpen() )
		return ERR_UART_ALREADY_OPEN;
		
	// Open port
	if( (errCode = this->openPort( channelPath, O_RDWR | O_NOCTTY | O_NONBLOCK )) != ERR_OK )
		return errCode;
	
	// Set port options
	cflag = CS8 | CLOCAL;	// message length 8 bits
	if( enableReceiver )
		cflag |= CREAD;	
	else
		cflag &= ~CREAD; 
	if( twoStopBits )
		cflag |= CSTOPB;
	else
		cflag &= ~CSTOPB;
	if( parity )
		cflag |= (PARENB | PARODD);
	else
		cflag &= ~(PARENB | PARODD);
	if( rtscts )
		cflag |= CRTSCTS;
	else
		cflag &= ~CRTSCTS;
	
	if( (errCode = this->setcFlag( cflag )) != ERR_OK )
		return errCode;
	
	// Set read timeout and non-blocking
	this->setReadTimeout( 0, 5 * 10 ); // 5 seconds (50 deciseconds)
	
	this->startThread();

	return ERR_OK;
}
void CUARTChannel::close() {
	this->stopThread();
}

bool CUARTChannel::write( std::vector<unsigned char> buffer )
{
	if( !this->isOpen() || !this->isRunning() ) 
		return false;

	std::lock_guard<std::mutex> lock( m_writeMutex );
	m_writeBuffer.push( buffer );

	return true;
}
std::vector<unsigned char> CUARTChannel::read( size_t count )
{
	std::vector<unsigned char> buffer;

	std::lock_guard<std::mutex> lock( m_readMutex );
	
	if( count > m_readBuffer.size() || count == 0 )
		count = m_readBuffer.size();
	buffer.reserve( count );
	for( unsigned int i = 0; i < count; i++ ) {
		buffer.push_back( m_readBuffer.front() );
		m_readBuffer.pop();
	}
	
	return buffer;
}

int CUARTChannel::flush()
{
	std::unique_lock<std::mutex> lockWrite( m_writeMutex );
	m_writeBuffer = WriteQueue();
	lockWrite.unlock();
	std::unique_lock<std::mutex> lockRead( m_readMutex );
	m_readBuffer = std::queue<unsigned char>();
	lockRead.unlock();

	std::lock_guard<std::mutex> lock( m_flushMutex );
	if( tcflush( m_hPortHandle, TCIOFLUSH ) == -1 )
		return ERR_UART_FLUSH_CHANNEL;
	return ERR_OK;
}

bool CUARTChannel::dataAvailable() {
	return !m_readBuffer.empty();
}

bool CUARTChannel::isOpen() {
	return (m_hPortHandle >= 0);
}

bool CUARTChannel::flushAttributes()
{
	std::unique_lock<std::mutex> lock( m_flushMutex );
	if( tcsetattr( m_hPortHandle, TCSAFLUSH, &m_portOptions ) == -1 )
		return false;
	lock.unlock();
	
	if( this->flush() != ERR_OK )
		return false;

	return true;
}

int CUARTChannel::setBaudRate( speed_t baud )
{
	if( cfsetispeed( &m_portOptions, baud ) == -1 ) {
		return ERR_UART_INVALID_BAUD;
	}
	if( cfsetospeed( &m_portOptions, baud ) == -1 ) {
		return ERR_UART_INVALID_BAUD;
	}
	if( !this->flushAttributes() )
		return ERR_UART_SET_ATTRIB;
	
	return ERR_OK;
}

int CUARTChannel::setReadTimeout( cc_t bytesNeeded, cc_t deciseconds )
{
	m_portOptions.c_cc[VMIN] = bytesNeeded;
	m_portOptions.c_cc[VTIME] = deciseconds;
	if( !this->flushAttributes() )
		return ERR_UART_SET_READ_TIMEOUT;
	return ERR_OK;
}

int CUARTChannel::setiFlag( tcflag_t iflag ) {
	m_portOptions.c_iflag = iflag;
	if( !this->flushAttributes() )
		return ERR_UART_SET_IFLAG;
	return ERR_OK;
}
tcflag_t CUARTChannel::getiFlag() {
	return m_portOptions.c_iflag;
}

int CUARTChannel::setoFlag( tcflag_t oflag ) {
	m_portOptions.c_oflag = oflag;
	if( !this->flushAttributes() )
		return ERR_UART_SET_OFLAG;
	return ERR_OK;
}
tcflag_t CUARTChannel::getoFlag() {
	return m_portOptions.c_oflag;
}

int CUARTChannel::setcFlag( tcflag_t cflag ) {
	m_portOptions.c_cflag = cflag;
	if( !this->flushAttributes() )
		return ERR_UART_SET_CFLAG;
	return ERR_OK;
}
tcflag_t CUARTChannel::getcFlag() {
	return m_portOptions.c_cflag;
}
