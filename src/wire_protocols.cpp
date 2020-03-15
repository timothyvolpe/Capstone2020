#ifdef __linux__
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <sys/ioctl.h>
#endif
#include <assert.h>
#include <cstring>
#include "def.h"
#include "wire_protocols.h"
#include "vehicle.h"
#include "terminal.h"

CWireProtocol::CWireProtocol( std::string portName ) : m_portName( portName )
{
	m_hPortHandle = -1;
	
	m_threadRunning = false;
	
	std::memset( &m_portOptions, 0, sizeof( m_portOptions ) );
}
CWireProtocol::~CWireProtocol()
{
}

void CWireProtocol::threadMain()
{
	int events;
	pollfd fileDesc;
	bool bytesAtPort, writeAvail;
	
	assert( m_hPortHandle != -1 );
	assert( !m_threadRunning );
	
	DebugMessage( "COMM THREAD ENTER\n" );
	m_threadRunning = true;
	
	fileDesc.fd = m_hPortHandle;
	fileDesc.events = POLLIN | POLLOUT | POLLERR;
	fileDesc.revents = 0;
	
	bytesAtPort = false;
	writeAvail = false;
	
	try
	{
		while( m_threadRunning )
		{		
			// Check for serial events
			events = poll( &fileDesc, 1, 0 );
			if( events == -1 )
			{
				if( errno == EFAULT || errno == EINVAL || errno == ENOMEM )
				{
					// Fatal error
					m_threadError = ERR_COMM_POLL;
					m_threadRunning = false;
					break;
				}
			}
			if( events > 0 )
			{
				// Read some bytes
				if( fileDesc.revents & POLLIN ) {
					bytesAtPort = true;
				}
				// Write some bytes
				if( fileDesc.revents & POLLOUT ) {
					writeAvail = true;
				}
				// Error
				if( fileDesc.revents & POLLERR ) {
					m_threadError = ERR_COMM_POLL;
					m_threadRunning = false;
					break;
				}
			}
			else
				continue; // Can't read or write, so do nothing
				
			if( bytesAtPort )
			{
				std::unique_lock<std::mutex> lock( m_flushMutex );
				
				// Check the number of bytes at the port
				unsigned long byteCount;
				if( ioctl( m_hPortHandle, FIONREAD, &byteCount ) == -1 ) {
					m_threadError = ERR_COMM_READ;
					m_threadRunning = false;
					break;
				}
			
				if( byteCount <= 0 )
					bytesAtPort = false;
				else
				{
					std::vector<unsigned char> buffer;
					ssize_t actualBytesRead;
					buffer.resize( byteCount );
					actualBytesRead = ::read( m_hPortHandle, &buffer[0], byteCount );
					if( actualBytesRead == -1 ) {
						if( errno != EINTR ) {
							m_threadError = ERR_COMM_READ;
							m_threadRunning = false;
							break;
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
								m_threadError = ERR_COMM_READ;
								m_threadRunning = false;
								break;
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
	}
	catch(const std::exception &e) {
		Terminal()->printImportant( "Communication thread encountered an exception and had to stop: %s\n", e.what() );
	}
	catch(...) {
		Terminal()->printImportant( "Communication thread encountered an unknown exception and had to stop.\n" );
	}
	
	DebugMessage( "COMM THREAD EXIT\n" );
}

bool CWireProtocol::flushAttributes()
{
	std::unique_lock<std::mutex> lock( m_flushMutex );
	if( tcsetattr( m_hPortHandle, TCSAFLUSH, &m_portOptions ) == -1 )
		return false;
	lock.unlock();
	
	if( this->flush() != ERR_OK )
		return false;

	return true;
}

int CWireProtocol::setBaudRate( speed_t baud )
{
	if( cfsetispeed( &m_portOptions, baud ) == -1 ) {
		return ERR_COMM_INVALID_BAUD;
	}
	if( cfsetospeed( &m_portOptions, baud ) == -1 ) {
		return ERR_COMM_INVALID_BAUD;
	}
	if( !this->flushAttributes() )
		return ERR_COMM_SET_ATTRIB;
	
	return ERR_OK;
}

int CWireProtocol::setReadTimeout( cc_t bytesNeeded, cc_t deciseconds )
{
	m_portOptions.c_cc[VMIN] = bytesNeeded;
	m_portOptions.c_cc[VTIME] = deciseconds;
	if( !this->flushAttributes() )
		return ERR_COMM_SET_READ_TIMEOUT;
	return ERR_OK;
}

int CWireProtocol::setiFlag( tcflag_t iflag ) {
	m_portOptions.c_iflag = iflag;
	if( !this->flushAttributes() )
		return ERR_COMM_SET_IFLAG;
	return ERR_OK;
}
tcflag_t CWireProtocol::getiFlag() {
	return m_portOptions.c_iflag;
}

int CWireProtocol::setoFlag( tcflag_t oflag ) {
	m_portOptions.c_oflag = oflag;
	if( !this->flushAttributes() )
		return ERR_COMM_SET_OFLAG;
	return ERR_OK;
}
tcflag_t CWireProtocol::getoFlag() {
	return m_portOptions.c_oflag;
}

int CWireProtocol::setcFlag( tcflag_t cflag ) {
	m_portOptions.c_cflag = cflag;
	if( !this->flushAttributes() )
		return ERR_COMM_SET_CFLAG;
	return ERR_OK;
}
tcflag_t CWireProtocol::getcFlag() {
	return m_portOptions.c_cflag;
}

bool CWireProtocol::isOpen() {
	return (m_hPortHandle >= 0);
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
}
void CWireProtocol::stopThread()
{
	// Stop thread
	if( m_threadRunning ) {
		m_threadRunning = false;
		m_thread.join();
	}
	// Close channel
	if( m_hPortHandle >= 0 ) {
		::close( m_hPortHandle );
	}
	m_hPortHandle = -1;
}

bool CWireProtocol::write( std::vector<unsigned char> buffer )
{
	if( !this->isOpen() || !this->isRunning() ) 
		return false;

	std::lock_guard<std::mutex> lock( m_writeMutex );
	m_writeBuffer.push( buffer );

	return true;
}
std::vector<unsigned char> CWireProtocol::read( size_t count )
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

int CWireProtocol::flush()
{
	std::unique_lock<std::mutex> lockWrite( m_writeMutex );
	m_writeBuffer = WriteQueue();
	lockWrite.unlock();
	std::unique_lock<std::mutex> lockRead( m_readMutex );
	m_readBuffer = std::queue<unsigned char>();
	lockRead.unlock();

	std::lock_guard<std::mutex> lock( m_flushMutex );
	if( tcflush( m_hPortHandle, TCIOFLUSH ) == -1 )
		return ERR_COMM_FLUSH_CHANNEL;
	return ERR_OK;
}

bool CWireProtocol::dataAvailable() {
	return !m_readBuffer.empty();
}

std::string CWireProtocol::getPortName() {
	return m_portName;
}

//////////////////
/////////////////////
//////////////////////

CI2CBus::CI2CBus() {
	m_hBusHandle = -1;
}
CI2CBus::~CI2CBus() {
	this->close();
}

int CI2CBus::open( std::string busPath )
{
#ifdef __linux__
	// Open the bus
	m_hBusHandle = ::open( busPath.c_str(), O_RDWR );
	if( m_hBusHandle == -1 )
	{
		if( errno == EACCES )
			return ERR_I2C_ACCESS_DENIED;
		else if( errno == ENOENT || errno == ENOSPC || errno == ENOTDIR || errno == EINVAL )
			return ERR_I2C_INVALID_PATH;
		else
			return ERR_I2C_OPEN_FAILED;
	}
#endif

	return ERR_OK;
}

void CI2CBus::close()
{
#ifdef __linux__
	if( m_hBusHandle >= 0 ) {
		::close( m_hBusHandle );
	}
	m_hBusHandle = -1;
#endif
}

CUARTChannel::CUARTChannel( std::string portName ) : CWireProtocol( portName ) {
	
}
CUARTChannel::~CUARTChannel() {
	this->close();
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
