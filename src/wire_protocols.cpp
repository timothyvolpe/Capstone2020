#ifdef __linux__
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <sys/ioctl.h>
#endif
#include <assert.h>
#include "def.h"
#include "wire_protocols.h"
#include "vehicle.h"
#include "terminal.h"

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

CUARTChannel::CUARTChannel()
{
	m_hChannelHandle = -1;
	m_channelError = 0;
	
	m_threadRunning = 0;
	m_threadHeartbeat = 0;
	m_lastThreadHeartbeat = 0;
}
CUARTChannel::~CUARTChannel() {
	this->close();
}

void CUARTChannel::uartThreadMain()
{
	int events;
	pollfd fileDesc;
	bool bytesAtPort, writeAvail;
	
	DebugMessage( "UART THREAD ENTER\n" );
	
	assert( m_hChannelHandle );
	
	fileDesc.fd = m_hChannelHandle;
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
					m_channelError = ERR_UART_POLL;
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
					m_channelError = ERR_UART_POLL;
					m_threadRunning = false;
					break;
				}
			}
			else
				continue; // Can't read or write, so do nothing
				
			if( bytesAtPort )
			{
				std::unique_lock<std::mutex> lock( m_flushLock );
				
				// Check the number of bytes at the port
				unsigned long byteCount;
				if( ioctl( m_hChannelHandle, FIONREAD, &byteCount ) == -1 ) {
					m_channelError = ERR_UART_READ;
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
					actualBytesRead = ::read( m_hChannelHandle, &buffer[0], byteCount );
					if( actualBytesRead == -1 ) {
						if( errno != EINTR ) {
							m_channelError = ERR_UART_READ;
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
					std::unique_lock<std::mutex> lock( m_flushLock );
					do
					{
						ssize_t curBytesWritten;
						curBytesWritten = ::write( m_hChannelHandle, &buffer[0], buffer.size() );
						if( curBytesWritten <= 0 )
						{	
							attempts++;
							if( attempts >= UART_WRITE_ATTEMPS )
								break;
							if( errno != EINTR | errno != EAGAIN ) {
								m_channelError = ERR_UART_READ;
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
		Terminal()->printImportant( "UART thread encountered an exception and had to stop: %s\n", e.what() );
	}
	catch(...) {
		Terminal()->printImportant( "UART thread encountered an unknown exception and had to stop.\n" );
	}
	
	DebugMessage( "UART THREAD EXIT\n" );
}

int CUARTChannel::open( std::string channelPath, bool enableReceiver, bool twoStopBits, bool parity, bool rtscts )
{
	if( this->isOpen() )
		return ERR_UART_ALREADY_OPEN;
	
#ifdef __linux__
	// Open the channel
	m_hChannelHandle = ::open( channelPath.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK );
	if( m_hChannelHandle == -1 )
	{
		if( errno == EACCES )
			return ERR_UART_ACCESS_DENIED;
		else if( errno == ENOENT || errno == ENOSPC || errno == ENOTDIR || errno == EINVAL )
			return ERR_UART_INVALID_PATH;
		else
			return ERR_UART_OPEN_FAILED;
	}
	
	// Get current options
	//::tcgetattr( m_hChannelHandle, &m_uartOptions );
	m_uartOptions.c_cflag = CS8 | CLOCAL;	// message length 8 bits
	if( enableReceiver )
		m_uartOptions.c_cflag |= CREAD;	
	else
		m_uartOptions.c_cflag &= ~CREAD; 
	if( twoStopBits )
		m_uartOptions.c_cflag |= CSTOPB;
	else
		m_uartOptions.c_cflag &= ~CSTOPB;
	if( parity )
		m_uartOptions.c_cflag |= (PARENB | PARODD);
	else
		m_uartOptions.c_cflag &= ~(PARENB | PARODD);
	if( rtscts )
		m_uartOptions.c_cflag |= CRTSCTS;
	else
		m_uartOptions.c_cflag &= ~CRTSCTS;
		
	m_uartOptions.c_cc[VMIN] = 0;
	m_uartOptions.c_cc[VTIME] = 5 * 10;	// 5 seconds (50 deciseconds)
		
	if( !this->setAttributes() )
		return ERR_UART_SET_ATTRIB;
#endif

	// Start the thread
	m_threadRunning = true;
	m_channelError = 0;
	m_threadHeartbeat = 0;
	m_lastThreadHeartbeat = 0;
	m_uartThread = std::thread( &CUARTChannel::uartThreadMain, this );

	return ERR_OK;
}
void CUARTChannel::close()
{
#ifdef __linux__
	// Stop thread
	if( m_threadRunning ) {
		m_threadRunning = false;
		m_uartThread.join();
	}
	// Close channel
	if( m_hChannelHandle >= 0 ) {
		::close( m_hChannelHandle );
	}
	m_hChannelHandle = -1;
#endif
}

bool CUARTChannel::isOpen() {
	return (m_hChannelHandle >= 0);
}

bool CUARTChannel::write( std::vector<unsigned char> buffer )
{
	if( !this->isOpen() ) 
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

bool CUARTChannel::dataAvailable() {
	return !m_readBuffer.empty();
}

#ifdef __linux__
int CUARTChannel::flush()
{
	std::unique_lock<std::mutex> lockWrite( m_writeMutex );
	m_writeBuffer = WriteQueue();
	lockWrite.unlock();
	std::unique_lock<std::mutex> lockRead( m_readMutex );
	m_readBuffer = std::queue<unsigned char>();
	lockRead.unlock();
	

	std::lock_guard<std::mutex> lock( m_flushLock );
	
	if( tcflush( m_hChannelHandle, TCIOFLUSH ) == -1 )
		return ERR_UART_FLUSH_CHANNEL;
	return ERR_OK;
}

bool CUARTChannel::setAttributes()
{
	std::unique_lock<std::mutex> lock( m_flushLock );
	if( tcsetattr( m_hChannelHandle, TCSAFLUSH, &m_uartOptions ) == -1 )
		return false;
	lock.unlock();
	if( this->flush() != ERR_OK )
		return false;

	return true;
}

int CUARTChannel::setBaudRate( speed_t baud )
{
	if( cfsetispeed( &m_uartOptions, baud ) == -1 ) {
		return ERR_UART_INVALID_BAUD;
	}
	if( cfsetospeed( &m_uartOptions, baud ) == -1 ) {
		return ERR_UART_INVALID_BAUD;
	}
	if( !this->setAttributes() )
		return ERR_UART_SET_ATTRIB;
	
	return ERR_OK;
}

int CUARTChannel::setReadTimeout( cc_t deciseconds )
{
	m_uartOptions.c_cc[VTIME] = deciseconds;
	if( !this->setAttributes() )
		return ERR_UART_SET_READ_TIMEOUT;
	return ERR_OK;
}

int CUARTChannel::setiFlag( tcflag_t iflag ) {
	m_uartOptions.c_iflag = iflag;
	if( !this->setAttributes() )
		return ERR_UART_SET_IFLAG;
	return ERR_OK;
}
tcflag_t CUARTChannel::getiFlag() {
	return m_uartOptions.c_iflag;
}

int CUARTChannel::setoFlag( tcflag_t oflag ) {
	m_uartOptions.c_oflag = oflag;
	if( !this->setAttributes() )
		return ERR_UART_SET_OFLAG;
	return ERR_OK;
}
tcflag_t CUARTChannel::getoFlag() {
	return m_uartOptions.c_oflag;
}

int CUARTChannel::setcFlag( tcflag_t oflag ) {
	m_uartOptions.c_cflag = oflag;
	if( !this->setAttributes() )
		return ERR_UART_SET_OFLAG;
	return ERR_OK;
}
tcflag_t CUARTChannel::getcFlag() {
	return m_uartOptions.c_cflag;
}
#endif
