#ifdef __linux__
#include <fcntl.h>
#include <unistd.h>
#endif
#include "def.h"
#include "wire_protocols.h"

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

CUARTChannel::CUARTChannel() {
	m_hChannelHandle = -1;
}
CUARTChannel::~CUARTChannel() {
	this->close();
}

void CUARTChannel::uartThreadMain()
{
	while( m_threadRunning ) {
		continue;
	}
}

int CUARTChannel::open( std::string channelPath, bool enableReceiver, bool twoStopBits, bool parity, bool rtscts )
{
	if( this->isOpen() )
		return ERR_UART_ALREADY_OPEN;
	
#ifdef __linux__
	// Open the channel
	m_hChannelHandle = ::open( channelPath.c_str(), O_RDWR | O_NOCTTY | O_NDELAY );
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
	::tcgetattr( m_hChannelHandle, &m_uartOptions );
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

#ifdef __linux__
int CUARTChannel::flush()
{
	if( tcflush( m_hChannelHandle, TCIOFLUSH ) == -1 )
		return ERR_UART_FLUSH_CHANNEL;
	return ERR_OK;
}

bool CUARTChannel::setAttributes()
{
	if( tcsetattr( m_hChannelHandle, TCSAFLUSH, &m_uartOptions ) == -1 )
		return false;
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
