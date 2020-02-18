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

int CUARTChannel::open( std::string channelPath, bool enableReceiver )
{
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
	m_uartOptions.c_cflag = CS8 | CLOCAL;
	if( enableReceiver )
		m_uartOptions.c_cflag |= CREAD;
	else
		m_uartOptions.c_cflag &= ~CREAD; 
	tcflush( m_hChannelHandle, TCIFLUSH );
	tcsetattr( m_hChannelHandle, TCSANOW, &m_uartOptions );
#endif

	return ERR_OK;
}
void CUARTChannel::close()
{
#ifdef __linux__
	if( m_hChannelHandle >= 0 ) {
		::close( m_hChannelHandle );
	}
	m_hChannelHandle = -1;
#endif
}

#ifdef __linux__
int CUARTChannel::setBaudRate( speed_t baud )
{
	if( cfsetispeed( &m_uartOptions, baud ) == -1 ) {
		return ERR_UART_INVALID_BAUD;
	}
	if( cfsetospeed( &m_uartOptions, baud ) == -1 ) {
		return ERR_UART_INVALID_BAUD;
	}
	return ERR_OK;
}
#endif
