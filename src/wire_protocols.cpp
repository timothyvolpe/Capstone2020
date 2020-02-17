#include <fcntl.h>
#include <unistd.h>
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
