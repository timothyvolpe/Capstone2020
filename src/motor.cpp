#include <assert.h>
#include <cstddef>
#include <vector>
#include <cstring>
#include "def.h"
#include "motor.h"
#include "wire_protocols.h"

uint16_t robowclaw_crc16( RoboClawPacket packet )
{
	uint16_t crc = 0;
	std::vector<unsigned char> data;
	data.reserve( offsetof(RoboClawPacket, crc) );
	data.push_back( packet.address );
	data.push_back( packet.command );
	data.push_back( packet.value );
	
	for( auto it = data.begin(); it != data.end(); it++ )
	{
		crc = crc ^ ((uint16_t)(*it) << 8);
		for (unsigned char bit = 0; bit < 8; bit++)
		{
			if (crc & 0x8000) {
				crc = (crc << 1) ^ 0x1021;
			} else {
				crc = crc << 1;
			}
		}
	}
	return crc;
}

CMotorController::CMotorController( CUARTChannel *pUART, unsigned char address ) 
{
	m_pMotorUARTReference = pUART;
	m_motorAddress = address;
	
	m_lastForwardSpeedSet = 0;
	m_lastReverseSpeedSet = 0;
}
CMotorController::~CMotorController() {
	m_pMotorUARTReference = 0;
}

int CMotorController::init() {
	return ERR_OK;
}
void CMotorController::shutdown() {
}

std::vector<unsigned char> CMotorController::serializePacket( RoboClawPacket &packet )
{
	uint16_t crc16;
	std::vector<unsigned char> buffer;
	
	// Generate CRC
	crc16 = robowclaw_crc16( packet );
	packet.crc[0] = (crc16>>8) & 0xFF;
	packet.crc[1] = crc16 & 0xFF;
	
	// Convert to binary data
	auto const ptr = reinterpret_cast<unsigned char*>(&packet);
	buffer = std::vector<unsigned char>( ptr, ptr + sizeof(packet) );
	
	return buffer;
}

int CMotorController::getControllerInfo( std::string &versionStr )
{
	assert( m_pMotorUARTReference );
	
	RoboClawPacket packet;
	std::vector<unsigned char> response;
	int errCode;
	
	packet.address = m_motorAddress;
	packet.command = RoboClawCommand::READ_FIRMWARE;
	packet.value = 0;
	
	errCode = m_pMotorUARTReference->flush();
	if( errCode != ERR_OK )
		return errCode;
	if( !m_pMotorUARTReference->write( this->serializePacket( packet ) ) )
		return ERR_UART_WRITE;
	
	// Wait up to 10 ms (after 10 ms Roboclaw discards data)
	std::chrono::steady_clock::time_point writeTime = std::chrono::steady_clock::now();
	while( std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - writeTime).count() < 10 )
	{
		if( m_pMotorUARTReference->dataAvailable() ) {
			response = m_pMotorUARTReference->read( 0 );
			versionStr = std::string( response.begin(), response.end() );
		}
		else
			std::this_thread::sleep_for( std::chrono::milliseconds( 1 ) );
	}
	
	if( response.empty() )
		return ERR_UART_NO_RESPONSE;
	
	
	return ERR_OK;
}

int CMotorController::forward( RoboClawChannels channelId, char speed )
{
	assert( m_pMotorUARTReference );
	assert( channelId == RoboClawChannels::CHANNEL1 || channelId == RoboClawChannels::CHANNEL2 );
	
	RoboClawPacket packet;
	
	packet.address = m_motorAddress;
	
	// Select appropriate command based on channel
	switch( channelId )
	{
	case RoboClawChannels::CHANNEL1:
		packet.command = RoboClawCommand::MOTOR1_FORWARD;
		break;
	case RoboClawChannels::CHANNEL2:
		packet.command = RoboClawCommand::MOTOR2_FORWARD;
		break;
	default:
		return ERR_OK; // impossible...?
	}
	
	// Clamp speed
	if( speed < 0 || speed > 127 )
		speed = 0;
	packet.value = speed;
	
	// Write to port
	if( !m_pMotorUARTReference->write( this->serializePacket( packet ) ) )
		return ERR_UART_WRITE;

	return ERR_OK;
}
int CMotorController::reverse( RoboClawChannels channelId, char speed )
{
	assert( m_pMotorUARTReference );
	
	return ERR_OK;
}
