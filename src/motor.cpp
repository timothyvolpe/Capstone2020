#include <assert.h>
#include <cstddef>
#include <vector>
#include <cstring>
#include <algorithm>
#include "def.h"
#include "motor.h"
#include "wire_protocols.h"

#include <iostream>

uint16_t roboclaw_crc16( const std::vector<unsigned char> &buffer )
{
	unsigned int crc = 0;
	
	for( auto it = buffer.begin(); it != buffer.end(); it++ )
	{
		crc = crc ^ ((unsigned int)(*it) << 8);
		for (unsigned char bit = 0; bit < 8; bit++)
		{
			if (crc & 0x8000) {
				crc = (crc << 1) ^ 0x1021;
			} else {
				crc = crc << 1;
			}
		}
	}
	return (uint16_t)crc;
}

uint16_t roboclaw_crc16( RoboClawPacket packet )
{
	std::vector<unsigned char> data;
	data.reserve( offsetof(RoboClawPacket, crc) );
	data.push_back( packet.address );
	data.push_back( packet.command );
	if( packet.valueBytes.size() > 0 )
		data.insert( data.end(), packet.valueBytes.begin(), packet.valueBytes.end() );
	
	return roboclaw_crc16( data );
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
	if( packet.valueBytes.size() > 0 ) {
		crc16 = roboclaw_crc16( packet );
		packet.crc[0] = (crc16>>8) & 0xFF;
		packet.crc[1] = crc16 & 0xFF;
	}
	
	// Convert to binary data
	buffer.push_back( packet.address );
	buffer.push_back( packet.command );
	if( packet.valueBytes.size() > 0 )
	{
		buffer.insert( buffer.end(), packet.valueBytes.begin(), packet.valueBytes.end() );
		buffer.push_back( packet.crc[0] );
		buffer.push_back( packet.crc[1] );
	}
	
	return buffer;
}

int CMotorController::sendCommandARBBlocking( unsigned char command, const std::vector<unsigned char> &valueBytes, size_t responseLength, std::vector<unsigned char> &response )
{
	RoboClawPacket packet;
	int errCode;
	std::vector<unsigned char> buffer;
	size_t remainingBytes;
	
	packet.address = m_motorAddress;
	packet.command = command;
	packet.valueBytes = valueBytes;
	
	errCode = m_pMotorUARTReference->flush();
	if( errCode != ERR_OK )
		return errCode;
	if( !m_pMotorUARTReference->write( this->serializePacket( packet ) ) )
		return ERR_UART_WRITE;
	
	// Wait up to 10 ms for data, or until responseLength is reached
	std::chrono::steady_clock::time_point writeTime = std::chrono::steady_clock::now();
	response = std::vector<unsigned char>();
	remainingBytes = responseLength;
	while( std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - writeTime).count() < 10 )
	{
		if( m_pMotorUARTReference->dataAvailable() )
		{
			buffer = m_pMotorUARTReference->read( remainingBytes );
			if( buffer.size() > 0 ) {
				remainingBytes -= buffer.size();
				response.insert( response.end(), buffer.begin(), buffer.end() );
				if( remainingBytes <= 0 )
					break;
			}
		}
		else
			std::this_thread::sleep_for( std::chrono::milliseconds( 1 ) );
	}
	
	return ERR_OK;
}

int CMotorController::sendCommandBlocking( unsigned char command, size_t responseLength, std::vector<unsigned char> &response )
{
	return this->sendCommandARBBlocking( command, std::vector<unsigned char>(), responseLength, response );
}
int CMotorController::sendCommand8Blocking( unsigned char command, uint8_t value, size_t responseLength, std::vector<unsigned char> &response )
{
	return ERR_OK;
}
int CMotorController::sendCommand16Blocking( unsigned char command, uint16_t value, size_t responseLength, std::vector<unsigned char> &response )
{
	return ERR_OK;
}
int CMotorController::sendCommand32Blocking( unsigned char command, uint32_t value, size_t responseLength, std::vector<unsigned char> &response )
{
	return ERR_OK;
}

bool CMotorController::verifyResponse( unsigned char command, const std::vector<unsigned char> &response )
{
	if( response.size() < 1 )
		return false;
	else if( response.size() == 1 ) {
		if( response[0] == 0xFF )
			return true;
		else
			return false;
	}
	else if( response.size() < 3 ) {
		return false;
	}

	uint8_t checksumLo = response.back();
	uint8_t checksumHi = (*(response.end() - 2));
	
	uint16_t crc16 = checksumLo | checksumHi<<8;
	
	// Generate crc16 of rest of data, with command and address
	std::vector<unsigned char> checksummableBytes = std::vector<unsigned char>( response.begin(), response.end()-2 );
	checksummableBytes.insert( checksummableBytes.begin(), command );
	checksummableBytes.insert( checksummableBytes.begin(), m_motorAddress );
	uint16_t crc16_local = roboclaw_crc16( checksummableBytes );
	
	if( crc16 != crc16_local )
		return false;
		
	return true;
}

int CMotorController::getControllerInfo( std::string &versionStr )
{
	assert( m_pMotorUARTReference );
	
	std::vector<unsigned char> response;
	int errCode;
	
	if( (errCode = this->sendCommandBlocking( RoboClawCommand::READ_FIRMWARE, 48, response ) ) != ERR_OK )
		return errCode;
	if( response.empty() )
		return ERR_UART_NO_RESPONSE;
		
	// Verify response
	if( !this->verifyResponse( RoboClawCommand::READ_FIRMWARE, response ) ) {
		std::cout << "Failed CRC" << std::endl;
	}
		
	versionStr = std::string( response.begin(), response.end() );
	// Remove line ending added by device
	versionStr.erase(std::remove(versionStr.begin(), versionStr.end(), '\n'), versionStr.end());
	
	return ERR_OK;
}

int CMotorController::getControllerStatus( uint16_t *pStatus )
{
	std::vector<unsigned char> response;
	int errCode;
	
	if( (errCode = this->sendCommandBlocking( RoboClawCommand::READ_STATUS, 3, response ) ) != ERR_OK )
		return errCode;
	if( response.empty() )
		return ERR_UART_NO_RESPONSE;
	if( response.size() < 2 )
		return ERR_UART_INVALID_RESPONSE;
		
	(*pStatus) = (uint16_t)(response[1]) << 8 + response[0];
	
	return ERR_OK;
}

int CMotorController::forward( RoboClawChannels channelId, char speed )
{
	assert( m_pMotorUARTReference );
	assert( channelId == RoboClawChannels::CHANNEL1 || channelId == RoboClawChannels::CHANNEL2 );
	
	unsigned char command;
	int errCode;
	std::vector<unsigned char> response;
	
	// Select appropriate command based on channel
	switch( channelId )
	{
	case RoboClawChannels::CHANNEL1:
		command = RoboClawCommand::MOTOR1_FORWARD;
		break;
	case RoboClawChannels::CHANNEL2:
		command = RoboClawCommand::MOTOR2_FORWARD;
		break;
	default:
		return ERR_OK; // impossible...?
	}
	
	// Clamp speed
	if( speed < 0 || speed > 127 )
		speed = 0;
	
	if( (errCode = this->sendCommand8Blocking( command, speed, 1, response ) ) != ERR_OK )
		return errCode;

	return ERR_OK;
}
int CMotorController::reverse( RoboClawChannels channelId, char speed )
{
	assert( m_pMotorUARTReference );
	assert( channelId == RoboClawChannels::CHANNEL1 || channelId == RoboClawChannels::CHANNEL2 );
	
	unsigned char command;
	int errCode;
	std::vector<unsigned char> response;
	
	// Select appropriate command based on channel
	switch( channelId )
	{
	case RoboClawChannels::CHANNEL1:
		command = RoboClawCommand::MOTOR1_REVERSE;
		break;
	case RoboClawChannels::CHANNEL2:
		command = RoboClawCommand::MOTOR2_REVERSE;
		break;
	default:
		return ERR_OK; // impossible...?
	}
	
	// Clamp speed
	if( speed < 0 || speed > 127 )
		speed = 0;
	
	if( (errCode = this->sendCommand8Blocking( command, speed, 1, response ) ) != ERR_OK )
		return errCode;

	return ERR_OK;
}
