#include <assert.h>
#include <cstddef>
#include <vector>
#include <cstring>
#include <algorithm>
#include <endian.h>
#include "def.h"
#include "motor.h"
#include "wire_protocols.h"

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
	int tries;
	
	packet.address = m_motorAddress;
	packet.command = command;
	packet.valueBytes = valueBytes;
	
	tries = 0;
	while( tries < MOTOR_UART_TRIES )
	{
		tries++;
		
		errCode = m_pMotorUARTReference->flush();
		if( errCode != ERR_OK )
			return errCode;
		if( !m_pMotorUARTReference->write( this->serializePacket( packet ) ) )
			return ERR_UART_WRITE;
		
		// Wait up to 10 ms for data, or until responseLength is reached
		std::chrono::steady_clock::time_point writeTime = std::chrono::steady_clock::now();
		response = std::vector<unsigned char>();
		remainingBytes = responseLength;
		while( std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - writeTime).count() < MOTOR_UART_WAIT_MS )
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
		
		
		// Validate reponse
		if( response.empty() && !this->verifyResponse( command, response ) )
			continue;
		else
			break;
	}
	
	// Validate reponse
	if( response.empty() )
		return ERR_UART_NO_RESPONSE;
	if( !this->verifyResponse( command, response ) )
		return ERR_UART_INVALID_RESPONSE;
	
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

int CMotorController::sendCommand1616Blocking( unsigned char command, uint16_t value1, uint16_t value2, size_t responseLength, std::vector<unsigned char> &response )
{
	std::vector<unsigned char> bytes( 4 );
	bytes[0] = (value1 >> 8) & 0xFF;
	bytes[1] = value1 & 0xFF;
	bytes[2] = (value2 >> 8) & 0xFF;
	bytes[3] = value2 & 0xFF;
	return this->sendCommandARBBlocking( command, bytes, responseLength, response );
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
	
	if( (errCode = this->sendCommandBlocking( RoboClawCommand::READ_FIRMWARE, 50, response ) ) != ERR_OK )
		return errCode;
		
	versionStr = std::string( response.begin(), response.end() );
	// Remove line ending added by device
	versionStr.erase(std::remove(versionStr.begin(), versionStr.end(), '\n'), versionStr.end());
	
	return ERR_OK;
}

int CMotorController::getControllerStatus( uint16_t *pStatus )
{
	assert( pStatus );
	
	std::vector<unsigned char> response;
	int errCode;
	
	if( (errCode = this->sendCommandBlocking( RoboClawCommand::READ_STATUS, 6, response ) ) != ERR_OK )
		return errCode;
		
	// Combine hi and low bytes and account for endianness
	(*pStatus) = be16toh( response[1] + (response[0] << 8) );

	return ERR_OK;
}

int CMotorController::getTemperature( float *pTemp1, float *pTemp2 )
{
	assert( pTemp1 );
	
	std::vector<unsigned char> response;
	int errCode;
	uint16_t temp1, temp2;
	
	// Temperature 1
	if( (errCode = this->sendCommandBlocking( RoboClawCommand::READ_TEMPERATURE, 4, response ) ) != ERR_OK )
		return errCode;
		
	// Combine hi and low bytes and account for endianness
	temp1 = be16toh( response[1] + response[0] << 8 );
	(*pTemp1) = temp1 * 0.1f;
	
	// Temperature 2
	if( pTemp2 )
	{
		// Temperature 2
		if( (errCode = this->sendCommandBlocking( RoboClawCommand::READ_TEMPERATURE2, 4, response ) ) != ERR_OK )
			return errCode;
			
		temp2 = be16toh( response[1] + response[0] << 8 );
		(*pTemp2) = temp2 * 0.1f;
	}
	
	return ERR_OK;
}

int CMotorController::getMainBatteryVoltage( float *pVoltage )
{
	assert( pVoltage );
	
	std::vector<unsigned char> response;
	int errCode;
	uint16_t voltage;
	
	if( (errCode = this->sendCommandBlocking( RoboClawCommand::READ_VOLTAGE_MAIN, 4, response ) ) != ERR_OK )
		return errCode;
		
	// Combine hi and low bytes and account for endianness
	voltage = be16toh( response[1] + response[0] << 8 );
	(*pVoltage) = voltage * 0.1f;
	
	return ERR_OK;
}

int CMotorController::getLogicBatteryVoltage( float *pVoltage )
{
	assert( pVoltage );
	
	std::vector<unsigned char> response;
	int errCode;
	uint16_t voltage;
	
	if( (errCode = this->sendCommandBlocking( RoboClawCommand::READ_VOLTAGE_LOGIC, 4, response ) ) != ERR_OK )
		return errCode;
		
	// Combine hi and low bytes and account for endianness
	voltage = be16toh( response[1] + response[0] << 8 );
	(*pVoltage) = voltage * 0.1f;
	
	return ERR_OK;
}

// 0x00FB 	= 0b00000000 11111011
// 0x00FC	= 0b00000000 11111100
// 0x00FD	= 0b00000000 11111101

// 0x0000	= 0b00000000 00000000	Address 0x80
// 0x0100	= 0b00000001 00000000 	Address 0x81
// 0x0200	= 0b00000010 00000000	Address 0x82

// 0x00E0	= 0b00000000 11100000	Baud 460800

// 11111101
// --------
// 111XXXXX		= 0xE0, Baud 460800
// XXX1XXXX		= 0x10,	Battery Mode 4 Cell
// XXXX1XXX		= 0x08, MCU
// XXXXX1XX		= 0x04

int CMotorController::getConfigSettings( uint16_t *pConfigSettings )
{
	assert( pConfigSettings );
	
	std::vector<unsigned char> response;
	int errCode;
	
	if( (errCode = this->sendCommandBlocking( RoboClawCommand::READ_STANDARD_CONFIG, 4, response ) ) != ERR_OK )
		return errCode;
		
	// Combine hi and low bytes and account for endianness
	(*pConfigSettings) = be16toh( response[1] + response[0] << 8 );
	
	return ERR_OK;
}

int CMotorController::getMotorCurrents( float *pCurrentM1, float *pCurrentM2 )
{
	assert( pCurrentM1 && pCurrentM2 );
	
	std::vector<unsigned char> response;
	int errCode;
	
	if( (errCode = this->sendCommandBlocking( RoboClawCommand::READ_MOTOR_CURRENTS, 6, response ) ) != ERR_OK )
		return errCode;
	
	(*pCurrentM1) = be16toh( response[1] + response[0] << 8 ) / 100.0f;
	(*pCurrentM2) = be16toh( response[3] + response[2] << 8 ) / 100.0f;
	
	return ERR_OK;
}

int CMotorController::getMotorDutyCycles( float *pDuty1, float *pDuty2 )
{
	assert( pDuty1 && pDuty2 );
	
	std::vector<unsigned char> response;
	int errCode;
	
	if( (errCode = this->sendCommandBlocking( RoboClawCommand::READ_MOTOR_PWMS, 6, response ) ) != ERR_OK )
		return errCode;
		
	(*pDuty1) = be16toh( response[1] + response[0] << 8 ) / 327.67f;
	(*pDuty2) = be16toh( response[3] + response[2] << 8 ) / 327.67f;
	
	return ERR_OK;
}

int CMotorController::setMainVoltageLevels( float mainMin, float mainMax )
{
	std::vector<unsigned char> response;
	int errCode;
	uint16_t minVal, maxVal;
	
	minVal = (uint16_t)(mainMin / 0.1f);
	maxVal = (uint16_t)(mainMax / 0.1f);
	
	if( (errCode = this->sendCommand1616Blocking( RoboClawCommand::SET_MAIN_VOLTAGES, minVal, maxVal, 1, response ) ) != ERR_OK )
		return errCode;
	
	return ERR_OK;
}
int CMotorController::setLogicVoltageLevels( float logicMin, float logicMax )
{
	std::vector<unsigned char> response;
	int errCode;
	uint16_t minVal, maxVal;
	
	minVal = (uint16_t)(logicMin / 0.1f);
	maxVal = (uint16_t)(logicMax / 0.1f);
	
	if( (errCode = this->sendCommand1616Blocking( RoboClawCommand::SET_LOGIC_VOLTAGES, minVal, maxVal, 1, response ) ) != ERR_OK )
		return errCode;
	
	return ERR_OK;
}
int CMotorController::getMainVoltageLevels( float *pMainMin, float *pMainMax )
{
	assert( pMainMin && pMainMax );
	
	std::vector<unsigned char> response;
	int errCode;
	
	if( (errCode = this->sendCommandBlocking( RoboClawCommand::READ_MAIN_VOLTAGES, 6, response ) ) != ERR_OK )
		return errCode;
		
	(*pMainMin) = be16toh( response[1] + response[0] << 8 ) * 0.1f;
	(*pMainMax) = be16toh( response[3] + response[2] << 8 ) * 0.1f;
	
	return ERR_OK;
}
int CMotorController::getLogicVoltageLevels( float *pLogicMin, float *pLogicMax )
{
	assert( pLogicMin && pLogicMax );
	
	std::vector<unsigned char> response;
	int errCode;
	
	if( (errCode = this->sendCommandBlocking( RoboClawCommand::READ_LOGIC_VOLTAGES, 6, response ) ) != ERR_OK )
		return errCode;
		
	(*pLogicMin) = be16toh( response[1] + response[0] << 8 ) * 0.1f;
	(*pLogicMax) = be16toh( response[3] + response[2] << 8 ) * 0.1f;
	
	return ERR_OK;
}

int CMotorController::forward( RoboClawChannels channelId, int8_t speed )
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
int CMotorController::reverse( RoboClawChannels channelId, int8_t speed )
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
