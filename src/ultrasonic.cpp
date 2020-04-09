#include <thread>
#include <assert.h>
#include "ultrasonic.h"
#include "def.h"
#include "wire_protocols.h"
#include "vehicle.h"

CUltrasonicSensor::CUltrasonicSensor( CI2CBus *pI2CBus, unsigned char address )
{
	m_sensorState = UltrasonicSensorState::STATE_TAKE_READING;
	m_consecMissedReadings = 0;
	
	m_lastRangeValue = 0;
	
	m_sensorAddress = address;
	m_pI2CBus = pI2CBus;
}
CUltrasonicSensor::~CUltrasonicSensor() {
}

int CUltrasonicSensor::initialize()
{
	// Check that a device exists at the other end
	m_pI2CBus->checkAddress( m_sensorAddress );
	
	m_lastReadingTaken = std::chrono::steady_clock::now();
	
	return ERR_OK;
}

int CUltrasonicSensor::update()
{
	int errCode;
	std::vector<unsigned char> response;
	
	// Check reading age
	if( std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - m_lastReadingTaken).count() > ULTRASONIC_READING_MAX_AGE_MS )
		return ERR_ULTRASONIC_READING_EXPIRED;
	
	switch( m_sensorState )
	{
	case UltrasonicSensorState::STATE_TAKE_READING:
		// Try to take a reading
		if( (errCode = this->takeReading()) != ERR_OK ) {
			if( errCode == ERR_ULTRASONIC_NOT_READY ) {
				break;
			}
			return errCode;
		}
		m_sensorState = UltrasonicSensorState::STATE_VERIFY_TAKE_COMMAND;
		break;
	case UltrasonicSensorState::STATE_VERIFY_TAKE_COMMAND:
		// Check to see if the command was acknowledged
		if( !m_pI2CBus->check_ack_i2c( m_sensorAddress, 0 ) )
		{
			m_consecMissedReadings++;
			if( m_consecMissedReadings >= ULTRASONIC_MISSED_READING_LIMIT )
				return ERR_ULTRASONIC_TOO_MANY_MISSED;
			m_sensorState = UltrasonicSensorState::STATE_TAKE_READING;
			break;
		}
		m_lastRange = std::chrono::steady_clock::now();
		m_sensorState = UltrasonicSensorState::STATE_READ_READING;
		break;
	case UltrasonicSensorState::STATE_READ_READING:
		// Try to read the reading
		if( (errCode = this->getReading()) != ERR_OK ) {
			if( errCode == ERR_ULTRASONIC_NOT_READY ) {
				break;
			}
			return errCode;
		}
		m_sensorState = UltrasonicSensorState::STATE_VERIFY_READ_COMMAND;
		break;
	case UltrasonicSensorState::STATE_VERIFY_READ_COMMAND:
		// Check to see if the command was acknowledged with a response
		if( !m_pI2CBus->receive_i2c( m_sensorAddress, 0, response ) || response.size() != 2 )
		{
			m_consecMissedReadings++;
			if( m_consecMissedReadings >= ULTRASONIC_MISSED_READING_LIMIT )
				return ERR_ULTRASONIC_TOO_MANY_MISSED;
			m_sensorState = UltrasonicSensorState::STATE_TAKE_READING;
			break;
		}
		else
			m_consecMissedReadings = 0;
		m_lastReadingTaken = std::chrono::steady_clock::now();
		m_lastRangeValue = response[1] | (response[0] << 8); 
		m_sensorState = UltrasonicSensorState::STATE_TAKE_READING;
		break;
	default:
		assert( false ); // this shouldnt happen
		return ERR_ULTRASONIC_NOT_READY;
	}
	
	return ERR_OK;
}

int CUltrasonicSensor::takeReading()
{
	// Check to make sure appropriate time has elapsed 
	if( std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - m_lastRange).count() < ULTRASONIC_OFF_TIME_MS )
		return ERR_ULTRASONIC_NOT_READY;
	
	// TODO: Check status pin to make sure device is ready
	
	// Send command to take reading
	if( !m_pI2CBus->i2c_write_8( m_sensorAddress, ULTRASONIC_COMMAND_RANGE ) )
		return ERR_ULTRASONIC_RANGE;
	
	return ERR_OK;
}
int CUltrasonicSensor::getReading()
{
	std::vector<unsigned char> response;
	
	// Check to make sure appropriate time has elapsed 
	if( std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - m_lastReadingTaken).count() < ULTRASONIC_READ_TIME_MS )
		return ERR_ULTRASONIC_NOT_READY;
		
	// TODO: Poll status pin to make sure it is low
	
	// Send read command
	if( !m_pI2CBus->i2c_read_16( m_sensorAddress ) )
		return ERR_ULTRASONIC_RANGE;
	
	return ERR_OK;
}
int CUltrasonicSensor::setAddress( uint8_t newAddress )
{
	// Check status pin to make sure device is ready
	
	// Send command to change address
	// Send address unlock bytes
	// Send new address
	
	// Wait for the device to reset
	std::this_thread::sleep_for( std::chrono::milliseconds( ULTRASONIC_RESET_DELAY_MS ) );
	
	// Check for response to ensure address changed successfully
	
	return ERR_OK;
}

uint16_t CUltrasonicSensor::getLastReading() {
	return m_lastRangeValue;
}
