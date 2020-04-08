#include <thread>
#include "ultrasonic.h"
#include "def.h"
#include "wire_protocols.h"

CUltrasonicSensor::CUltrasonicSensor( CI2CBus *pI2CBus, unsigned char address ) {
	m_sensorAddress = address;
	m_pI2CBus = pI2CBus;
}
CUltrasonicSensor::~CUltrasonicSensor() {
}

int CUltrasonicSensor::initialize()
{
	// Check that a device exists at the other end
	m_pI2CBus->checkAddress( m_sensorAddress );
	
	return ERR_OK;
}

int CUltrasonicSensor::takeReading()
{
	// Check to make sure appropriate time has elapsed 
	if( std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - m_lastReadingTaken).count() < ULTRASONIC_OFF_TIME_MS )
		return ERR_ULTRASONIC_NOT_READY;
	
	// TODO: Check status pin to make sure device is ready
	
	// Send command to take reading
	if( !m_pI2CBus->write_i2c_byte( m_sensorAddress, ULTRASONIC_COMMAND_RANGE, false ) )
		return ERR_ULTRASONIC_RANGE;
	// Make sure it was read correctly
	std::vector<unsigned char> response;
	if( !m_pI2CBus->read_i2c( m_sensorAddress, 100, response ) )
		return ERR_ULTRASONIC_RANGE;
	
	// Save time point
	m_lastReadingTaken = std::chrono::steady_clock::now();
	
	return ERR_OK;
}
int CUltrasonicSensor::getReading( uint16_t *pRangeReading )
{
	std::vector<unsigned char> response;
	
	(*pRangeReading) = 0;
	
	// Check to make sure appropriate time has elapsed 
	if( std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - m_lastReadingTaken).count() < ULTRASONIC_READ_TIME_MS )
		return ERR_ULTRASONIC_NOT_READY;
		
	// TODO: Poll status pin to make sure it is low
	
	// Send command to get range value
	if( !m_pI2CBus->write_i2c_byte( m_sensorAddress, ULTRASONIC_COMMAND_RANGE, true, 2 ) )
		return ERR_ULTRASONIC_RANGE;
	// Check for reading
	if( !m_pI2CBus->read_i2c( m_sensorAddress, 100, response ) )
		return ERR_ULTRASONIC_RANGE;
	// Convert response to integer
	(*pRangeReading) = response.size()+1;
	
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

int CUltrasonicSensor::takeReadingAndWait( uint16_t *pRangeReading )
{
	int errCode;
	
	// Instruct the sensor to take a reading
	if( (errCode = this->takeReading()) != ERR_OK )
		return errCode;
	// Wait
	m_pI2CBus->flushWriteBlocking( 1000 );
	std::this_thread::sleep_for( std::chrono::milliseconds( ULTRASONIC_READ_TIME_MS ) );
	if( (errCode = this->getReading( pRangeReading )) != ERR_OK )
		return errCode;
		
	return ERR_OK;
}
