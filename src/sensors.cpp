#include <iostream>
#include <cstring>
#include "vehicle.h"
#include "sensors.h"
#include "ultrasonic.h"
#include "lidar.h"
#include "kinetics.h"
#include "def.h"
#include "wire_protocols.h"
#include "config.h"

CSensorManager::CSensorManager()
{
	m_pI2cBus = 0;
	
	std::memset( &m_pUltrasonicSensors[0], 0, sizeof( CUltrasonicSensor* )*ULTRASONIC_MAX_SENSOR_COUNT );
	m_pLIDARSensor = 0;
	m_pInertialMotionSensors = 0;
	m_pGPS = 0;
}
CSensorManager::~CSensorManager()
{
	for( unsigned int i = 0; i < ULTRASONIC_MAX_SENSOR_COUNT; i++ )
	{
		if( m_pUltrasonicSensors[i] ) {
			delete m_pUltrasonicSensors[i];
			m_pUltrasonicSensors[i] = 0;
		}
	}
	if( m_pLIDARSensor ) {
		delete m_pLIDARSensor;
		m_pLIDARSensor = 0;
	}
	if( m_pInertialMotionSensors ) {
		delete m_pInertialMotionSensors;
		m_pInertialMotionSensors = 0;
	}
	if( m_pGPS ) {
		delete m_pGPS;
		m_pGPS = 0;
	}
	if( m_pI2cBus ) {
		m_pI2cBus->close();
		delete m_pI2cBus;
		m_pI2cBus = 0;
	}
}

int CSensorManager::initSensors()
{
	int errCode;
	int sensorCount = CVehicle::instance().getConfig()->getUltrasonicSensorCount();
	
	// Open i2c comm bus
	Terminal()->startItem( "Setting up I2C bus" );
	m_pI2cBus = new CI2CBus( "SensorI2C" );
	errCode = m_pI2cBus->open( "/dev/i2c-1" );
	if( errCode != ERR_OK ) {
		Terminal()->finishItem( false );
		return errCode;
	}
	Terminal()->finishItem( true );
	
	// Initialize ultrasonic sensors
	Terminal()->startItem( "Initializing ultrasonic sensors" );
	uint8_t sensorAddress[] = {
		CVehicle::instance().getConfig()->getUltrasonicFLAddress(),
		CVehicle::instance().getConfig()->getUltrasonicFRAddress(),
		CVehicle::instance().getConfig()->getUltrasonicBLAddress(),
		CVehicle::instance().getConfig()->getUltrasonicBRAddress()
	};
	assert( sizeof(sensorAddress) / sizeof( uint8_t) >= ULTRASONIC_MAX_SENSOR_COUNT );
	if( sensorCount < ULTRASONIC_MAX_SENSOR_COUNT )
		Terminal()->printImportant( "WARNING: Not using all ultrasonic sensors, should only be used for testing!\nChange ultrasonic.sensor_count in config.ini to %d\n", ULTRASONIC_MAX_SENSOR_COUNT );
	
	for( unsigned int i = 0; i < sensorCount; i++ )
	{
		Terminal()->startItem( "Ultrasonic sensor %d", i+1 );
		m_pUltrasonicSensors[i] = new CUltrasonicSensor( m_pI2cBus, sensorAddress[i] );	
		if( (errCode = m_pUltrasonicSensors[i]->initialize()) != ERR_OK )
			return errCode;
		Terminal()->finishItem( true );
	}
	Terminal()->finishItem( true ); // Initializing ultrasonic sensors
	
	// Initialize RPLIDAR
	Terminal()->startItem( "Initializing LIDAR sensor" );
	m_pLIDARSensor = new CLIDARSensor();
	Terminal()->finishItem( true );
	
	// Initialize IMU
	Terminal()->startItem( "Initializing inertial motion sensors" );
	m_pInertialMotionSensors = new CInertialMotionSensors();
	Terminal()->finishItem( true );
	
	// Initialize GPS
	Terminal()->startItem( "Initializing GPS unit" );
	m_pGPS = new CGlobalPositioning();
	Terminal()->finishItem( true );

	return ERR_OK;
}

void CSensorManager::startSensors()
{
}

int CSensorManager::update()
{
	int errCode;
	
	// Check comm threads for errors
	errCode = m_pI2cBus->getThreadError();
	if( errCode != ERR_OK ) {
		Terminal()->printImportant( "ERROR: There was a failure in the %s thread\n", m_pI2cBus->getPortName().c_str() );
		return errCode;
	}
	
	return ERR_OK;
}

bool CSensorManager::printUltrasonicReadings()
{
	int sensorCount = CVehicle::instance().getConfig()->getUltrasonicSensorCount();
	int errCode;
	uint16_t rangeReading;
	
	Terminal()->print( "Taking ultrasonic range readings...\n" );
	
	for( unsigned int i = 0; i < sensorCount; i++ )
	{
		if( (errCode = m_pUltrasonicSensors[i]->takeReadingAndWait( &rangeReading )) == ERR_OK )
			Terminal()->printImportant( "Ultrasonic %d: %d\n", i, rangeReading );
		else {
			Terminal()->printImportant( "Failed to read sensor %d\n", i );
			return false;
		}
	}
	
	Terminal()->printImportant( "Done!\n" );
	
	return true;
}

CI2CBus* CSensorManager::getI2CBus() {
	return m_pI2cBus;
}
