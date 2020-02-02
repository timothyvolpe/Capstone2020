#include <iostream>
#include "vehicle.h"
#include "sensors.h"
#include "ultrasonic.h"
#include "lidar.h"
#include "kinetics.h"
#include "def.h"

CSensorManager::CSensorManager()
{
	memset( &m_pUltrasonicSensors, 0, sizeof( CUltrasonicSensor )*ULTRASONIC_SENSOR_COUNT );
	m_pLIDARSensor = 0;
	m_pInertialMotionSensors = 0;
	m_pGPS = 0;
}
CSensorManager::~CSensorManager()
{
	for( unsigned int i = 0; i < ULTRASONIC_SENSOR_COUNT; i++ )
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
}

int CSensorManager::initSensors()
{
	// Initialize ultrasonic sensors
	Terminal()->print( "    Initializing ultrasonic sensors...\n" );
	for( unsigned int i = 0; i < ULTRASONIC_SENSOR_COUNT; i++ ) {
		Terminal()->print( "    - Ultrasonic sensor %d...", i+1 );
		m_pUltrasonicSensors[i] = new CUltrasonicSensor();
		Terminal()->print( "SUCCESS\n" );
	}
	// Initialize RPLIDAR
	Terminal()->print( "    Initializing LIDAR sensor... " );
	m_pLIDARSensor = new CLIDARSensor();
	Terminal()->print( "SUCCESS\n" );
	// Initialize IMU
	Terminal()->print( "    Initializing inertial motion sensors... " );
	m_pInertialMotionSensors = new CInertialMotionSensors();
	Terminal()->print( "SUCCESS\n" );
	// Initialize GPS
	Terminal()->print( "    Initializing GPS unit... " );
	m_pGPS = new CGlobalPositioning();
	Terminal()->print( "SUCCESS\n" );

	return ERR_OK;
}

void CSensorManager::startSensors()
{
}