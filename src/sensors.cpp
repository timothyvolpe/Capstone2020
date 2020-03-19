#include <iostream>
#include <cstring>
#include "vehicle.h"
#include "sensors.h"
#include "ultrasonic.h"
#include "lidar.h"
#include "kinetics.h"
#include "def.h"

CSensorManager::CSensorManager()
{
	std::memset( &m_pUltrasonicSensors, 0, sizeof( CUltrasonicSensor )*ULTRASONIC_SENSOR_COUNT );
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
	Terminal()->startItem( "Initializing ultrasonic sensors" );
	for( unsigned int i = 0; i < ULTRASONIC_SENSOR_COUNT; i++ ) {
		Terminal()->startItem( "Ultrasonic sensor %d", i+1 );
		m_pUltrasonicSensors[i] = new CUltrasonicSensor();
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
