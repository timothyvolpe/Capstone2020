#include <iostream>
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
	std::cout << "    Initializing ultrasonic sensors...\n";
	for( unsigned int i = 0; i < ULTRASONIC_SENSOR_COUNT; i++ ) {
		std::cout << "    - Ultrasonic sensor " << i+1 << "... ";
		m_pUltrasonicSensors[i] = new CUltrasonicSensor();
		std::cout << "SUCCESS\n";
	}
	// Initialize RPLIDAR
	std::cout << "    Initializing LIDAR sensor... ";
	m_pLIDARSensor = new CLIDARSensor();
	std::cout << "SUCCESS\n";
	// Initialize IMU
	std::cout << "    Initializing inertial motion sensors... ";
	m_pInertialMotionSensors = new CInertialMotionSensors();
	std::cout << "SUCCESS\n";
	// Initialize GPS
	std::cout << "    Initializing GPS unit... ";
	m_pGPS = new CGlobalPositioning();
	std::cout << "SUCCESS\n";

	return ERR_OK;
}

void CSensorManager::startSensors()
{
}