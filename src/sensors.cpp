#include <iostream>
#include "sensors.h"
#include "ultrasonic.h"

CSensorManager::CSensorManager()
{
}
CSensorManager::~CSensorManager()
{
	for( unsigned int i = 0; i < ULTRASONIC_SENSOR_COUNT; i++ )
	{
		if( m_pUltrasonicsSensors[i] ) {
			delete m_pUltrasonicsSensors[i];
			m_pUltrasonicsSensors[i] = 0;
		}
	}
}

bool CSensorManager::initSensors()
{
	// Initialize ultrasonic sensors
	std::cout << "  Initializing ultrasonic sensors...\n";
	for( unsigned int i = 0; i < ULTRASONIC_SENSOR_COUNT; i++ ) {
		std::cout << "  - Ultrasonic sensors " << i+1 << "... ";
		m_pUltrasonicsSensors[i] = new CUltrasonicSensor();
		std::cout << "SUCCESS\n";
	}

	return true;
}

void CSensorManager::startSensors()
{
}