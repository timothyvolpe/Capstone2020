#include <iostream>
#include "vehicle.h"
#include "sensors.h"

CVehicle::CVehicle() {
	m_pSensorManager = 0;
}
CVehicle::~CVehicle()
{
	if( m_pSensorManager ) {
		delete m_pSensorManager;
		m_pSensorManager = 0;
	}
}

bool CVehicle::initialize()
{

	std::cout << "  Initializing sensors...\n";
	m_pSensorManager = new CSensorManager();
	if( !m_pSensorManager->initSensors() ) {
		std::cout << "  Sensor initialization: FAILED\n";
		return false;
	}
	std::cout << "  Sensor initialization: SUCCESS\n";

	return true;
}