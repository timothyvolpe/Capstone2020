#include <iostream>
#include <assert.h>
#include <memory>
#include "vehicle.h"
#include "sensors.h"
#include "def.h"
#include "terminal.h"
#include "messages.h"

CVehicle::CVehicle() {
	m_pVehicleTerminal = 0;
	m_pSensorManager = 0;
	m_isRunning = false;
}
CVehicle::~CVehicle()
{
	if( m_pVehicleTerminal ) {
		delete m_pVehicleTerminal;
		m_pVehicleTerminal = 0;
	}
	if( m_pSensorManager ) {
		delete m_pSensorManager;
		m_pSensorManager = 0;
	}
}

int CVehicle::initialize()
{
	int errCode;

	std::cout << "  Initializing sensors...\n";
	m_pSensorManager = new CSensorManager();
	errCode = m_pSensorManager->initSensors();
	if( errCode != ERR_OK ) {
		std::cout << "  Sensor initialization: FAILED\n";
		return errCode;
	}
	std::cout << "  Sensor initialization: SUCCESS\n";

	// Initialize the terminal
	m_pVehicleTerminal = new CTerminal();
	errCode = m_pVehicleTerminal->init();
	if( errCode != ERR_OK ) {
		std::cout << "  Failed to initialize user terminal\n";
		return errCode;
	}

	return ERR_OK;
}

int CVehicle::start()
{
	assert( !m_isRunning );

	m_isRunning = true;
	while( m_isRunning ) {
		if( !m_messageQueue.empty() )
		{
			std::unique_ptr<message_t> pMsg  = std::move( m_messageQueue.front() );
			terminal_msg_t *pMsgData = static_cast<terminal_msg_t*>(pMsg.get());

			std::cout << "Our Copy ID: " << pMsgData->message_id << "\n";
			std::cout << "Our Copy Importance: " << pMsgData->important << "\n";
			std::cout << "Our Copy Timeout: " << pMsgData->timeoutMS << "\n";
			std::cout << "Our Copy Command: " << pMsgData->commandName.c_str() << "\n";

			m_messageQueue.pop();
		}
	}

	m_isRunning = false;

	return ERR_OK;
}

void CVehicle::postMessage( message_t &pMsg )
{
	std::unique_ptr<message_t> msgPtr = std::make_unique<message_t>( pMsg );

	terminal_msg_t *pMsgData = static_cast<terminal_msg_t*>(msgPtr.get());

	m_messageQueue.push( std::move(msgPtr) );

	/*std::cout << "Our Copy ID: " << ourCopy.message_id << "\n";
	std::cout << "Our Copy Importance: " << ourCopy.important << "\n";
	std::cout << "Our Copy Timeout: " << ourCopy.timeoutMS << "\n";*/
}