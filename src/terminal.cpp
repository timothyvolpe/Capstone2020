#include <chrono>
#include <iostream>
#include <string>
#include "def.h"
#include "vehicle.h"
#include "terminal.h"
#include "messages.h"

CTerminal::CTerminal() {

}
CTerminal::~CTerminal() {

}

int CTerminal::init()
{
	// Create and start user input thread
	m_threadRunning = true;
	m_inputThread = std::thread( &CTerminal::inputThreadMain, this );

	return ERR_OK;
}
void CTerminal::shutdown()
{
	if( m_threadRunning ) {
		m_threadRunning = false;
		// Because we can't interrupt cin
		this->print( "\nPress enter to exit...\n" );
	}
	m_inputThread.join();
}

void CTerminal::inputThreadMain()
{
	//std::cin
	while( m_threadRunning )
	{
		std::string userInput;

		// Need platform-specific fix to prevent interleaving characters
		std::getline( std::cin, userInput );
		if( !userInput.empty() )
		{
			// Send a message
			terminal_msg_t termMsg( MSGID_TERMINAL_MSG, true, 0 );
			termMsg.command = userInput;
			LocalVehicle().postMessage( std::make_unique<terminal_msg_t>( termMsg ) );
		}
	}
}
