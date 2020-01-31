#include <chrono>
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
	// Create the user input thread
	m_inputThread = std::thread( &CTerminal::inputThreadMain, this );

	terminal_msg_t testMsg( MSGID_QUIT, true, 0 );
	testMsg.commandName = "test";

	CVehicle::instance().postMessage( testMsg );

	return ERR_OK;
}

void CTerminal::inputThreadMain()
{
	//std::cin
}