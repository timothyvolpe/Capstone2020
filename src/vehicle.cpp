#include <iostream>
#include <assert.h>
#include <memory>
#include <thread>
#include <algorithm>
#include <iterator>
#include <cctype>
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
	if( m_pSensorManager ) {
		delete m_pSensorManager;
		m_pSensorManager = 0;
	}
	if( m_pVehicleTerminal ) {
		m_pVehicleTerminal->shutdown();
		delete m_pVehicleTerminal;
		m_pVehicleTerminal = 0;
	}
}

int CVehicle::initialize()
{
	int errCode;

	// Initialize the terminal
	std::cout << "Starting debug logging...\n";
	m_pVehicleTerminal = new CTerminal();
	errCode = m_pVehicleTerminal->init();
	if( errCode != ERR_OK ) {
		Terminal()->print( "  Failed to initialize user terminal\n" );
		return errCode;
	}

	Terminal()->print( "  Initializing sensors...\n" );
	m_pSensorManager = new CSensorManager();
	errCode = m_pSensorManager->initSensors();
	if( errCode != ERR_OK ) {
		Terminal()->print( "  Sensor initialization: FAILED\n" );
		return errCode;
	}
	Terminal()->print( "  Sensor initialization: SUCCESS\n" );

	return ERR_OK;
}

int CVehicle::start()
{
	assert( !m_isRunning );

	Terminal()->print( "Ready\n" );
	m_isRunning = true;
	while( m_isRunning )
	{
		if( !m_messageQueue.empty() )
		{
			std::unique_ptr<message_t> pMsg  = std::move( m_messageQueue.front() );

			// Sort the messages
			switch( pMsg->message_id )
			{
			case MSGID_QUIT:
				m_isRunning = false;
				Terminal()->print( "Quitting...\n" );
				break;
			case MSGID_TERMINAL_MSG:
				this->parseCommandMessage( std::move( pMsg ) );
				break;
			case MSGID_UNKNOWN:
			default:
				Terminal()->print( "WARNING: Received unknown message (ID: %d)\n", pMsg->message_id );
				break;
			}
			m_messageQueue.pop();
		}
	}

	m_isRunning = false;

	return ERR_OK;
}

void CVehicle::parseCommandMessage( std::unique_ptr<message_t> pCommandMsg )
{
	assert( pCommandMsg );
	assert( pCommandMsg->message_id == MSGID_TERMINAL_MSG );

	// Cast
	terminal_msg_t *pMsgData = static_cast<terminal_msg_t*>(pCommandMsg.get());
	std::string cmdStr = pMsgData->command;
	std::string cmdStrClean;
	std::vector<std::string> tokens;

	// Convert to lowercase
	std::transform( cmdStr.begin(), cmdStr.end(), cmdStr.begin(),
		[]( unsigned char c ) { return std::tolower( c ); } );
	// Remove extra whitespace
	std::unique_copy( cmdStr.begin(), cmdStr.end(), std::back_insert_iterator<std::string>( cmdStrClean ),
		[]( char a, char b ) { return isspace( a ) && isspace( b ); } );
	// Tokenize into words
	std::string currentWord;
	for( auto it = cmdStrClean.begin(); it != cmdStrClean.end(); it++ )
	{
		if( (*it) != ' ' )
			currentWord += (*it);
		else {
			tokens.push_back( currentWord );
			currentWord = "";
		}
	}
	// Get last token
	if( !currentWord.empty() )
		tokens.push_back( currentWord );

	if( tokens.size() < 1 ) {
		Terminal()->print( "Received empty command\n" );
		return;
	}
	std::string commandName = tokens[0];

	// Evaluate commands
	if( commandName.compare( "quit" ) == 0 || commandName.compare( "exit" ) == 0 ) {
		message_t quitMsg( MSGID_QUIT, true, 0 );
		this->postMessage( std::make_unique<message_t>( quitMsg ) );
		return;
	}
	else {
		Terminal()->print( "Unknown command \'%s\'\n", commandName.c_str() );
	}
}

void CVehicle::postMessage( std::unique_ptr<message_t> pMsg )
{
	assert( pMsg );

	std::unique_lock<std::mutex> lock( m_msgLoopMutex );

	m_messageQueue.push( std::move( pMsg ) );
}

CTerminal* CVehicle::getTerminal() {
	assert( m_pVehicleTerminal );
	return m_pVehicleTerminal;
}