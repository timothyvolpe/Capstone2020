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
#include "wire_protocols.h"
#include "motor.h"
#include "config.h"

CVehicle::CVehicle()
{
	m_pVehicleTerminal = 0;
	m_pSensorManager = 0;
	m_isRunning = false;
	
	m_pConfig = 0;

	m_pMotionManager = 0;
	
	m_lastMotorUpdate = std::chrono::steady_clock::now(); 
}
CVehicle::~CVehicle()
{
}

int CVehicle::initialize()
{
	int errCode;

	// Initialize the terminal
	std::cout << "  Starting debug logging...\n";
	m_pVehicleTerminal = new CTerminal();
	errCode = m_pVehicleTerminal->init();
	if( errCode != ERR_OK ) {
		Terminal()->printImportant( "  Failed to initialize user terminal\n" );
		return errCode;
	}
	Terminal()->print( "Debug logging started\n" );
	
	Terminal()->printImportant( "Initializing vehicle...\n\n" );
	
	// Load config
	Terminal()->startItem( "Loading config" );
	m_pConfig = new CConfig();
	if( (errCode = m_pConfig->load()) != ERR_OK ) {
		Terminal()->finishItem( false );
		return errCode;
	}
	Terminal()->finishItem( true );

	// Sensors
	Terminal()->startItem( "Initializing sensors" );
	m_pSensorManager = new CSensorManager();
	errCode = m_pSensorManager->initSensors();
	if( errCode != ERR_OK ) {
		Terminal()->finishItem( false );
		return errCode;
	}
	Terminal()->finishItem( true );

	// Motor Controllers
	Terminal()->startItem( "Initializing motion manager" );
	
	m_pMotionManager = new CMotionManager( "/dev/serial0" );
	if( (errCode = m_pMotionManager->initialize()) != ERR_OK ) {
		Terminal()->finishItem( false );
		return errCode;
	}
	Terminal()->finishItem( true );
	
	Terminal()->startItem( "Starting motion manager" );
	if( (errCode = m_pMotionManager->start()) != ERR_OK ) {
		Terminal()->finishItem( false );
		return errCode;
	}
	
	Terminal()->finishItem( true );

	return ERR_OK;
}
void CVehicle::shutdown()
{	
	if( m_pSensorManager ) {
		delete m_pSensorManager;
		m_pSensorManager = 0;
	}
	if( m_pMotionManager ) {
		m_pMotionManager->shutdown();
		delete m_pMotionManager;
		m_pMotionManager = 0;
	}
	if( m_pVehicleTerminal ) {
		m_pVehicleTerminal->shutdown();
		delete m_pVehicleTerminal;
		m_pVehicleTerminal = 0;
	}
	if( m_pConfig ) {
		delete m_pConfig;
		m_pConfig = 0;
	}
}

int CVehicle::start()
{
	assert( !m_isRunning );
	
	int errCode;

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
		if( ( errCode = this->update() ) != ERR_OK )
			break;
		Terminal()->update();
	}

	m_isRunning = false;

	return errCode;
}

int CVehicle::update()
{
	std::chrono::steady_clock::time_point curtime = std::chrono::steady_clock::now();
	int errCode;
	
	// Update sensors if necessary
	if( std::chrono::duration_cast<std::chrono::milliseconds>(curtime - m_lastSensorUpdate).count() > ((1/SENSOR_UPDATE_FREQUENCY)*1000.0) ) {
		if( (errCode = m_pSensorManager->update()) != ERR_OK )
			return errCode;
		m_lastSensorUpdate = curtime;
	}
	
	// Update motors if necessary
	if( std::chrono::duration_cast<std::chrono::milliseconds>(curtime - m_lastMotorUpdate).count() > ((1/MOTOR_UPDATE_FREQUENCY)*1000.0) ) {
		if( (errCode = m_pMotionManager->update()) != ERR_OK )
			return errCode;
		m_lastMotorUpdate = curtime;
	}
	
	return ERR_OK;
}

void CVehicle::parseCommandMessage( std::unique_ptr<message_t> pCommandMsg )
{
	assert( pCommandMsg );
	assert( pCommandMsg->message_id == MSGID_TERMINAL_MSG );
	
	int errCode;

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
	if( commandName.compare( "quit" ) == 0 || commandName.compare( "exit" ) == 0 || commandName.compare( "stop" ) == 0 ) {
		message_t quitMsg( MSGID_QUIT, true, 0 );
		this->postMessage( std::make_unique<message_t>( quitMsg ) );
		return;
	}
	else if( commandName.compare( "help" ) == 0 )
	{
		Terminal()->print( "\nCommand List:\n" );
		Terminal()->print( "quit\t\t\t- Exits the program safely.\n" );
		Terminal()->print( "exit\t\t\t- Exits the program safely.\n" );
		Terminal()->print( "stop\t\t\t- Exits the program safely.\n" );
		Terminal()->print( "mocstatus\t\t- Displays the motor controller status info.\n" );
		Terminal()->print( "forward [speed]\t- Drive both main motors forward. Speed 0-127\n" );
		Terminal()->print( "reverse [speed]\t- Drive both main motors in reverse. Speed 0-127\n" );
		Terminal()->print( "forward1 [speed]\t- Drive main motor on channel 1 forward. Speed 0-127\n" );
		Terminal()->print( "reverse1 [speed]\t- Drive main motor on channel 1 in reverse. Speed 0-127\n" );
		Terminal()->print( "forward2 [speed]\t- Drive main motor on channel 2 forward. Speed 0-127\n" );
		Terminal()->print( "reverse2 [speed]\t- Drive main motor on channel 2 in reverse. Speed 0-127\n" );
		Terminal()->print( "polli2c\t\t\t- Poll all I2C addresses for sensors\n" );
		Terminal()->print( "rangeultra\t\t-Take a range reading from the ultrasonic sensors\n" );
		Terminal()->printImportant( "\n" );
	}
	else if( commandName.compare( "mocstatus" ) == 0 ) {
		m_pMotionManager->printMotorStatus();
		Terminal()->printImportant( "\n" );
	}
	else if( commandName.compare( "polli2c" ) == 0 ) {
		if( !m_pSensorManager->getI2CBus()->pollAllAddress() )
			Terminal()->printImportant( "Command failed\n" );
		Terminal()->printImportant( "\n" );
	}
	else if( commandName.compare( "rangeultra" ) == 0 ) {
		if( !m_pSensorManager->printUltrasonicReadings() )
			Terminal()->printImportant( "Command failed\n" );
		Terminal()->printImportant( "\n" );
	}
	else if( commandName.compare( "forward1" ) == 0 || commandName.compare( "reverse1" ) == 0  || 
				commandName.compare( "forward2" ) == 0 || commandName.compare( "reverse2" ) == 0 || 
				commandName.compare( "forward" ) == 0 || commandName.compare( "reverse" ) == 0 )
	{
		if( tokens.size() < 2 )
			Terminal()->printImportant( "Missing speed argument. See 'help'\n" );
		else
		{
			try
			{
				int speed = std::stoi( tokens[1] );
				if( speed < 0 )
					speed = 0;
				if( speed > 127 )
					speed = 127;
				if( commandName.compare( "forward1" ) == 0 || commandName.compare( "forward" ) == 0 ) {
					Terminal()->print( "Driving motor 1 forward at %d speed...\n", speed );
					if( (errCode = this->m_pMotionManager->getPropController()->forward( RoboClawChannels::CHANNEL1, (int8_t)speed )) != ERR_OK ) {
						Terminal()->printImportant( "Failed to drive motor 1 forward: %s\n", GetErrorString( errCode ) );
					}
				}
				if( commandName.compare( "reverse1" ) == 0 || commandName.compare( "reverse" ) == 0 ) {
					Terminal()->print( "Driving motor 1 in reverse at %d speed...\n", speed );
					if( (errCode = this->m_pMotionManager->getPropController()->reverse( RoboClawChannels::CHANNEL1, (int8_t)speed )) != ERR_OK ) {
						Terminal()->printImportant( "Failed to drive motor 1 in reverse: %s\n", GetErrorString( errCode ) );
					}
				}
				if( commandName.compare( "forward2" ) == 0 || commandName.compare( "forward" ) == 0 ) {
					Terminal()->print( "Driving motor 2 forward at %d speed...\n", speed );
					if( (errCode = this->m_pMotionManager->getDoorController()->forward( RoboClawChannels::CHANNEL2, (int8_t)speed )) != ERR_OK ) {
						Terminal()->printImportant( "Failed to drive motor 2 forward: %s\n", GetErrorString( errCode ) );
					}
				}
				if( commandName.compare( "reverse2" ) == 0 || commandName.compare( "reverse" ) == 0 ) {
					Terminal()->print( "Driving motor 2 in reverse at %d speed...\n", speed );
					if( (errCode = this->m_pMotionManager->getDoorController()->reverse( RoboClawChannels::CHANNEL2, (int8_t)speed )) != ERR_OK ) {
						Terminal()->printImportant( "Failed to drive motor 2 in reverse: %s\n", GetErrorString( errCode ) );
					}
				}
			}
			catch( const std::invalid_argument &e ) {
				Terminal()->printImportant( "Argument was not a valid integer.\n" );
			}
		}
	}
	else {
		Terminal()->printImportant( "Unknown command \'%s\'\n", commandName.c_str() );
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
CConfig* CVehicle::getConfig() {
	assert( m_pConfig );
	return m_pConfig;
}
