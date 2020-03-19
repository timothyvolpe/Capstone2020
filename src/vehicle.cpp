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

CVehicle::CVehicle() {
	m_pVehicleTerminal = 0;
	m_pSensorManager = 0;
	m_isRunning = false;

	m_pI2cBus = 0;
	m_pMotorControllerChannel = 0;

	m_pMotorControllerLarge = 0;
	m_pMotorControllerSmall = 0;
	
	m_lastMotorUpdate = std::chrono::steady_clock::now(); 
}
CVehicle::~CVehicle()
{
}

int CVehicle::setupMotors()
{
	int errCode;
	
	if( (errCode = m_pMotorControllerLarge->setLogicVoltageLevels( ROBOCLAW_LOGIC_MIN, ROBOCLAW_LOGIC_MAX )) != ERR_OK ) {
		Terminal()->printImportant( "Failed to set logic level limits\n" );
		return errCode;
	}
	if( (errCode = m_pMotorControllerLarge->setMainVoltageLevels( ROBOCLAW_BATTERY_MIN, ROBOCLAW_BATTERY_MAX )) != ERR_OK ) {
		Terminal()->printImportant( "Failed to set logic level limits\n" );
		return errCode;
	}
	
	return ERR_OK;
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

	// Open i2c comm bus
	Terminal()->startItem( "Setting up I2C bus" );
	m_pI2cBus = new CI2CBus( "SensorI2C" );
	errCode = m_pI2cBus->open( "/dev/i2c-1" );
	if( errCode != ERR_OK ) {
		Terminal()->finishItem( false );
		return errCode;
	}
	Terminal()->finishItem( true );

	// Set uart commm channel
	Terminal()->startItem( "Setting up motor UART" );
	m_pMotorControllerChannel = new CUARTChannel( "MotorUART" );
	errCode = m_pMotorControllerChannel->open( "/dev/serial0", true, false, false, false );
	if( errCode != ERR_OK ) {
		Terminal()->finishItem( false );
		return errCode;
	}
	errCode = m_pMotorControllerChannel->setBaudRate( B460800 );
	errCode = m_pMotorControllerChannel->setiFlag( IGNBRK );
	errCode = m_pMotorControllerChannel->setoFlag( 0 );
	errCode = m_pMotorControllerChannel->setReadTimeout( 0, 50 );
	if( errCode != ERR_OK ) {
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
	Terminal()->startItem( "Initializing motor controllers" );
	
	// Large controller
	m_pMotorControllerLarge = new CMotorController( m_pMotorControllerChannel, ROBOCLAW_60A_ADDRESS );
	Terminal()->startItem( "Large motor controller initialization" );
	errCode = m_pMotorControllerLarge->init();
	if( errCode != ERR_OK ) {
		Terminal()->finishItem( false );
		return errCode;
	}
	Terminal()->finishItem( true );
	
	// Small controller
	m_pMotorControllerSmall = new CMotorController( m_pMotorControllerChannel, ROBOCLAW_30A_ADDRESS);
	Terminal()->startItem( "Small motor controller initialization" );
	errCode = m_pMotorControllerSmall->init();
	if( errCode != ERR_OK ) {
		Terminal()->finishItem( false );
		return errCode;
	}
	Terminal()->finishItem( true );
	Terminal()->finishItem( true ); // motor controllers
	
	// Set up motor controllers
	Terminal()->startItem( "Configuring motors" );
	errCode = this->setupMotors();
	if( errCode != ERR_OK ) {
		Terminal()->finishItem( false );
		return errCode;
	}
	Terminal()->finishItem( true );

	return ERR_OK;
}
void CVehicle::shutdown()
{	
	if( m_pMotorControllerSmall ) {
		m_pMotorControllerSmall->shutdown();
		delete m_pMotorControllerSmall;
		m_pMotorControllerSmall = 0;
	}
	if( m_pMotorControllerLarge ) {
		m_pMotorControllerLarge->shutdown();
		delete m_pMotorControllerLarge;
		m_pMotorControllerLarge = 0;
	}
	if( m_pSensorManager ) {
		delete m_pSensorManager;
		m_pSensorManager = 0;
	}
	if( m_pMotorControllerChannel ) {
		m_pMotorControllerChannel->close();
		delete m_pMotorControllerChannel;
		m_pMotorControllerChannel = 0;
	}
	if( m_pI2cBus ) {
		m_pI2cBus->close();
		delete m_pI2cBus;
		m_pI2cBus = 0;
	}
	if( m_pVehicleTerminal ) {
		m_pVehicleTerminal->shutdown();
		delete m_pVehicleTerminal;
		m_pVehicleTerminal = 0;
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

void CVehicle::showMotorControllerStatus()
{
	int errCode;
	std::string version;
	uint16_t motorStatus;
	float temp1, temp2;
	uint16_t motorConfig;
	float mainVoltage;
	float logicVoltage;
	float mainMin, mainMax;
	float logicMin, logicMax;
	float current1, current2;
	float duty1, duty2;
	
	if( (errCode = m_pMotorControllerLarge->getControllerInfo( version )) != ERR_OK ) {
		Terminal()->printImportant( "Failed to get controller info: %s\n", GetErrorString( errCode ) ); 
	}
	else
		Terminal()->print( "Controller Version:\t%s\n", version.c_str() );
	
	if( (errCode = m_pMotorControllerLarge->getControllerStatus( &motorStatus )) != ERR_OK ) {
		Terminal()->printImportant( "Failed to get motor controller status: %s\n", GetErrorString( errCode ) );
	}
	else
		Terminal()->print( "Controller Status:\t%04X\n", motorStatus );
		
	if( (errCode = m_pMotorControllerLarge->getTemperature( &temp1, &temp2 )) != ERR_OK ) {
		Terminal()->printImportant( "Failed to get motor controller temp: %s\n", GetErrorString( errCode ) );
	}
	else {
		Terminal()->print( "Temperature 1:\t\t%.2f C\n", temp1 ); 
		Terminal()->print( "Temperature 2:\t\t%.2f C\n", temp2 ); 
	}

	if( (errCode = m_pMotorControllerLarge->getConfigSettings( &motorConfig )) != ERR_OK ) {
		Terminal()->printImportant( "Failed to get motor controller config: %s\n", GetErrorString( errCode ) );
	}
	else
		Terminal()->print( "Standard Config:\t%04X\n", motorConfig ); 
		
	if( (errCode = m_pMotorControllerLarge->getMainBatteryVoltage( &mainVoltage )) != ERR_OK ) {
		Terminal()->printImportant( "Failed to get main battery voltage: %s\n", GetErrorString( errCode ) );
	}
	else
		Terminal()->print( "Battery Voltage:\t%.1f V\n", mainVoltage ); 
		
	if( (errCode = m_pMotorControllerLarge->getLogicBatteryVoltage( &logicVoltage )) != ERR_OK ) {
		Terminal()->printImportant( "Failed to get logic battery voltage: %s\n", GetErrorString( errCode ) );
	}
	else
		Terminal()->print( "Logic Battery Voltage:\t%.1f V\n", logicVoltage ); 
	
	if( (errCode = m_pMotorControllerLarge->getMainVoltageLevels( &mainMin, &mainMax )) != ERR_OK ) {
		Terminal()->printImportant( "Failed to get main battery voltage levels: %s\n", GetErrorString( errCode ) );
	}
	else {
		Terminal()->print( "Main Voltage Limits:\t%.1f V - %.1f V\n", mainMin, mainMax ); 
	}
		
	if( (errCode = m_pMotorControllerLarge->getLogicVoltageLevels( &logicMin, &logicMax )) != ERR_OK ) {
		Terminal()->printImportant( "Failed to get logic battery voltage levels: %s\n", GetErrorString( errCode ) );
	}
	else {
		Terminal()->print( "Logic Voltage Limits:\t%.1f V - %.1f V\n", logicMin, logicMax ); 
	}
		
	if( (errCode = m_pMotorControllerLarge->getMotorCurrents( &current1, &current2 )) != ERR_OK ) {
		Terminal()->printImportant( "Failed to get motor currents: %s\n", GetErrorString( errCode ) );
	}
	else {
		Terminal()->print( "Motor Current 1:\t%.3f A\n", current1 ); 
		Terminal()->print( "Motor Current 2:\t%.3f A\n", current2 ); 
	}
	
	if( (errCode = m_pMotorControllerLarge->getMotorDutyCycles( &duty1, &duty2 )) != ERR_OK ) {
		Terminal()->printImportant( "Failed to get motor duty cycles: %s\n", GetErrorString( errCode ) );
	}
	else {
		Terminal()->print( "Motor Duty Cycle 1:\t%.1f%%\n", duty1 ); 
		Terminal()->print( "Motor Duty Cycle 2:\t%.1f%%\n", duty2 ); 
	}
}

int CVehicle::update()
{
	std::chrono::steady_clock::time_point curtime = std::chrono::steady_clock::now();
	int errCode;
	
	// Check comm threads for errors
	errCode = m_pI2cBus->getThreadError();
	if( errCode != ERR_OK ) {
		Terminal()->printImportant( "ERROR: There was a failure in the %s thread\n", m_pI2cBus->getPortName().c_str() );
		return errCode;
	}
	errCode = m_pMotorControllerChannel->getThreadError();
	if( errCode != ERR_OK ) {
		Terminal()->printImportant( "ERROR: There was a failure in the %s thread\n", m_pMotorControllerChannel->getPortName().c_str() );
		return errCode;
	}
	
	// Update motor if necessary
	if( std::chrono::duration_cast<std::chrono::milliseconds>(curtime - m_lastMotorUpdate).count() > ((1/MOTOR_UPDATE_FREQUENCY)*1000.0) )
	{
		// Do stuff...
		
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
		Terminal()->printImportant( "\n" );
	}
	else if( commandName.compare( "mocstatus" ) == 0 )
	{
		Terminal()->printImportant( "\nMotor Controller Status:\n" );
		this->showMotorControllerStatus();
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
					if( (errCode = this->m_pMotorControllerLarge->forward( RoboClawChannels::CHANNEL1, (int8_t)speed )) != ERR_OK ) {
						Terminal()->printImportant( "Failed to drive motor 1 forward: %s\n", GetErrorString( errCode ) );
					}
				}
				if( commandName.compare( "reverse1" ) == 0 || commandName.compare( "reverse" ) == 0 ) {
					Terminal()->print( "Driving motor 1 in reverse at %d speed...\n", speed );
					if( (errCode = this->m_pMotorControllerLarge->reverse( RoboClawChannels::CHANNEL1, (int8_t)speed )) != ERR_OK ) {
						Terminal()->printImportant( "Failed to drive motor 1 in reverse: %s\n", GetErrorString( errCode ) );
					}
				}
				if( commandName.compare( "forward2" ) == 0 || commandName.compare( "forward" ) == 0 ) {
					Terminal()->print( "Driving motor 2 forward at %d speed...\n", speed );
					if( (errCode = this->m_pMotorControllerLarge->forward( RoboClawChannels::CHANNEL2, (int8_t)speed )) != ERR_OK ) {
						Terminal()->printImportant( "Failed to drive motor 2 forward: %s\n", GetErrorString( errCode ) );
					}
				}
				if( commandName.compare( "reverse2" ) == 0 || commandName.compare( "reverse" ) == 0 ) {
					Terminal()->print( "Driving motor 2 in reverse at %d speed...\n", speed );
					if( (errCode = this->m_pMotorControllerLarge->reverse( RoboClawChannels::CHANNEL2, (int8_t)speed )) != ERR_OK ) {
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
