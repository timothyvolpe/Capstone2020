#pragma once
#include <mutex>
#include <queue>
#include "terminal.h"

/**
* @file vehicle.h
* @brief Contains the CVehicle class, serves as a "main" class
*
* @authors Timothy Volpe
*
* @date 1/29/2020
*/

#define ROBOCLAW_60A_ADDRESS 0x80
#define ROBOCLAW_30A_ADDRESS 0x80

class CSensorManager;
class CI2CBus;
class CUARTChannel;
class CMotorController;

struct message_t;

/** use to reference the main vehicle class */
#define LocalVehicle() CVehicle::instance()
/** used to reference the terminal */
#define Terminal() CVehicle::instance().getTerminal()

/**
* @brief Main vehicle class singleton.
* @details This class controls the entire operation of the vehicle. It is created when the program
*	starts, and is initialized in main. Only one can be created.
*
*	The CVehicle class also handles the message loop which constantly pools for messaages
*	from child classes and processes. This allows sensor data to be collected in a non-blocking
*	manner. 
*
* @author Timothy Volpe
* @date 1/29/2020
*/
class CVehicle
{
private:
	CSensorManager* m_pSensorManager;
	CTerminal *m_pVehicleTerminal;

	bool m_isRunning;

	std::mutex m_msgLoopMutex;
	std::queue<std::unique_ptr<message_t>> m_messageQueue;

	/** Parse a terminal command input from the user. */
	void parseCommandMessage( std::unique_ptr<message_t> pCommandMsg );

	// Communication lines
	CI2CBus *m_pI2cBus;
	CUARTChannel *m_pMotorControllerChannel;

	CMotorController *m_pMotorControllerLarge;
	CMotorController *m_pMotorControllerSmall;
public:
	/**
	* @brief Returns instance of vehicle singleton.
	* @details This can be called anywhere to get the vehicle main parent object.
	*	The class should be initialized in main.
	*/
	static CVehicle& instance() {
		static CVehicle vehicleInstance; 
		return vehicleInstance;
	}

	/** 
	* @brief Default constructor. 
	* @warning Do not do anything here but initialize variables to 0. When this class is initialized is undefined.
	*/
	CVehicle();
	/** Default destructor, class on program exit! */
	~CVehicle();

	/** This class cannot be copied. */
	CVehicle( CVehicle const& )               = delete;
	/** This class cannot be assigned. */
	void operator=( CVehicle const& )  = delete;

	/**
	* @brief Initialize the vehicle singleton.
	* @details This should be called immediately in main, before any other program operation is allowed.
	*	This is to prevent accidently calls to #instance() before initialize is called.
	* @returns Returns #ERR_OK, or an appropriate error code if a failure occured. This should exit the program.
	*/
	int initialize();

	/**
	* @brief Shutdown everything and clean up data used
	* @details Use this instead of destructor to avoid ambiguous debug errors after main returns.
	*/
	void shutdown();

	/**
	* @brief Starts the message loop.
	* @details This will begin collecting and acting on messages received from vehicle sensors, helper processes, etc.
	* @warning This method is blocking, and will not exit until the vehicle operate ceases.
	* @returns Returns #ERR_OK, or an appropriate error code if a failure occured. This should exit the program.
	*/
	int start();

	/**
	* @brief Post a message to the message loop.
	* @details This function is designed to be thread safe. It will post a message to the vehicle thread loop.
	*	There is no guarantee for when the message will be executed, as it will be added to the end of the queue. 
	*	Messages marked as important will be guaranteed to be executed. Mesages with a timeout will be deleted after
	*	the timeout has expired, and messages that are not marked as important are not guaranteed to be executed.
	* @param[in]	pMsg	A unique pointer to the message data. The vehicle object will take ownership of this data.
	*/
	void postMessage( std::unique_ptr<message_t> pMsg );

	/**
	* @brief Returns the terminal class object.
	* @returns Pointer to terminal class object.
	*/
	CTerminal* getTerminal();
};
