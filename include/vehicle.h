#pragma once
#include <mutex>
#include <queue>

/**
* @file vehicle.h
* @brief Contains the CVehicle class, serves as a "main" class
*
* @authors Timothy Volpe
*
* @date 1/29/2020
*/

class CTerminal;
class CSensorManager;

struct message_t;

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
	* @warning pMsg will be copied, therefore the argument data will not be owned by the CVehicle class. It must be 
	*	deleted by the calling method.
	* @param[in]	pMsg	The message data to be posted. This will be copied into the message queue.
	*/
	void postMessage( message_t &pMsg );
};