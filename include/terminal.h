#pragma once
#include <thread>

/**
* @file terminal.h
* @brief Contains code that handles debug input to the terminal.
* @details This countains the CTerminal class and associated message structures.
*
* @authors Timothy Volpe
*
* @date 1/31/2020
*/

/**
* @brief Class for handling terminal input.
* @details This class spawns a seperate thread that handles terminal input. When a message is received,
*	the class will send a message to the main thread with the user input command contents.
*
* @author Timothy Volpe
* @date 1/31/2020
*/
class CTerminal
{
private:
	std::thread m_inputThread;

	void inputThreadMain();
public:
	CTerminal();
	~CTerminal();

	/**
	* @brief Initializes the terminal and starts user input thread.
	* @details This will allow the user to begin input commands to the vehicle from the terminal.
	* @returns Returns #ERR_OK, or an appropriate error code if a failure occured.
	*/
	int init();
};