#pragma once

/**
* @file motor.h
* @brief Contains code that handles the interfacing with the motor controllers.
*
* @authors Timothy Volpe
* @date 2/19/2020
*/

class CUARTChannel;

/**
* @brief RoboClaw Basicmicro motor controller interface.
* @details This class can communicate via UART with a RoboClaw motor controller.
*
* @author Timothy Volpe
* @date 2/19/2020
*/
class CMotorController
{
private:
	/** Do not delete this, it should be a reference to one owned by CVehicle. */
	CUARTChannel *m_pMotorUARTReference;
public:
	CMotorController( CUARTChannel *pUART );
	~CMotorController();

	int init();
	void shutdown();
};