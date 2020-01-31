/**
* @file main.cpp
* @brief Program entry point.
*
* @authors Timothy Volpe
*
* @date 1/29/2020
*/

// Notes

// Main thread handles message loop
// Child thread for console input

#include <iostream>
#include "vehicle.h"
#include "def.h"

/** @brief Program entry point. */
int main( int argc, char *argv[] )
{
	int errCode;

	std::cout << "  +========================================================================+\n";
	std::cout << "  == Welcome to the ENEL Green Power Canal Vehicle Controller Application ==\n";
	std::cout << "  +========================================================================+\n";
	std::cout << "\n";
	std::cout << "   This application is meant to run on a Raspberry Pi 4 64-bit device.\n";
	std::cout << "   It handles all of the sensor data and motion control of the canal\n"
				 "   vehicle.\n";
	std::cout << "\n";
	std::cout << "   Developed as part of a University of Massachusetts, Lowell Capstone\n"
				 "   Project.\n";
	std::cout << "\n";
	std::cout << "   Contributors:\n";
	std::cout << "      19-309 ENEL Green Power 1 - Timothy Volpe\n";
	std::cout << "\n";
	std::cout << "  +========================================================================+\n";
	std::cout << "\n";
	std::cout << "Starting...\n";

	// !! Do this first !!
	std::cout << "Initializing vehicle...\n";
	errCode = CVehicle::instance().initialize();
	if( errCode != ERR_OK ) {
		std::cout << "Vehicle initialization: FAILED\n";
		std::cout << "ERROR: " << GetErrorString( errCode ) << "\n";
		return errCode;
	}
	std::cout << "Vehicle initialization: SUCCESS\n";

	// Vehicle class should now be safe for use

	std::cout << "\n";

	// Start program loop
	std::cout << "Entering program loop...\n";
	errCode = CVehicle::instance().start();
	if( errCode != ERR_OK ) {
		std::cout << "ERROR: " << GetErrorString( errCode ) << "\n";
		return errCode;
	}

	std::cout << "\n";

	return ERR_OK;
}