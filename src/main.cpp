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
	int errCode = ERR_OK;

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
	// We can use Terminal().print(...) AFTER this point
	errCode = LocalVehicle().initialize();
	// We can now use Terminal().print(...)
	if( errCode != ERR_OK ) {
		Terminal()->print( "Vehicle initialization: FAILED\n" );
		Terminal()->print( "ERROR: %s\n", GetErrorString( errCode ) );
	}
	else
	{
		Terminal()->print( "Vehicle initialization: SUCCESS\n" );

		// Vehicle class should now be safe for use

		Terminal()->print( "\n" );

		// Start program loop
		Terminal()->print( "Entering program loop...\n" );
		errCode = LocalVehicle().start();
		if( errCode != ERR_OK )
			Terminal()->print( "ERROR: %s\n", GetErrorString( errCode ) );
	}

	// Clean up for exit
	LocalVehicle().shutdown();

	return errCode;
}