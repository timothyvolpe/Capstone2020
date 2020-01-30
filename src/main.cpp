/**
* @file main.cpp
* @brief Program entry point.
*
* @authors Timothy Volpe
*
* @date 1/29/2020
*/

#include <iostream>
#include "vehicle.h"

int main( int argc, char *argv[] )
{
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
	if( !CVehicle::instance().initialize() ) {
		std::cout << "Vehicle initialization: FAILED\n";
		return -1;
	}
	std::cout << "Vehicle initialization: SUCCESS\n";

	// Vehicle class should now be safe for use
}