#include <unistd.h>
#include <string>
#include <libgen.h>
#include <vector>
#include <linux/limits.h>
#include <iostream>
#include "config.h"
#include "def.h"
#include "vehicle.h"

#include <boost/property_tree/ini_parser.hpp>

CConfig::CConfig() {
}
CConfig::~CConfig() {
}

void CConfig::printError( std::string path, std::string what ) {
	Terminal()->printImportant( "Failed read \'%s\': %s, using default\n", path.c_str(), what.c_str() );
}

bool CConfig::extractSettings( boost::property_tree::ptree &pt, std::string path )
{
	// Get config settings
	// This is where defaults are defined as well
	
	m_uiMotorPropAddress = (unsigned char)this->getSetting<int>( pt, "motors.props_address", 0x82 );
	m_uiMotorDoorAddress = (unsigned char)this->getSetting<int>( pt, "motors.doors_address", 0x80 );
	
	m_fMotorMainBatVoltMin = this->getSetting<float>( pt, "motors.main_battery_voltage_min", 11.5f );
	m_fMotorMainBatVoltMax = this->getSetting<float>( pt, "motors.main_battery_voltage_max", 15.0f );
	m_fMotorLogicBatVoltMin = this->getSetting<float>( pt, "motors.logic_battery_voltage_min", 6.0f );
	m_fMotorLogicBatVoltMax = this->getSetting<float>( pt, "motors.logic_battery_voltage_max", 15.0f );
	
	m_fMotorMaxTemp1 = this->getSetting<float>( pt, "motors.max_temp1", 80.0f ); // C, spec'd max is 80
	m_fMotorMaxTemp2 = this->getSetting<float>( pt, "motors.max_temp2", 80.0f ); // C, spec'd max is 80
	
	m_fMotorPropMaxCurrent1 = this->getSetting<float>( pt, "motors.max_prop_current1", 50 ); // A, max motor current
	m_fMotorPropMaxCurrent2 = this->getSetting<float>( pt, "motors.max_prop_current2", 50 ); // A, max motor current
	m_fMotorDoorMaxCurrent1 = this->getSetting<float>( pt, "motors.max_door_current1", 15 ); // A, max controller
	m_fMotorDoorMaxCurrent2 = this->getSetting<float>( pt, "motors.max_door_current1", 15 ); // A, max controller
	
	// Attempt to create and write
	std::ofstream createFile( path );
	if( !createFile.is_open() ) {
		Terminal()->printImportant( "Failed to create file %s\n", path.c_str() );
		return false;
	}
	createFile.close();
	try {
		boost::property_tree::ini_parser::write_ini( path, pt ); 
	}
	catch( const boost::property_tree::ini_parser::ini_parser_error &e ) {
		Terminal()->printImportant( "Failed to fix file %s\n", path.c_str() );
		Terminal()->printImportant( "%s\n", e.what() );
		return false;
	}
	
	return true;
}

int CConfig::load()
{
	std::string configDir;
	std::vector<char> pathBuf;
	size_t charsCopied;
	
	// Attempt to load the config file
	charsCopied = 0;
	do {
		pathBuf.resize( pathBuf.size()+PATH_MAX );
		charsCopied = ::readlink( "/proc/self/exe", &pathBuf.at( 0 ), pathBuf.size() );
		if( charsCopied == -1 ) {
			pathBuf.clear();
			break;
		}
	} while( charsCopied >= pathBuf.size() );
	
	if( !pathBuf.empty() ) {
		configDir = std::string( pathBuf.begin(), pathBuf.end() );
		configDir = dirname( const_cast<char*>( configDir.c_str() ) );
		configDir += "/";
		configDir += CONFIG_FILE;
	}
	else
		return ERR_CONFIG_FILE;
		
	// Try to parse
	boost::property_tree::ptree pt;
	try {
		boost::property_tree::ini_parser::read_ini( configDir, pt );
	}
	catch( const boost::property_tree::ini_parser::ini_parser_error &e ) {
		Terminal()->print( "Missing or damaged config file, attempting to fix %s...\n", CONFIG_FILE );
		Terminal()->print( "%s\n", e.what() );
	}
	  
	// Create/fix/load config file
	if( !this->extractSettings( pt, configDir ) )
		return ERR_CONFIG_FILE;
	
	return ERR_OK;
}
