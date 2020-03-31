#pragma once
#include <boost/property_tree/ptree.hpp>

/**
* @file config.h
* @brief Contains code for loading of user configs.
* @details The CConfig class is used to load user settings at startup.
*
* @authors Timothy Volpe
*
* @date 3/31/2020
*/

/** In the working directory */
#define CONFIG_FILE "config.ini"

/**
* @brief This class loads the user config settings from file on startup.
* @details One of the constraints of this class is that it only loads from file on startup, and does not update the file.
* The only thing it will write to the file is missing settings, which will have their default values stored. This way the user
* can delete settings to restore them to their default values.
* 
* The config file cannot be changing during operation, as it will have no effect.
*/
class CConfig
{
private:
	/**
	* So we dont have to include vehicle.h to get Terminal() in the header
	*/
	static void printError( std::string path, std::string what );

	/**
	* @brief This extracts the settings, replaces missing ones, and even creates the file if it does not exist
	*/
	bool extractSettings( boost::property_tree::ptree &pt, std::string path );
	
	/**
	* @brief This gets a setting from the ptree, and creates it if its missing.
	*/
	template <class T>
	T getSetting( boost::property_tree::ptree &pt, std::string path, T defaultValue )
	{
		try {
			boost::optional<boost::property_tree::ptree& > child = pt.get_child_optional( path );
			if( !child ) {
				pt.put( path, defaultValue );
				return defaultValue;
			}
			else
				return pt.get<T>( path );
		}
		catch( const boost::property_tree::ptree_error &e ) {
			CConfig::printError( path, e.what() );
			return defaultValue;
		}
	}
	
	uint8_t		m_uiMotorPropAddress;
	uint8_t		m_uiMotorDoorAddress;
	
	float		m_fMotorMainBatVoltMin;
	float		m_fMotorMainBatVoltMax;
	float		m_fMotorLogicBatVoltMin;
	float		m_fMotorLogicBatVoltMax;
	
	float		m_fMotorMaxTemp1;
	float		m_fMotorMaxTemp2;
	
	float		m_fMotorPropMaxCurrent1;
	float		m_fMotorPropMaxCurrent2;
	float		m_fMotorDoorMaxCurrent1;
	float		m_fMotorDoorMaxCurrent2;
public:
	CConfig();
	~CConfig();
	
	/**
	* @brief This loads the config file and stores the config settings.
	* @details The config file is defined by #CONFIG_FILE, relative to the executable directory.
	* Any missing settings will be restored to their default value, and rewritten to the file. If there is a syntax error
	* in the file, or the file cannot be read for any reason, this operation will fail, and some settings may not be loaded.
	* It is advised to terminate if this occurs.
	* @returns Returns #ERR_OK on success, or an appropriate error code on failure.
	*/
	int load();
	
	inline const uint8_t getMotorPropAddress() { return m_uiMotorPropAddress; }
	inline const uint8_t getMotorDoorAddress() { return m_uiMotorDoorAddress; }
	
	inline const float getMotorMainBatVoltMin() { return m_fMotorMainBatVoltMin; }
	inline const float getMotorMainBatVoltMax() { return m_fMotorMainBatVoltMax; }
	
	inline const float getMotorLogicBatVoltMin() { return m_fMotorLogicBatVoltMin; }
	inline const float getMotorLogicBatVoltMax() { return m_fMotorLogicBatVoltMax; }
	
	inline const float getMotorMaxTemp1() { return m_fMotorMaxTemp1; }
	inline const float getMotorMaxTemp2() { return m_fMotorMaxTemp2; }
	
	inline const float getMotorPropMaxCurrent1() { return m_fMotorPropMaxCurrent1; }
	inline const float getMotorPropMaxCurrent2() { return m_fMotorPropMaxCurrent2; }
	inline const float getMotorDoorMaxCurrent1() { return m_fMotorDoorMaxCurrent1; }
	inline const float getMotorDoorMaxCurrent2() { return m_fMotorDoorMaxCurrent2; }
};
