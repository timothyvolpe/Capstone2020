/**
* @file vehicle.h
* @brief Contains the CVehicle class, serves as a "main" class
*
* @authors Timothy Volpe
*
* @date 1/29/2020
*/

class CSensorManager;

/**
* @brief Main vehicle class singleton.
* @details This class controls the entire operation of the vehicle. It is created when the program
*	starts, and is initialized in main. Only one can be created.
*
* @author Timothy Volpe
* @date 1/29/2020
*/
class CVehicle
{
private:
	CSensorManager* m_pSensorManager;
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
	* @returns Returns true if initialize was successful, and false if it failed. Program should exit on failure. An error message
	*	will be printed to the console.
	*/
	bool initialize();
};