#pragma once
#include <thread>
#include <atomic>
#include <mutex>
#include <queue>
#include <iostream>
#include <fstream>
#include <boost/format.hpp>
#include <stack>
#include <algorithm>

/**
* @file terminal.h
* @brief Contains code that handles debug input to the terminal.
* @details This countains the CTerminal class and associated message structures.
*
* @authors Timothy Volpe
*
* @date 1/31/2020
*/

/** Name of log file while running */
#define LOG_FILE_TEMPNAME "lastrun.log"
/** If this is true, every log file is stored with a timestamp. When it is false, only error log files are stored with timestamp. */
#define LOG_STORE_ALL false

/** How often to flush the log to file, in MS */
#define LOG_FLUSH_INTERVAL_MS 3000

/**
* @brief Wraps ofstream.
* @details The thread does not play nice with the ofstream classmember.
*/
struct OutputFile
{
	std::string m_logFilePath;
	std::ofstream m_outputFile;

	/** This class cannot be copied due to ofstream */
	OutputFile( std::string path );
	OutputFile( OutputFile const& )               = delete;
};

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
	std::atomic<bool> m_threadRunning;

	std::mutex m_logQueueLock;
	std::queue<std::string> m_logQueue;
	
	std::mutex m_printMutex;

	std::unique_ptr<OutputFile> m_outputFile;
	std::chrono::steady_clock::time_point m_lastFlush;

	void inputThreadMain();
	
	int m_itemDepth;
	
	std::stack<int> m_nestHeight;
public:
	CTerminal();
	~CTerminal();

	/**
	* @brief Initializes the terminal and starts user input thread.
	* @details This will allow the user to begin input commands to the vehicle from the terminal.
	* @returns Returns #ERR_OK, or an appropriate error code if a failure occured.
	*/
	int init();

	/**
	* @brief Terminates user input thread.
	*/
	void shutdown();

	/**
	* @brief Flush any output to the log file.
	* @detail Clears the log output queues and flushes their contents to the log file.
	*/
	void flushLog();

	/**
	* @brief Handles flushing after a certain period of time has elapsed.
	*/
	void update();

	/**
	* @brief Prints to the console and to the log queue to be saved to file.
	* @details Everything printed with this function will be dumped to a fill periodically, or on program exit/crash.
	*	This will allow failures to be preserved for debugging purposes. Additionally, this function will not interfere
	*	with user input.
	* @param[in]	format		The format string
	* @param[in]	args		The variables to insert into the format string
	* @warning	Using cout instead of this print function is not thread safe.
	*/
	template<typename... Args>
	void print( std::string format, Args... args )
	{
		std::string preamble;
		
		std::unique_lock<std::mutex> lock2( m_printMutex );
		
		if( m_itemDepth <= 0 )
			preamble = "";
		else
			preamble = std::string( m_itemDepth+1, '\t' );
		
		format = preamble += format;
		
		boost::format formatter( format );
		using unroll = int[]; unroll{ 0, (formatter % std::forward<Args>( args ), 0)... };
		std::string outstring = boost::str( formatter );
		
		std::cout << outstring;
		
		// Get number of new line characters
		if( m_itemDepth > 0 ) {
			assert( !m_nestHeight.empty() );
			m_nestHeight.top() += std::count( outstring.begin(), outstring.end(), '\n' );
		}
		lock2.unlock();

		// Thread safe queue push
		std::lock_guard<std::mutex> lock( m_logQueueLock );
		m_logQueue.push( outstring );
	}
	
	/**
	* @brief Same as CTerminal::print, but forces std::cout to flush
	* @details Use this to ensure that a line is printed even if the program crashes before the next flush.
	* @param[in]	format		The format string
	* @param[in]	args		The variables to insert into the format string
	*/
	template<typename... Args>
	void printImportant( std::string format, Args... args )
	{
		this->print( format, args... );
		std::lock_guard<std::mutex> lock( m_printMutex );
		std::cout << std::flush;
	}
	
	/**
	* @brief Signals the start of a process, which can return an error or success
	* @details Used for to format for readability. Can be nested.
	*/
	template<typename... Args>
	void startItem( std::string format, Args... args )
	{
		int tempDepth;
		
		std::string prefix( " [         ] " + std::string( m_itemDepth, '\t' ) );
		
		tempDepth = m_itemDepth;
		m_itemDepth = 0;
		this->printImportant( prefix + format + "...\n", args... );
		std::lock_guard<std::mutex> lock( m_printMutex );
		m_nestHeight.push( 1 );
		m_itemDepth = tempDepth+1;
	}
	
	/**
	* @brief Signals the end of a process
	* @param[in]	success		If true, the process ended successfully. If false, it did not, and an error is to be reported.
	*/
	void finishItem( bool success );
};
