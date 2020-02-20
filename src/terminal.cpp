#include <chrono>
#include <string>
#include <locale>
#include <codecvt>
#include "def.h"
#include "vehicle.h"
#include "terminal.h"
#include "messages.h"
#ifdef _WIN32
#include <Windows.h>
#elif __linux__
#include <libgen.h>
#include <linux/limits.h>
#endif

OutputFile::OutputFile( std::string path ) : m_outputFile( path.c_str(), std::ios::out | std::ios::trunc ) {
	m_logFilePath = path;
}

CTerminal::CTerminal() {
}
CTerminal::~CTerminal()
{
	// Ensure this happens
	if( m_outputFile.get()->m_outputFile.good() && m_outputFile.get()->m_outputFile.is_open() ) {
		this->flushLog();
		m_outputFile.get()->m_outputFile.close();
		m_outputFile.reset();
	}
}

int CTerminal::init()
{
	std::string executableDir;

	// Get executable path, implementation specific
#ifdef _WIN32
	std::vector<wchar_t> pathBuf;
	std::wstring widePath;
	std::wstring_convert<std::codecvt_utf8<wchar_t>, wchar_t> converter;
	size_t copied = 0;
	do {
		pathBuf.resize( pathBuf.size()+MAX_PATH );
		copied = ::GetModuleFileName( 0, &pathBuf.at( 0 ), (DWORD)pathBuf.size() );
	} while( copied >= pathBuf.size() );

	widePath = std::wstring( pathBuf.begin(), pathBuf.end() );
	if( widePath.find( L"\\" )  != std::string::npos )
		widePath = widePath.substr( 0, widePath.find_last_of( L"\\" ) );
	widePath += L"\\";
	executableDir = converter.to_bytes( widePath );
#elif __linux__
	std::vector<char> pathBuf;
	size_t copied = 0;
	do {
		pathBuf.resize( pathBuf.size()+PATH_MAX );
		copied = ::readlink( "/proc/self/exe", &pathBuf.at( 0 ), pathBuf.size() );
		if( copied == -1 ) {
			pathBuf.clear();
			break;
		}
	} while( copied >= pathBuf.size() );
	if( !pathBuf.empty() ) {
		executableDir = std::string( pathBuf.begin(), pathBuf.end() );
		executableDir = dirname( const_cast<char*>( executableDir.c_str() ) );
		executableDir += "/";
	}
	else
		return ERR_TERMINAL_EXEC_DIR;
#endif

	// Open the log file
	m_outputFile = std::make_unique<OutputFile>( executableDir + LOG_FILE_TEMPNAME );
	if( !m_outputFile.get()->m_outputFile.good() || !m_outputFile.get()->m_outputFile.is_open() ) {
		this->print( "\t\tLog file path: %s\n", m_outputFile.get()->m_logFilePath.c_str() );
		return ERR_TERMINAL_OUTPUT_FILE;
	}
	m_lastFlush = std::chrono::steady_clock::now();

	// Create and start user input thread
	m_threadRunning = true;
	m_inputThread = std::thread( &CTerminal::inputThreadMain, this );

	return ERR_OK;
}
void CTerminal::shutdown()
{
	if( m_threadRunning ) {
		m_threadRunning = false;
		// Because we can't interrupt cin
		std::cout << "\nPress enter to exit...\n";
	}
	m_inputThread.join();
}

void CTerminal::flushLog()
{
	std::queue<std::string> queueCopy;

	// Copy queue without interrupting output too much
	std::unique_lock<std::mutex> lock( m_logQueueLock );
	queueCopy = m_logQueue;
	m_logQueue = std::queue<std::string>();
	lock.unlock();

	// Write to file
	while( !queueCopy.empty() )
	{
		std::string logEntry = queueCopy.front();
		queueCopy.pop();

		if( m_outputFile.get()->m_outputFile.good() )
			m_outputFile.get()->m_outputFile.write( logEntry.c_str(), logEntry.size() );
		else {
			this->print( "Log file stream is corrupted!\n" );
			break;
		}
	}

	m_lastFlush = std::chrono::steady_clock::now();
}

void CTerminal::update()
{
	// Flush if necessary
	if( !m_logQueue.empty() ) {
		if( std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - m_lastFlush).count() > LOG_FLUSH_INTERVAL_MS )
			this->flushLog();
	}	
}

void CTerminal::inputThreadMain()
{
	DebugMessage( "TERMINAL THREAD ENTER\n" );
	
	while( m_threadRunning )
	{
		std::string userInput;

		// Need platform-specific fix to prevent interleaving characters
		std::getline( std::cin, userInput );
		if( !userInput.empty() )
		{
			// Send a message
			terminal_msg_t termMsg( MSGID_TERMINAL_MSG, true, 0 );
			termMsg.command = userInput;
			LocalVehicle().postMessage( std::make_unique<terminal_msg_t>( termMsg ) );
		}
	}
	
	DebugMessage( "TERMINAL THREAD EXIT\n" );
}
