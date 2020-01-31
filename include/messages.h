#pragma once

/**
* @file messages.h
* @brief Contains message struct definitions and message IDs
* @details All message structs to be exposed to the entire message loop should be defined here.
*
* @authors Timothy Volpe
*
* @date 1/31/2020
*/

#define MSGID_UNKNOWN		0
#define MSGID_QUIT			1

/**
* @brief The base message struct class
* @warning Messages marked as important but with a non-zero timeout will still be deleted once they expire.
*/
struct message_t
{
	message_t() {}
	message_t( unsigned int id, bool imprtnt, unsigned int timeoutInMS ) : message_id( id ),
		important( imprtnt ), timeoutMS( timeoutInMS ) {}
	message_t( const message_t& cpy ) {
		message_id = cpy.message_id;
		important = cpy.important;
		timeoutMS = cpy.timeoutMS;
	}

	/** The message ID used to identify the type of message. */
	unsigned int	message_id;
	/** The important. Non-important messages are not guaranteed to be executed. */
	bool			important;
	/** The lifetime or timeout, after which the message will be deleted. Set to zero for infinite. */
	unsigned int	timeoutMS;
};

struct terminal_msg_t : public message_t
{
	terminal_msg_t() {}
	terminal_msg_t( unsigned int id, bool imprtnt, unsigned int timeoutInMS ) : message_t( id, imprtnt, timeoutInMS ) {}
	terminal_msg_t( const terminal_msg_t& cpy ) : message_t(cpy) {
		commandName = cpy.commandName;
	}

	std::string		commandName;
};