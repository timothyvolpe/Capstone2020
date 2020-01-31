#include "def.h"

const char* GetErrorString( int errCode )
{
	for( size_t i = 0; i < ERRMSG_TABLE_LEN; i++ ) {
		if( ErrorMessageTable[i].code == errCode )
			return ErrorMessageTable[i].message;
	}
	return "INVALID ERROR CODE";
}