#pragma once

#include "egPrerequisites.h"
#include <string>
#include <queue>
#include <cstdarg>
#include "utTimer.h"


namespace Horde3D {

// =================================================================================================
// Engine Config
// =================================================================================================

struct LogMessage
{
	std::string  text;
	int          level;
	float        time;

	LogMessage()
	{
	}

	LogMessage( const std::string &text, int level, float time ) :
		text( text ), level( level ), time( time )
	{
	}
};

// =================================================================================================
typedef void (*EngineLogProcessingFunc)( int level, const char *msg );

class EngineLog
{
protected:
	
	Timer                     _timer;
	char                      _textBuf[2048];
	uint32                    _maxNumMessages;
	std::queue< LogMessage >  _messages;

	void pushMessage( const std::string &text, uint32 level );
	void pushMessage( int level, const char *msg, va_list ap );
	
	EngineLogProcessingFunc _customLogHandler;

public:
	
	EngineLog();

	void writeError( const char *msg, ... );
	void writeWarning( const char *msg, ... );
	void writeInfo( const char *msg, ... );
	void writeDebugInfo( const char *msg, ... );

	bool getMessage( LogMessage &msg );

	uint32 getMaxNumMessages() { return _maxNumMessages; }
	void setMaxNumMessages( uint32 maxNumMessages ) { _maxNumMessages = maxNumMessages; }

	void setCustomLogHandler( EngineLogProcessingFunc func ) { _customLogHandler = func; }
};

}
