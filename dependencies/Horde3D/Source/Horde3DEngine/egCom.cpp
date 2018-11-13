#include "egCom.h"
#include "utMath.h"
#include "egModules.h"
#include <stdarg.h>
#include <stdio.h>

#include "utDebug.h"

namespace Horde3D {

EngineLog::EngineLog()
{
	_timer.setEnabled( true );
	_maxNumMessages = 512;
	_customLogHandler = NULL;
}

void EngineLog::pushMessage( int level, const char *msg, va_list args )
{
}

void EngineLog::writeError( const char *msg, ... )
{
}

void EngineLog::writeWarning( const char *msg, ... )
{
}

void EngineLog::writeInfo( const char *msg, ... )
{
}

void EngineLog::writeDebugInfo( const char *msg, ... )
{
}

bool EngineLog::getMessage( LogMessage &msg )
{
    return false;
}

}  // namespace
