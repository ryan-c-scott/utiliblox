#include "egResource.h"
#include "egModules.h"
#include "egCom.h"
#include "utDebug.h"

namespace Horde3D {

const char *Modules::versionString = "Horde3D 1.0.0 Beta5";

bool                   Modules::_errorFlag = false;
EngineLog              *Modules::_engineLog = 0x0;
ResourceManager        *Modules::_resourceManager = 0x0;

bool Modules::init()
{
	if( _engineLog == 0x0 ) _engineLog = new EngineLog();
	if( _resourceManager == 0x0 ) _resourceManager = new ResourceManager();

	return true;
}

void Modules::release()
{
	delete _resourceManager; _resourceManager = 0x0;
	delete _engineLog; _engineLog = 0x0;
}

void Modules::setError( const char *errorStr1, const char *errorStr2 )
{
	static std::string msg;
	msg.resize( 0 );

	if( errorStr1 ) msg.append( errorStr1 );
	if( errorStr2 ) msg.append( errorStr2 );
	
	_errorFlag = true;
	_engineLog->writeDebugInfo( msg.c_str() );
}


bool Modules::getError()
{
	if( _errorFlag )
	{
		_errorFlag = false;
		return true;
	}
	else
		return false;
}

}
