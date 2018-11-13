// *************************************************************************************************
//
// Horde3D
//   Next-Generation Graphics Engine
// --------------------------------------
// Copyright (C) 2006-2011 Nicolas Schulz
//
// This software is distributed under the terms of the Eclipse Public License v1.0.
// A copy of the license may be obtained at: http://www.eclipse.org/legal/epl-v10.html
//
// *************************************************************************************************

#pragma once

#include "egPrerequisites.h"

namespace Horde3D {

// Forward declarations
class EngineLog;
class ResourceManager;

// =================================================================================================
// Modules
// =================================================================================================

class Modules
{
private:

	static bool                   _errorFlag;

	static EngineLog              *_engineLog;
	static ResourceManager        *_resourceManager;

	static void installExtensions();

public:

	static const char *versionString;
	
	static bool init();
	static void release();

	static void setError( const char *errorStr1 = 0x0, const char *errorStr2 = 0x0 );
	static bool getError();
	
	static EngineLog &log() { return *_engineLog; }
	static ResourceManager &resMan() { return *_resourceManager; }
};

}

