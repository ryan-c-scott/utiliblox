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
#include "utMath.h"
#include <string>
#include <EASTL/vector.h>
#include <EASTL/map.h>

namespace VL
{
    class DataComponentManager;
}

namespace Horde3D {

// =================================================================================================
// Resource
// =================================================================================================

struct ResourceTypes
{
	enum List
	{
		Undefined = 0,
		Geometry,
		Animation,
		Material,
		Shader,
		Texture,
		ParticleEffect,
        ShaderProgram,
        RawAudio,
        DependencyMap,
        Script,
        UIFont,
        AudioPatch,
        Prefab,
        Scene,
        Text,
        Atlas,
        Protobuf,
        Pipeline,
	};
};

struct ResourceFlags
{
	enum Flags
	{
		NoQuery = 1,
		NoTexCompression = 2,
		NoTexMipmaps = 4,
		TexCubemap = 8,
		TexDynamic = 16,
		TexRenderable = 32,
		TexSRGB = 64
	};
};

// =================================================================================================

class Resource
{
protected:

	int                  _type;
	std::string          _name;
	ResHandle            _handle;
	int                  _flags;
    uint64_t             _hashedName;
	
	uint32               _refCount;  // Number of other objects referencing this resource
	uint32               _userRefCount;  // Number of handles created by user

	bool                 _loaded;
    bool _finalized;
	bool                 _noQuery;

public:

	Resource( int type, const std::string &name, int flags );
	virtual ~Resource();
	virtual Resource *clone();  // TODO: Implement this for all resource types
	
	virtual void initDefault();
	virtual void release();
	virtual bool load( const char *data, int size );
    virtual bool finalize();
    virtual void AddStaticData( VL::DataComponentManager &staticData ) {}
	void unload();
	
	int &getType() { return _type; }
	int getFlags() { return _flags; }
	const std::string &getName() { return _name; }
    uint64_t getHashedName() { return _hashedName; }
	ResHandle getHandle() { return _handle; }
	bool isLoaded() { return _loaded; }
    bool isFinalized() { return _finalized; }
	void addRef() { ++_refCount; }
	void subRef() { --_refCount; }
    int refs() { return _refCount; }

	friend class ResourceManager;
};


// =================================================================================================
// Resource Manager
// =================================================================================================

typedef void (*ResTypeInitializationFunc)();
typedef void (*ResTypeReleaseFunc)();
typedef Resource *(*ResTypeFactoryFunc)( const std::string &name, int flags );

struct ResourceRegEntry
{
	std::string                typeString;
	ResTypeInitializationFunc  initializationFunc;  // Called when type is registered
	ResTypeReleaseFunc         releaseFunc;  // Called when type is unregistered
	ResTypeFactoryFunc         factoryFunc;  // Factory to create resource object
};

// =================================================================================================

class ResourceManager
{
protected:

	eastl::vector < Resource * >         _resources;
	eastl::map< int, ResourceRegEntry >  _registry;  // Registry of resource types

	ResHandle addResource( Resource &res );

public:

	ResourceManager();
	~ResourceManager();

	void registerType( int type, const std::string &typeString, ResTypeInitializationFunc inf,
	                   ResTypeReleaseFunc rf, ResTypeFactoryFunc ff );
	
	Resource *getNextResource( int type, ResHandle start );
	Resource *findResource( int type, const std::string &name );
	Resource *findResource( const std::string &name );
	Resource *findResource( const uint64_t hashedName );
	ResHandle addResource( int type, const std::string &name, int flags, bool userCall );
	ResHandle addNonExistingResource( Resource &resource, bool userCall );
	ResHandle cloneResource( Resource &sourceRes, const std::string &name );
	int removeResource( Resource &resource, bool userCall );
	void clear();
	ResHandle queryUnloadedResource( int index );
	void releaseUnusedResources();
    void killResource( int res );

	Resource *resolveResHandle( ResHandle handle )
		{ return (handle != 0 && (unsigned)(handle - 1) < _resources.size()) ? _resources[handle - 1] : 0x0; }

	eastl::vector < Resource * > &getResources() { return _resources; }
};

}
