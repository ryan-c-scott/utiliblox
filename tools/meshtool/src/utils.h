#pragma once


#include <string>

#ifdef PLATFORM_WIN
#   define WIN32_LEAN_AND_MEAN 1
#	ifndef NOMINMAX
#		define NOMINMAX
#	endif
#	include <windows.h>
#	include <direct.h>
typedef unsigned char uint8_t;
typedef unsigned uint32_t;

#else
#   include <unistd.h>
#   include <stdint.h>
#   define _chdir chdir
#	include <sys/stat.h>
#	include <dirent.h>
#endif

#if !defined( PLATFORM_WIN ) && !defined( PLATFORM_WIN_CE )
#	define _stricmp strcasecmp
#	define _mkdir( name ) mkdir( name, 0755 )
#endif

#if !defined( PLATFORM_WIN ) && !defined( PLATFORM_WIN_CE )
#	define _stricmp strcasecmp
#	define _mkdir( name ) mkdir( name, 0755 )
#endif


std::string extractFileName( const std::string fullPath, bool extension );
std::string extractFilePath( const std::string &fullPath, unsigned int stripLevel=0 );
std::string cleanPath( const std::string path );
void createDirectories( const std::string newPath );
void Log( const std::string msg );

void* operator new[](size_t size, const char* pName, int flags, unsigned debugFlags, const char* file, int line);
void* operator new[](size_t size, size_t alignment, size_t alignmentOffset, const char* pName, int flags, unsigned debugFlags, const char* file, int line);

uint32_t packUint32( uint8_t _x, uint8_t _y, uint8_t _z, uint8_t _w );
uint32_t packF4u( float _x, float _y = 0.0f, float _z = 0.0f, float _w = 0.0f );
