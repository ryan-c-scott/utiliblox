#include <stdio.h>
#include <iostream>

#include "utils.h"

std::string extractFileName( const std::string fullPath, bool extension )
{
	int first = 0, last = (int)fullPath.length() - 1;
	
	for( int i = last; i >= 0; --i )
	{
		if( fullPath[i] == '.' )
		{
			last = i;
		}
		else if( fullPath[i] == '\\' || fullPath[i] == '/' )
		{
			first = i + 1;
			break;
		}
	}

	if( extension )
		return fullPath.substr( first, fullPath.length() - first );
	else
		return fullPath.substr( first, last - first );
}

std::string extractFilePath( const std::string &fullPath, unsigned int stripLevel )
{
	int last = 0;
	
	for( int i = (int)fullPath.length() - 1; i >= 0; --i )
	{
		if( fullPath[i] == '\\' || fullPath[i] == '/' )
		{
			last = ++i;
			break;
		}
	}

    int stripStart = 0;
    int stripping = 0;

    for( int i = 0; stripping < stripLevel && i < fullPath.length(); ++i ) {
        char c = fullPath[ i ];

        if( c == '\\' || c == '/' ) {
            stripStart = ++i;
            stripping++;
        }
    }

	return fullPath.substr( stripStart, last - stripStart );
}

std::string cleanPath( std::string path )
{
	// Remove spaces at the beginning
	int cnt = 0;
	for( int i = 0; i < (int)path.length(); ++i )
	{
		if( path[i] != ' ' ) break;
		else ++cnt;
	}
	if( cnt > 0 ) path.erase( 0, cnt );

	// Remove slashes, backslashes and spaces at the end
	cnt = 0;
	for( int i = (int)path.length() - 1; i >= 0; --i )
	{
		if( path[i] != '/' && path[i] != '\\' && path[i] != ' ' ) break;
		else ++cnt;
	}

	if( cnt > 0 ) path.erase( path.length() - cnt, cnt );

	return path;
}

void createDirectories( const std::string newPath )
{
	if( newPath.empty() ) return;
	
	std::string tmpString;
	tmpString.reserve( 256 );
	size_t i = 0, len = newPath.length();

	while( ++i < len )
	{
		if( newPath[i] == '/' || newPath[i] == '\\' || i == len-1 )
		{
			tmpString = newPath.substr( 0, ++i );
			_mkdir( tmpString.c_str() );
		}
	}
}

void Log( std::string msg )
{
    std::cout<< msg << std::endl;
}

uint32_t packUint32( uint8_t _x, uint8_t _y, uint8_t _z, uint8_t _w )
{
	union
	{
		uint32_t ui32;
		uint8_t arr[4];
	} un;

	un.arr[0] = _x;
	un.arr[1] = _y;
	un.arr[2] = _z;
	un.arr[3] = _w;

	return un.ui32;
}

uint32_t packF4u( float _x, float _y, float _z, float _w)
{
	const uint8_t xx = uint8_t(_x*127.0f + 128.0f);
	const uint8_t yy = uint8_t(_y*127.0f + 128.0f);
	const uint8_t zz = uint8_t(_z*127.0f + 128.0f);
	const uint8_t ww = uint8_t(_w*127.0f + 128.0f);
	return packUint32(xx, yy, zz, ww);
}
