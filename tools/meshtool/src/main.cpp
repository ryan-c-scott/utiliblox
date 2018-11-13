#include <string>

#include <EASTL/vector.h>
#include <EASTL/set.h>
#include <EASTL/hash_map.h>
#include <EASTL/string.h>

#include <fbxsdk.h>

#include <iostream>
#include <fstream>
#include <sstream>

#include "utils.h"

extern "C"
{
#include <lua.h>
#include <lauxlib.h>
#include <lualib.h>
    
#include <physfs.h>

    LUALIB_API int luaopen_yajl(lua_State *L);
    LUALIB_API int js_to_string(lua_State *L);
}

//
using namespace std;

const float MaxFloat = 3.402823466e+38F;
bool _debuggingEnabled = false;

// EASTL integration
// EASTL expects us to define these, see allocator.h line 194
void* operator new[](size_t size, const char* pName, int flags,
                     unsigned debugFlags, const char* file, int line)
{
        return malloc( size );
}
void* operator new[](size_t size, size_t alignment, size_t alignmentOffset,
                     const char* pName, int flags, unsigned debugFlags, const char* file, int line)
{
        return malloc( size );
}

int Vsnprintf8(char8_t* pDestination, size_t n, const char8_t* pFormat, va_list arguments)
{
    #ifdef _MSC_VER
        return _vsnprintf(pDestination, n, pFormat, arguments);
    #else
        return vsnprintf(pDestination, n, pFormat, arguments);
    #endif
}

void printHelp()
{
        Log( "Usage:" );
        Log( "\tmeshtool <script> [-p key value]... <file>..." );
        Log( "" );
        Log( "\tscript           asset file to be processed" );
        Log( "\t-p, --param      key/value pair to be set in _PARAMS table for script" );
        Log( "\tfile             file list to be put in _FILES table for script" );
        Log( "" );
        Log( "" );
}

eastl::hash_map<eastl::string, FbxScene*> _openScenes;
FbxManager *_importManager = NULL;

const FbxScene* OpenScene( const char *path )
{
    // Check whether file is in open set and return pointer from there if present
    auto sceneEntry = _openScenes.find( path );

    if( sceneEntry != _openScenes.end() ) {
        return sceneEntry->second;
    }
    
    const auto scene = FbxScene::Create( _importManager, path );
    
    if( !scene ) {
        // TODO:  Fix message
        Log( "Processing failed" );
        return scene;
    }

    //
    FbxImporter* lImporter = FbxImporter::Create( _importManager, "" );

    // Initialize the importer by providing a filename.
    int lFileMajor, lFileMinor, lFileRevision;
    const bool lImportStatus = lImporter->Initialize( path, -1, _importManager->GetIOSettings() );
    lImporter->GetFileVersion( lFileMajor, lFileMinor, lFileRevision );

    if( !lImportStatus ) {
        FbxString error = lImporter->GetStatus().GetErrorString();
        FBXSDK_printf( "Call to FbxImporter::Initialize() failed.\n" );
        FBXSDK_printf( "Error returned: %s\n\n", error.Buffer() );

        if( lImporter->GetStatus().GetCode() == FbxStatus::eInvalidFileVersion ) {
            // FBXSDK_printf("FBX file format version for this FBX SDK is %d.%d.%d\n", lSDKMajor, lSDKMinor, lSDKRevision);
            // FBXSDK_printf("FBX file format version for file '%s' is %d.%d.%d\n\n", pFilename, lFileMajor, lFileMinor, lFileRevision);
        }

        return NULL;
    }

    auto &settings = *_importManager->GetIOSettings();
    settings.SetBoolProp( IMP_FBX_MATERIAL, true );
    settings.SetBoolProp( IMP_FBX_TEXTURE, true );
    settings.SetBoolProp( IMP_FBX_LINK, true );
    settings.SetBoolProp( IMP_FBX_SHAPE, true );
    settings.SetBoolProp( IMP_FBX_GOBO, true );
    settings.SetBoolProp( IMP_FBX_ANIMATION, true );
    settings.SetBoolProp( IMP_FBX_GLOBAL_SETTINGS, true );

    ////
    // Import the scene.
    lImporter->Import( scene );

    // Destroy the importer.
    lImporter->Destroy();
    
    return scene;
}

void CloseScene( const char *path )
{
    // Verify scene in open set
    auto sceneEntry = _openScenes.find( path );

    if( sceneEntry == _openScenes.end() ) {
        return;
    }

    auto scene = sceneEntry->second;
    
    _openScenes.erase( sceneEntry );
}

// Lua
static void PushMaterial( lua_State *L, FbxSurfaceMaterial &material )
{
    lua_createtable( L, 0, 0 );
        
    lua_pushstring( L, material.GetName() );
    lua_setfield( L, -2, "name" );
}

static void PushFileTexture( lua_State *L, FbxFileTexture &texture )
{
    lua_createtable( L, 0, 0 );
        
    lua_pushstring( L, texture.GetName() );
    lua_setfield( L, -2, "name" );

    lua_pushstring( L, texture.GetRelativeFileName() );
    lua_setfield( L, -2, "relative_path" );

    lua_pushstring( L, texture.GetFileName() );
    lua_setfield( L, -2, "path" );
        
    lua_pushinteger( L, texture.GetMaterialUse() );
    lua_setfield( L, -2, "use" );
}

static void PushMesh( lua_State *L, FbxMesh &mesh )
{
    lua_createtable( L, 0, 0 );

    lua_pushinteger( L, mesh.GetPolygonCount() );
    lua_setfield( L, -2, "polygon_count" );

    lua_pushinteger( L, mesh.GetPolygonVertexCount() );
    lua_setfield( L, -2, "vertex_count" );

    lua_pushinteger( L, mesh.GetUVLayerCount() );
    lua_setfield( L, -2, "uv_count" );
}

static int _ProcessFile( lua_State *L )
{
    if( lua_gettop( L ) < 1 || lua_isnil( L, 1 ) ) {
        lua_pushstring( L, "missing or bad file name parameter" );
        lua_error( L );
        return 0;
    }
    
    auto path = lua_tostring( L, 1 );
    auto scene = OpenScene( path );
    
    // Return a structure with meshes and material data

    if( !scene ) {
        lua_pushstring( L, "Error processing file" );
        lua_error( L );
        return 0;
    }

    lua_createtable( L, 0, 3 );

    // TODO:  Populate tables

    //
    lua_createtable( L, 0, 0 );
    for( int i = 0; i < scene->GetSrcObjectCount<FbxMesh>(); ++i ) {
        const auto mesh = scene->GetSrcObject<FbxMesh>( i );
        lua_pushinteger( L, i + 1 );
        PushMesh( L, *mesh );
        lua_settable( L, -3 );
    }
    lua_setfield( L, -2, "meshes" );

    //
    lua_createtable( L, 0, 0 );
    for( int i = 0; i < scene->GetSrcObjectCount<FbxSurfaceMaterial>(); ++i ) {
        const auto material = scene->GetSrcObject<FbxSurfaceMaterial>( i );
        lua_pushinteger( L, i + 1 );
        PushMaterial( L, *material );
        lua_settable( L, -3 );
    }
    lua_setfield( L, -2, "materials" );

    //
    lua_createtable( L, 0, 0 );
    for( int i = 0; i < scene->GetSrcObjectCount<FbxFileTexture>(); ++i ) {
        const auto texture = scene->GetSrcObject<FbxFileTexture>( i );
        lua_pushinteger( L, i + 1 );
        PushFileTexture( L, *texture );
        lua_settable( L, -3 );
    }
    lua_setfield( L, -2, "textures" );
    
    return 1;
}

static int _CloseFile( lua_State *L )
{
    if( lua_gettop( L ) != 1 ) {
        lua_pushstring( L, "Bad filename");
        lua_error( L );
        return 0;
    }

    auto path = lua_tostring( L, 1 );
    CloseScene( path );
    
    return 0;
}

static int _OutputJson( lua_State *L )
{
    if( lua_gettop( L ) != 2 || !lua_istable( L, 1 ) || !lua_isstring( L, 2 ) ) {
        lua_pushstring( L, "Bad parameters to OutputJson" );
        lua_error( L );
        return 0;
    }

    auto outPath = lua_tostring( L, 2 );
    lua_pop( L, 1 );

    js_to_string( L );

    auto json = lua_tostring( L, -1 );
    auto fptr = fopen( outPath, "w" );

    if( fptr ) {
        fwrite( json, sizeof(char), strlen(json), fptr );
        fclose( fptr );
    }
    else {
        lua_pushstring( L, "Error writing file" );
        lua_error( L );
    }
    
    return 0;
}

static int _OutputLua( lua_State *L )
{
    if( lua_gettop( L ) != 2 || !lua_istable( L, 1 ) || !lua_isstring( L, 2 ) ) {
        lua_pushstring( L, "Bad parameters to OutputJson" );
        lua_error( L );
        return 0;
    }

    auto outPath = lua_tostring( L, 2 );
    std::ofstream out;
    lua_pop( L, 1 );

    int depth = 0;

    out.open( outPath );
    out << "{";
    
    lua_pushnil( L );
    while( true ) {
        if( !lua_next( L, -2 ) ) {
            depth--;
            out << "}\n";
            
            if( depth < 0 ) {
                break;
            }
            else {
                out << ",\n";
            }

            continue;
        }

        // KEYS //
        switch( lua_type( L, -2 ) ) {
        case LUA_TSTRING:
            out << "[\"" << lua_tostring( L, -2 ) << "\"]";
            break;

        case LUA_TNUMBER:
            out << lua_tonumber( L, -2 );
            break;
        }

        out << " = ";
        
        // VALUES //
        switch( lua_type( L, -1 ) ) {
        case LUA_TSTRING:
            out << "\"" << lua_tostring( L, -1 ) << "\",\n";
            break;

        case LUA_TNIL:
        case LUA_TNUMBER:
        case LUA_TBOOLEAN:
            out << lua_tonumber( L, -1 ) << ",\n";
            break;

        case LUA_TTABLE:
            depth++;
            out << "{";
            break;
        }

        lua_pop( L, 1 );
    }

    out.close();
    
    return 0;
}

static int LuaPanic( lua_State *L )
{
        const char *str = lua_tostring( L, -1 );
        printf( "PANIC: %s", str );
        return 0;
}

lua_State* CreateLuaState()
{
    lua_State *L = luaL_newstate();

    lua_atpanic( L, LuaPanic );
    luaL_openlibs( L );

    luaopen_yajl( L );
        lua_setglobal( L, "json" );

    const luaL_reg registrations[] = {
        { "ProcessFile", _ProcessFile },
        { "CloseFile", _CloseFile },
        { "OutputJson", _OutputJson },
        { "OutputLua", _OutputLua },
        { NULL, NULL }
    };

    for( int i = 0; registrations[ i ].name != NULL; i++ ) {
        lua_register( L, registrations[ i ].name, registrations[ i ].func );
    }
    
    return L;
}

//
void SetupFbx()
{
    _importManager = FbxManager::Create();

    if( !_importManager ) {
        FBXSDK_printf( "Error: Unable to create FBX Manager!\n" );
        exit(1);
    }
        else {
        FBXSDK_printf( "Autodesk FBX SDK version %s\n", _importManager->GetVersion() );
    }
}

void DestroyFbx()
{
    // TODO:  Maybe destroy any existing scenes that are open
    
    _importManager->Destroy();
}

int main( int argc, char **argv )
{
        if( argc < 2 ) {
                printHelp();
                return 1;
        }

    lua_State *L = CreateLuaState();

    // FBX Setup
    SetupFbx();
        
        //Create an IOSettings object. This object holds all import/export settings.
        FbxIOSettings* ios = FbxIOSettings::Create( _importManager, IOSROOT );
        _importManager->SetIOSettings( ios );

        // Load FBX plugins
        FbxString lPath = FbxGetApplicationDirectory();
        _importManager->LoadPluginsDirectory( lPath.Buffer() );
    //////////////////////////
    
        // =============================================================================================
        // Parse arguments
        // =============================================================================================
    eastl::vector< string > assetList;
    eastl::hash_map< eastl::string, eastl::string > paramList;
    string input, scriptPath;
    unsigned int stripLevel = 0;
        bool geoOpt = true, overwriteMats = false;
    int effectivePosition = 0;
        
        // Check optional arguments
        for( int i = 1; i < argc; ++i ) {
                if( _stricmp( argv[i], "--param" ) == 0 || _stricmp( argv[i], "-p" ) == 0 ) {
            paramList[ argv[ ++i ] ] = argv[ ++i ];
        }
        else {
                        input = cleanPath( argv[i] );

            if( input[0] == '/' || input[1] == ':' || input[0] == '\\' ) {
                size_t index = input.find_last_of( "\\/" );
                _chdir( input.substr( 0, index ).c_str() );
                input = input.substr( index + 1, input.length() - index );
            }

            if( effectivePosition == 0 ) {
                scriptPath = input;
            }
            else {
                assetList.push_back( input );
            }
            
            effectivePosition++;
        }            
        } // end for arguments


    if( scriptPath.length() <= 0 ) {
        Log( "Error: No script specified" );
        exit( 1 );
    }

    // Push file list into a lua global
    lua_createtable( L, assetList.size(), 0 );
    
        for( unsigned int i = 0; i < assetList.size(); ++i ) {
        lua_pushinteger( L, i + 1 );
        lua_pushstring( L, assetList[ i ].c_str() );
        lua_settable( L, -3 );
        }

    lua_setglobal( L, "_FILES" );

    // Push specified script parameters
    lua_createtable( L, 0, paramList.size() );

    for( auto param : paramList ) {
        lua_pushstring( L, param.first.c_str() );
        lua_pushstring( L, param.second.c_str() );
        lua_settable( L, -3 );
        }

    lua_setglobal( L, "_PARAMS" );
    
    
    if( luaL_dofile( L, scriptPath.c_str() ) ) {
        Log( lua_tostring( L, -1 ) );
    }
    
    DestroyFbx();
    
        return 0;
}
