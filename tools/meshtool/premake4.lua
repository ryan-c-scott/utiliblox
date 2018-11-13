project "meshtool"
kind "ConsoleApp"
language "C++"

targetdir("../bin")

defines {
   "PLATFORM_WIN",
   "WIN32",
   "_WINDOWS"
}

configuration "Debug"
libdirs { "../../dependencies/fbxsdk-2019.0/lib/vs2015/x64/debug/" }

configuration "Release"
libdirs { "../../dependencies/fbxsdk-2019.0/lib/vs2015/x64/release/" }
configuration {}

includedirs
{
   'src',
   "../../src",
   "../../dependencies/fbxsdk-2019.0/include",
   "../../dependencies/EASTL/include",
   "../../dependencies/LuaJIT-2.0.0/src/",
   "../../dependencies/physfs/src",
   "../../dependencies/yajl/include",
}

libdirs 
{
   "../../dependencies/libs",
   "../../dependencies/LuaJIT-2.0.0/src/",
}

files
{
   "src/**.cpp",
   "src/**.h",
   "../../src/protobufs/**.cc",
   "../../dependencies/EASTL/source/*",
   "../lua.natvis",
   "../../dependencies/EASTL/doc/EASTL.natvis",
   "../../dependencies/lua-yajl/lua_yajl.c",
}

links
{
   "physfs",
   "yajl",
   'lua51',
   'libfbxsdk-mt',
}

postbuildcommands { "copy ..\\dependencies\\LuaJIT-2.0.0\\src\\lua51.dll ..\\tools\\bin\\" }