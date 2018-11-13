project "yajl"
kind "StaticLib"
language "C++"
targetdir( "../libs" );

files {
   "src/*.c",
   "src/api/*.c",
}

includedirs {
   "src",
   "include",
}
