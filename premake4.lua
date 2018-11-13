_projectDirectoryName = "utilityblox";
_projectName = "UtilityBlox";

--
solution "utilityblox"
platforms { "x64" }
configurations { "Debug", "Release" }
location "build"

flags { 
   "NoExceptions",
   "No64BitChecks",
   'StaticRuntime',
}

configuration "Release"
flags { "OptimizeSpeed", "Symbols" }
defines { "NDEBUG" }

configuration "Debug"
defines { "_DEBUG" }
flags { "Symbols" }

----
configuration {}
include "tools/meshtool";

----
configuration {}
include 'dependencies';

