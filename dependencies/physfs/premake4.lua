---------------------
project "physfs";
kind "StaticLib";
language "C";
targetdir( "../libs" );

defines
{
   "_REENTRANT",
   "_THREAD_SAFE",
   "_CRT_SECURE_NO_WARNINGS=1",
   "PHYSFS_SUPPORTS_ZIP=1",
   "PHYSFS_SUPPORTS_7Z=1",
   --"PHYSFS_SUPPORTS_ISO9660=1",
}

files
{
   -- lzma support
   "src/lzma/C/7zCrc.c",
   "src/lzma/C/Archive/7z/7zBuffer.c",
   "src/lzma/C/Archive/7z/7zDecode.c",
   "src/lzma/C/Archive/7z/7zExtract.c",
   "src/lzma/C/Archive/7z/7zHeader.c",
   "src/lzma/C/Archive/7z/7zIn.c",
   "src/lzma/C/Archive/7z/7zItem.c",
   "src/lzma/C/Archive/7z/7zMethodID.c",
   "src/lzma/C/Compress/Branch/BranchX86.c",
   "src/lzma/C/Compress/Branch/BranchX86_2.c",
   "src/lzma/C/Compress/Lzma/LzmaDecode.c",

   -- physfs itself
   "src/physfs.c",
   "src/physfs_byteorder.c",
   "src/physfs_unicode.c",
   "src/platform_posix.c",
   "src/platform_unix.c",
   "src/platform_macosx.c",
   "src/platform_windows.c",
   "src/archiver_dir.c",
   "src/archiver_unpacked.c",
   "src/archiver_grp.c",
   "src/archiver_hog.c",
   "src/archiver_lzma.c",
   "src/archiver_mvl.c",
   "src/archiver_qpak.c",
   "src/archiver_wad.c",
   "src/archiver_zip.c",
   "src/archiver_slb.c",
   "src/archiver_iso9660.c",
}

includedirs
{
   "src",
}

if os.is( "macosx" ) then
   linkoptions { "-framework Foundation", "-framework IOKit" }
end
