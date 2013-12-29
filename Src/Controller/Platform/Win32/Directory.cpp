/**
* \file Controller/Platform/Win32/Directory.cpp
* Implements a class for accessing directories.
* This is the Win32 implementation.
* \author Colin Graf
*/

#undef UNICODE
#include <windows.h>

#include "Directory.h"
#include "Platform/BHAssert.h"

Directory::Directory() : findFile(0)
{
  ASSERT(sizeof(ffd) >= sizeof(WIN32_FIND_DATA));
}

Directory::~Directory()
{
  if(findFile)
    FindClose((HANDLE)findFile);
}

bool Directory::open(const std::string& pattern)
{
  if(findFile)
  {
    FindClose((HANDLE)findFile);
    findFile = 0;
  }

  size_t end = pattern.find_last_of("/\\");
  if(end == std::string::npos)
    return false;
  dirname = pattern.substr(0, end);

  findFile = FindFirstFile(pattern.c_str(), (LPWIN32_FIND_DATA)ffd);
  if(findFile == INVALID_HANDLE_VALUE)
  {
    findFile = 0;
    return false;
  }
  bufferedEntry = true;
  return true;
}

bool Directory::read(std::string& name, bool& isDir)
{
  if(!findFile)
    return false;
  if(bufferedEntry)
    bufferedEntry = false;
  else if(!FindNextFile((HANDLE)findFile, (LPWIN32_FIND_DATA)ffd))
  {
    FindClose((HANDLE)findFile);
    findFile = 0;
    return false;
  }
  name = dirname + "\\" + ((LPWIN32_FIND_DATA)ffd)->cFileName;
  isDir = ((LPWIN32_FIND_DATA)ffd)->dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY ? true : false;
  return true;
}
