/**
 * \file Directory.cpp
 * Implements a class for accessing directories.
 * \author Colin Graf
 */

#include <sys/types.h>
#include <sys/stat.h>
#ifdef WINDOWS
#include <Shlwapi.h>
#pragma comment(lib,"shlwapi.lib")
#else
#include <unistd.h>
#include <fnmatch.h>
#endif
#include <dirent.h>
#include <cerrno>

#include "Directory.h"
#include "Platform/BHAssert.h"

Directory::~Directory()
{
  if(dp)
    closedir(static_cast<DIR*>(dp));
}

bool Directory::open(const std::string& pattern)
{
  if(dp)
  {
    closedir(static_cast<DIR*>(dp));
    dp = 0;
  }

  size_t end = pattern.find_last_of("/\\");
  if(end == std::string::npos)
    return false;
  filepattern = pattern.substr(end + 1);
  dirname = pattern.substr(0, end);

  dp = opendir(dirname.c_str());
  return dp != 0;
}

bool Directory::read(std::string& name, bool& isDir)
{
  if(!dp)
    return false;

  for(;;)
  {
    struct dirent* dent = readdir(static_cast<DIR*>(dp));
    if(!dent)
    {
      closedir(static_cast<DIR*>(dp));
      dp = 0;
      return false;
    }
#ifdef WINDOWS
    if(PathMatchSpec(dent->d_name, filepattern.c_str()))
#else
    if(fnmatch(filepattern.c_str(), dent->d_name, 0) == 0)
#endif
    {
      name = dirname + "/" + dent->d_name;
      isDir = false;
      struct stat buff;
      if(stat(name.c_str(), &buff) == 0)
        if(S_ISDIR(buff.st_mode))
          isDir = true;
      return true;
    }
  }
}
