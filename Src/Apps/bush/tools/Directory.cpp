/**
 * \file Directory.cpp
 * Implements a class for accessing directories.
 * \author Colin Graf
 */

#ifdef WINDOWS
#include <Shlwapi.h>
#pragma comment(lib,"shlwapi.lib")
#else
#include <fnmatch.h>
#endif

#include "Directory.h"

bool Directory::open(const std::string& pattern)
{
  size_t end = pattern.find_last_of("/\\");
  if(end == std::string::npos)
    return false;
  filepattern = pattern.substr(end + 1);
  dirname = pattern.substr(0, end);

  try
  {
    iter = std::filesystem::directory_iterator(dirname);
  }
  catch(...)
  {
    return false;
  }
  return true;
}

bool Directory::read(std::string& name, bool& isDir)
{
  for(; iter != std::filesystem::directory_iterator(); ++iter)
  {
#ifdef WINDOWS
    if(PathMatchSpec(iter->path().filename().string().c_str(), filepattern.c_str()))
#else
    if(fnmatch(filepattern.c_str(), iter->path().filename().c_str(), 0) == 0)
#endif
    {
      name = iter->path().string();
      isDir = iter->is_directory();
      ++iter;
      return true;
    }
  }
  return false;
}
