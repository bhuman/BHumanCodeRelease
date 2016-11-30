/**
 * @file Platform/Nao/File.cpp
 */

#include "Platform/File.h"

#include <cstring>

std::list<std::string> File::getFullNames(const std::string& name)
{
  std::list<std::string> names;
  if((name[0] != '.' || (name.size() >= 2 && name[1] == '.')) && !isAbsolute(name.c_str())) // given path is relative to getBHDir()
  {
    std::list<std::string> dirs = getConfigDirs();
    for(std::string& dir : dirs)
      names.push_back(dir + name);
  }
  else
    names.push_back(name);
  return names;
}

const char* File::getBHDir()
{
  static char dir[FILENAME_MAX] = {0};
  if(!dir[0])
  {
    strcpy(dir, ".");
  }
  return dir;
}
