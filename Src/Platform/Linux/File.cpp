/**
 * @file Platform/Linux/File.cpp
 */

#include "Platform/File.h"
#include "Platform/BHAssert.h"

#include <cstring>
#include <unistd.h>
#include <sys/stat.h>

std::list<std::string> File::getFullNames(const std::string& rawName)
{
  std::string name = rawName;
  for(size_t i = name.length(); i-- > 0;)
    if(name[i] == '\\')
      name[i] = '/';

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
    VERIFY(getcwd(dir, sizeof(dir) - 7) != 0);
    char* end = dir + strlen(dir) - 1;
    struct stat buff;
    while(true)
    {
      if(end < dir || *end == '/')
      {
        strcpy(end + 1, "Config");
        if(stat(dir, &buff) == 0)
          if(S_ISDIR(buff.st_mode))
          {
            end[end > dir ? 0 : 1] = '\0';
            return dir;
          }
      }
      if(end < dir)
        break;
      end--;
    }
    strcpy(dir, ".");
  }
  return dir;
}
