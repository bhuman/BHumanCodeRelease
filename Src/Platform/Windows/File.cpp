/**
 * @file Platform/Windows/File.cpp
 */

#include "Platform/File.h"
#include "Platform/BHAssert.h"

#include <Windows.h>

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
  static char dir[MAX_PATH] = {0};
  if(!dir[0])
  {
    // determine module file from command line
    const char* commandLine = GetCommandLine();
    size_t len;
    if(*commandLine == '"')
    {
      commandLine++;
      const char* end = strchr(commandLine, '"');
      if(end)
        len = end - commandLine;
      else
        len = strlen(commandLine);
    }
    else
      len = strlen(commandLine);

    if(len >= sizeof(dir) - 8)
      len = sizeof(dir) - 8;
    memcpy(dir, commandLine, len);
    dir[len] = '\0';

    // if there is no given directory, use the current working dir
    if(!strchr(dir, '\\'))
    {
      len = int(GetCurrentDirectory(sizeof(dir) - 9, dir));
      if(len && dir[len - 1] != '\\')
      {
        dir[len++] = '\\';
        dir[len] = '\0';
      }
    }

    //drive letter in lower case:
    if(len && dir[1] == ':')
      *dir |= tolower(*dir);

    // try to find the config directory
    char* end = dir + len - 1;
    while(true)
    {
      if(end < dir || *end == '/' || *end == '\\' || *end == ':')
      {
        if(end >= dir && *end == ':')
          *(end++) = '\\';
        strcpy(end + 1, "Config");
        DWORD attr = GetFileAttributes(dir);
        if(attr != INVALID_FILE_ATTRIBUTES && attr & FILE_ATTRIBUTE_DIRECTORY)
        {
          end[end > dir ? 0 : 1] = '\0';
          for(; end >= dir; end--)
            if(*end == '\\')
              *end = '/';
          return dir;
        }
      }
      if(end < dir)
        break;
      end--;
    }
    FAIL("Could not find the Config directory.");
  }
  return dir;
}
