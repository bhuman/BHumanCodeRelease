#include "Utils/bush/tools/Filesystem.h"

#include "Directory.h"
#include "Platform/File.h"
#include "Platform.h"
#include <algorithm>
#include <cstring>
#include <iostream>
#include <sstream>
#include <fstream>
#include <QFile>

#if defined LINUX || defined MACOS
#include <cstdlib>
#include <sys/types.h>
#include <cerrno>
#endif

std::vector<std::string> Filesystem::getWlanConfigs(const std::string& prefix)
{
  return getEntries(std::string(File::getBHDir())
                    + "/Install/Network/Profiles/" + prefix, true, false);
}

std::vector<std::string> Filesystem::getLocations(const std::string& prefix)
{
  return getEntries(std::string(File::getBHDir())
                    + "/Config/Locations/" + prefix, false, true);
}

std::vector<std::string> Filesystem::getScenarios(const std::string& prefix)
{
  return getEntries(std::string(File::getBHDir())
                    + "/Config/Scenarios/" + prefix, false, true);
}

std::vector<std::string> Filesystem::getProjects(const std::string& prefix)
{
#ifdef WINDOWS
  return getEntries(std::string(File::getBHDir())
                    + "/Make/" + makeDirectory() + "/" + prefix, true, false, ".vcxproj", false);
#else
  return getEntries(std::string(File::getBHDir())
                    + "/Make/Linux/" + prefix, true, false, ".make", false);
#endif
}

std::vector<std::string> Filesystem::getEntries(const std::string& directory,
    bool files,
    bool directories,
    const std::string& suffix,
    bool keepSuffixes)
{
  std::vector<std::string> entries;
  Directory d;
  if(d.open(directory + "*" + suffix))
  {
    std::string dir;
    bool isDir;
    while(d.read(dir, isDir))
      if(isDir == !files || isDir == directories)
      {
        std::string stripped = dir.replace(0, directory.length(), "");
        if(!keepSuffixes)
          stripped = stripped.replace(stripped.size() - suffix.size(), stripped.size(), "");
        if(stripped != "." && stripped != "..")
        {
          if(isDir && files)
            stripped += '/';
          entries.push_back(stripped);
        }
      }
  }
  else
    std::cout << "Cannot open " << directory << std::endl; // TODO log?
  return entries;
}

std::string Filesystem::getFileAsString(const std::string& filename)
{
  std::stringstream buf;
  std::ifstream fin(filename.c_str());

  char c;
  while(fin.good() && (c = static_cast<char>(fin.get())) != EOF)
    buf << c;
  fin.close();

  return buf.str();
}
