#include "Filesystem.h"
#include "tools/Directory.h"
#include "tools/Platform.h"
#include "Platform/File.h"
#include <iostream>

std::vector<std::string> Filesystem::getWlanConfigs(const std::string& prefix)
{
  return getEntries(std::string(File::getBHDir())
                    + "/Install/Profiles/" + prefix, true, false);
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
                    + "/Build/" + platformDirectory() + "/CMake/" + prefix, true, false, ".vcxproj", false);
#else
  return getEntries(std::string(File::getBHDir())
                    + "/Make/CMake/" + prefix, true, false, ".cmake", false);
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
