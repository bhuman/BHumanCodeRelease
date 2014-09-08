#pragma once

#include <vector>
#include <string>

class Filesystem
{
public:
  static std::vector<std::string> getWlanConfigs(const std::string prefix = "");
  static std::vector<std::string> getLocations(const std::string prefix = "");
  static std::vector<std::string> getProjects(const std::string prefix = "");
  static std::vector<std::string> getEntries(const std::string& directory,
      bool files,
      bool directories,
      const std::string& suffix = "",
      bool keepSuffixes = true);
  static std::string getFileAsString(const std::string& filename);

  /**Returns the path to the private key used to connect to the NAO*/
  static std::string getNaoKey();
};
