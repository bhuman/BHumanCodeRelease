#pragma once

#define ENV_COMNTOOLS_11 "VS110COMNTOOLS" // VS 2012
#include <string>

extern std::string bhumandDirOnRobot; // TODO global configuration file
std::string makeDirectory();
std::string platformDirectory();
void goToConfigDirectory(const char* argv0);
std::string linuxToPlatformPath(const std::string& path);
std::string getLinuxPath(const std::string& path);
