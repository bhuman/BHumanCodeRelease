#pragma once

#define ENV_COMNTOOLS_12 "VS120COMNTOOLS" // VS 2013
#include <string>

extern std::string bhumandDirOnRobot; // TODO global configuration file
std::string makeDirectory();
std::string platformDirectory();
void goToConfigDirectory(const char* argv0);
std::string linuxToPlatformPath(const std::string& path);
std::string getLinuxPath(const std::string& path);
