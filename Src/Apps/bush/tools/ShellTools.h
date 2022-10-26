#pragma once

#include <string>

std::string remoteCommand(const std::string& command, const std::string& ip);
std::string remoteCommandForQProcess(const std::string& command, const std::string& ip);
std::string connectCommand(const std::string& ip);
