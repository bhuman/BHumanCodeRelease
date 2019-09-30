#pragma once

#include <string>

std::string remoteCommand(const std::string& command, const std::string& ip);
std::string remoteCommandForQProcess(const std::string& command, const std::string& ip);
std::string connectCommand(const std::string& ip);

std::string scpCommand(const std::string& fromDir, const std::string& fromHost, const std::string& toDir, const std::string& toHost);
std::string scpCommandFromRobot(const std::string& fromDir, const std::string& ip, const std::string& toDir);
std::string scpCommandToRobot(const std::string& fromDir, const std::string& ip, const std::string& toDir);
