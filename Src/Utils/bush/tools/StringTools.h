#pragma once

#include <vector>
#include <string>
#include <QString>

std::vector<std::string> split(const std::string& line, char separator = ' ');
std::string join(const std::vector<std::string> line, const std::string& separator = " ");
bool startsWidth(const std::string& str, const std::string& prefix);
std::vector<std::string> getBuildConfigs(const std::string& prefix = "");

std::string enquoteString(const std::string& arg);

QString fromString(const std::string& arg);

template<typename T>
std::string toString(T arg)
{
  return QString("%1").arg(arg).toUtf8().data();
}
