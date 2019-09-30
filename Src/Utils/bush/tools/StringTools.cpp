#include "Utils/bush/tools/StringTools.h"
#include <sstream>

std::vector<std::string> split(const std::string& line, char separator)
{
  std::vector<std::string> elements;
  size_t last = 0;
  size_t i;
  for(i = 0; i < line.length(); ++i)
  {
    if(line[i] == separator)
    {
      if(i - last > 0)
        elements.push_back(line.substr(last, i - last));
      last = i + 1;
    }
  }
  if(i - last > 0)
    elements.push_back(line.substr(last, i - last));
  return elements;
}

std::string join(const std::vector<std::string> line, const std::string& separator)
{
  std::stringstream buf;
  for(size_t i = 0; i < line.size(); ++i)
  {
    buf << line[i];
    if(i < line.size() - 1)
      buf << separator;
  }
  return buf.str();
}

bool startsWidth(const std::string& str, const std::string& prefix)
{
  if(str.length() < prefix.length())
    return false;

  for(size_t i = 0; i < prefix.length(); ++i)
    if(prefix[i] != str[i])
      return false;

  return true;
}

std::vector<std::string> getBuildConfigs(const std::string& prefix)
{
  std::vector<std::string> configs;
  configs.push_back("Develop");
  configs.push_back("Release");
  configs.push_back("Debug");

  for(std::vector<std::string>::iterator it = configs.begin();
      it != configs.end();)
  {
    if(it->find(prefix) == 0)
      it++->replace(0, prefix.size(), "");
    else
      it = configs.erase(it);
  }

  return configs;
}

std::string enquoteString(const std::string& arg)
{
  return "\"" + toString(QString(arg.c_str()).replace(" ", "\\ ")) + "\"";
}

QString fromString(const std::string& arg)
{
  return QString(arg.c_str());
}
