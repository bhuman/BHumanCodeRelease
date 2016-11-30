#include "Utils/bush/cmdlib/ProcessRunner.h"
#include "Utils/bush/tools/Platform.h"
#include "Utils/bush/tools/StringTools.h"
#include <QDir>
#include <QRegExp>
#include <QString>

#ifdef WINDOWS
#include <Windows.h>
#endif

#ifdef MACOS
#include <iostream>
#include <cstdlib>
#endif

std::string bhumandDirOnRobot = "/home/nao/";

std::string makeDirectory()
{
#ifdef WINDOWS
  return "VS2015";
#elif defined MACOS
  return "macOS";
#else
  return "Linux";
#endif
}

std::string platformDirectory()
{
#ifdef WINDOWS
  return "Windows";
#elif defined MACOS
  return "macOS";
#else
  return "Linux";
#endif
}

void goToConfigDirectory(const char* argv0)
{
#ifdef WINDOWS
  char fileName[_MAX_PATH];
  char longFileName[_MAX_PATH];
  GetModuleFileNameA(GetModuleHandleA(0), fileName, _MAX_PATH);
  GetLongPathNameA(fileName, longFileName, _MAX_PATH);
  QString applicationPath = QString(longFileName);
  applicationPath = applicationPath.replace(QRegExp("Build\\\\\\w+\\\\bush\\\\\\w+\\\\bush.exe"), "");
#elif defined MACOS
  QString applicationPath = QDir::cleanPath(*argv0 == '/' ? QString(argv0) : QDir::root().current().path() + "/" + argv0);
  applicationPath = applicationPath.replace(QRegExp("Build/\\w+/bush/\\w+/bush.app/Contents/MacOS/bush"), "");
#else
  QString applicationPath = QDir::cleanPath(*argv0 == '/' ? QString(argv0) : QDir::root().current().path() + "/" + argv0);
  applicationPath = applicationPath.replace(QRegExp("Build/\\w+/bush/\\w+/bush"), "");
#endif

  applicationPath += QString("Config");
  QDir::setCurrent(applicationPath);
}

std::string linuxToPlatformPath(const std::string& path)
{
#ifdef WINDOWS
  return toString(QString(path.c_str()).replace("/", "\\"));
#else // linux and mac
  return path;
#endif
}

std::string getLinuxPath(const std::string& path)
{
#ifdef WINDOWS
  QString command("cygpath -u \"");
  command += fromString(path) + "\"";
  ProcessRunner r(command);
  r.run();
  return toString(r.getOutput().trimmed());
#else
  return path;
#endif
}
