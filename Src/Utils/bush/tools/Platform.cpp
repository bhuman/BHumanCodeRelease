#include "Utils/bush/cmdlib/ProcessRunner.h"
#include "Utils/bush/tools/Platform.h"
#include "Utils/bush/tools/StringTools.h"
#include <QDir>
#include <QRegExp>
#include <QString>

#ifdef WIN32
#include <Windows.h>
#endif // WIN32

#if defined(MACOSX)
#include <iostream>
#include <cstdlib>
#endif

std::string bhumandDirOnRobot = "/home/nao/";

std::string makeDirectory()
{
#ifdef WIN32
  char* vsPath = getenv(ENV_COMNTOOLS_11);
  return "VS2012";
#elif defined(MACOSX)
  return "MacOS";
#else
  return "Linux";
#endif
}

std::string platformDirectory()
{
#ifdef WIN32
  return "Win32";
#elif defined(MACOSX)
  return "MacOS";
#else
  return "Linux";
#endif
}

void goToConfigDirectory(const char* argv0)
{
#ifdef WIN32
  char fileName[_MAX_PATH];
  char longFileName[_MAX_PATH];
  GetModuleFileNameA(GetModuleHandleA(0), fileName, _MAX_PATH);
  GetLongPathNameA(fileName, longFileName, _MAX_PATH);
  QString applicationPath = QString(longFileName);
  applicationPath = applicationPath.replace(QRegExp("Build\\\\bush\\\\\\w+\\\\\\w+\\\\bush.exe"), "");
#elif defined(MACOSX)
  QString applicationPath = QDir::cleanPath(*argv0 == '/' ? QString(argv0) : QDir::root().current().path() + "/" + argv0);
  applicationPath = applicationPath.replace(QRegExp("Build/bush/\\w+/\\w+/bush.app/Contents/MacOS/bush"), "");
#else
  QString applicationPath = QDir::cleanPath(*argv0 == '/' ? QString(argv0) : QDir::root().current().path() + "/" + argv0);
  applicationPath = applicationPath.replace(QRegExp("Build/bush/\\w+/\\w+/bush"), "");
#endif

  applicationPath += QString("Config");
  QDir::setCurrent(applicationPath);
}

std::string linuxToPlatformPath(const std::string& path)
{
#ifdef WIN32
  return toString(QString(path.c_str()).replace("/", "\\"));
#else // linux and mac
  return path;
#endif
}

std::string getLinuxPath(const std::string& path)
{
#ifdef WIN32
  QString command("cygpath -u \"");
  command += fromString(path) + "\"";
  ProcessRunner r(command);
  r.run();
  return toString(r.getOutput().trimmed());
#else
  return path;
#endif
}
