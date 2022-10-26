#include "Platform.h"
#include <QDir>
#include <QRegularExpression>
#include <QString>

#ifdef WINDOWS
#include <Windows.h>
#endif

std::string makeDirectory()
{
#ifdef WINDOWS
  return "Windows";
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
  static_cast<void>(argv0);
  char fileName[_MAX_PATH];
  char longFileName[_MAX_PATH];
  GetModuleFileNameA(GetModuleHandleA(0), fileName, _MAX_PATH);
  GetLongPathNameA(fileName, longFileName, _MAX_PATH);
  QString applicationPath = QString(longFileName);
  applicationPath = applicationPath.replace(QRegularExpression("Build\\\\\\w+\\\\bush\\\\\\w+\\\\bush.exe"), "");
#elif defined MACOS
  QString applicationPath = QDir::cleanPath(*argv0 == '/' ? QString(argv0) : QDir::root().current().path() + "/" + argv0);
  applicationPath = applicationPath.replace(QRegularExpression("Build/\\w+/bush/\\w+/bush.app/Contents/MacOS/bush"), "");
#else
  QString applicationPath = QDir::cleanPath(*argv0 == '/' ? QString(argv0) : QDir::root().current().path() + "/" + argv0);
  applicationPath = applicationPath.replace(QRegularExpression("Build/\\w+/bush/\\w+/bush"), "");
#endif

  applicationPath += QString("Config");
  QDir::setCurrent(applicationPath);
}
