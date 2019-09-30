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
  return "VS2019";
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

std::string getVisualStudioPath()
{
#ifdef WINDOWS
  HANDLE vswhereStdoutP;
  HANDLE vswhereStdoutC;

  SECURITY_ATTRIBUTES saAttr;
  saAttr.nLength = sizeof(SECURITY_ATTRIBUTES);
  saAttr.bInheritHandle = TRUE;
  saAttr.lpSecurityDescriptor = NULL;
  if(!CreatePipe(&vswhereStdoutP, &vswhereStdoutC, &saAttr, 0) || !SetHandleInformation(vswhereStdoutP, HANDLE_FLAG_INHERIT, 0))
    return "";

  STARTUPINFO startInfo;
  BOOL bSuccess = FALSE;
  ZeroMemory(&startInfo, sizeof(STARTUPINFO));
  startInfo.cb = sizeof(STARTUPINFO);
  startInfo.hStdError = GetStdHandle(STD_ERROR_HANDLE);
  startInfo.hStdOutput = vswhereStdoutC;
  startInfo.hStdInput = GetStdHandle(STD_INPUT_HANDLE);
  startInfo.dwFlags |= STARTF_USESTDHANDLES;
  PROCESS_INFORMATION procInfo;
  ZeroMemory(&procInfo, sizeof(PROCESS_INFORMATION));

  char cmdLine[] = "-latest -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath";
  if(CreateProcess("C:\\Program Files (x86)\\Microsoft Visual Studio\\Installer\\vswhere.exe", cmdLine, NULL, NULL, TRUE, 0, NULL, NULL, &startInfo, &procInfo))
  {
    CloseHandle(procInfo.hProcess);
    CloseHandle(procInfo.hThread);

    char buf[_MAX_PATH];
    DWORD pathLen;
    if(ReadFile(vswhereStdoutP, buf, _MAX_PATH, &pathLen, NULL) && pathLen > 2)
    {
      CloseHandle(vswhereStdoutP);
      CloseHandle(vswhereStdoutC);
      return std::string(buf, pathLen - 2) + "\\";
    }
  }
  CloseHandle(vswhereStdoutP);
  CloseHandle(vswhereStdoutC);
#endif
  return "";
}
