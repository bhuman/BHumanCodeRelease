#include "Utils/bush/tools/ShellTools.h"
#include "Utils/bush/tools/StringTools.h"
#include "Utils/bush/tools/Platform.h"
#include "Platform/File.h"
#include "Filesystem.h"

std::string remoteCommand(const std::string& command, const std::string ip)
{
  static std::string keyFile = Filesystem::getNaoKey();
  std::string ticks = "\"";

  return "ssh -i \"" + keyFile + "\" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=quiet nao@" +
         ip + " " + ticks + command + " < /dev/null > /dev/null 2>&1 &" + ticks;
}

std::string remoteCommandForQProcess(const std::string& command, const std::string& ip)
{
  static std::string keyFile = Filesystem::getNaoKey();
  return "ssh -i \"" + keyFile + "\" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=quiet nao@" + ip + " " + command + "";
}

std::string scpCommand(const std::string& fromFile, const std::string& fromHost, const std::string& toDir, const std::string& toHost)
{
  static std::string keyFile = Filesystem::getNaoKey();
  std::string from;
  if(fromHost == "")
    from = enquoteString(getLinuxPath(fromFile));
  else
    from = fromHost + ":" + enquoteString(fromFile);
  std::string to;
  if(toHost == "")
    to = enquoteString(getLinuxPath(toDir));
  else
    to = toHost + ":" + enquoteString(toDir);

  return "scp -r -i \"" + getLinuxPath(keyFile) + "\" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=quiet " + from + " " + to;
}

std::string scpCommandFromRobot(const std::string& fromDir, const std::string& ip, const std::string& toDir)
{
  return scpCommand(fromDir, "nao@" + ip, toDir, "");
}

std::string scpCommandToRobot(const std::string& fromDir, const std::string& ip, const std::string& toDir)
{
  return scpCommand(fromDir, "", toDir, "nao@" + ip);
}
