#include "Utils/bush/tools/ShellTools.h"
#include "Utils/bush/tools/StringTools.h"
#include "Utils/bush/tools/Platform.h"
#include "Platform/File.h"
#include "Filesystem.h"
#include <regex>

static std::string escapeCmdString(const std::string& string)
{
#ifdef WINDOWS
  static std::regex regex("[\\^\\&\\|\\<\\>]");
  return std::regex_replace(string, regex, "^$&");
#else
  return string;
#endif
}

std::string remoteCommand(const std::string& command, const std::string& ip)
{
  return remoteCommandForQProcess(command + " </dev/null >/dev/null 2>&1 &", ip);
}

std::string remoteCommandForQProcess(const std::string& command, const std::string& ip)
{
  // The triple quote is converted to a single quote by Qt.
  return connectCommand(ip + " \"\"\"" + escapeCmdString(command) + "\"\"\"");
}

std::string connectCommand(const std::string& ip)
{
  return
#ifdef WINDOWS
    "cmd /c "
#endif
    "bash -c \"cp Keys/id_rsa_nao /tmp/id_rsa_nao && chmod 600 /tmp/id_rsa_nao && ssh -i /tmp/id_rsa_nao -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=quiet nao@" + ip + "\"";
}

std::string scpCommand(const std::string& fromFile, const std::string& fromHost, const std::string& toDir, const std::string& toHost)
{
  std::string from;
  if(fromHost == "")
    from = enquoteString(fromFile);
  else
    from = fromHost + ":" + enquoteString(fromFile);
  std::string to;
  if(toHost == "")
    to = enquoteString(toDir);
  else
    to = toHost + ":" + enquoteString(toDir);

  return
#ifdef WINDOWS
    "cmd /c "
#endif
    "bash -c \"cp Keys/id_rsa_nao /tmp/id_rsa_nao && chmod 600 /tmp/id_rsa_nao && scp -r -i /tmp/id_rsa_nao -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=quiet " + from + " " + to + "\"";
}

std::string scpCommandFromRobot(const std::string& fromDir, const std::string& ip, const std::string& toDir)
{
  return scpCommand(fromDir, "nao@" + ip, toDir, "");
}

std::string scpCommandToRobot(const std::string& fromDir, const std::string& ip, const std::string& toDir)
{
  return scpCommand(fromDir, "", toDir, "nao@" + ip);
}
