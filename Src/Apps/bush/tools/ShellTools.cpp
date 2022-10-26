#include "ShellTools.h"
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
    "bash -c \"cp -n ../Install/Keys/id_rsa_nao /tmp/id_rsa_nao; chmod 600 /tmp/id_rsa_nao && ssh -i /tmp/id_rsa_nao -o PasswordAuthentication=no -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=quiet nao@" + ip + "\"";
}
