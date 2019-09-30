#include "Platform/File.h"
#include "Utils/bush/cmds/SSHCmd.h"
#include "Utils/bush/cmdlib/Context.h"
#include "Utils/bush/cmdlib/Commands.h"
#include "Utils/bush/cmdlib/ProcessRunner.h"
#include "Utils/bush/tools/ShellTools.h"
#include "Utils/bush/tools/StringTools.h"
#include "Utils/bush/Session.h"
#include <cstdlib>

SSHCmd SSHCmd::theSSHCmd;

SSHCmd::SSHCmd()
{
  Commands::getInstance().addCommand(this);
}

std::string SSHCmd::getName() const
{
  return "ssh";
}

std::string SSHCmd::getDescription() const
{
  return "Executes a command via ssh or opens a ssh session.";
}

bool SSHCmd::preExecution(Context& context, const std::vector<std::string>& params)
{
  if(!params.empty())
  {
    command = params.front();
    for(auto param = params.begin() + 1; param != params.end(); ++param)
      command += " " + *param;
  }
  else
    command.clear();

  return true;
}

Task* SSHCmd::perRobotExecution(Context& context, Robot& robot)
{
  return new SSHTask(context, &robot, command);
}

SSHCmd::SSHTask::SSHTask(Context& context,
                         Robot* robot,
                         const std::string& command)
  : RobotTask(context, robot),
    command(command)
{}

bool SSHCmd::SSHTask::execute()
{
  std::string commandToRun = "";
  if(command == "")
  {
#ifdef WINDOWS
    commandToRun = "cmd /c start " + connectCommand(robot->getBestIP(context()));
#elif defined LINUX
    commandToRun = "xterm -hold -e " + connectCommand(robot->getBestIP(context()));
#elif defined MACOS
    commandToRun = std::string(File::getBHDir()) + "/Make/macOS/loginFromBush " + robot->getBestIP(context());
#endif // WINDOWS
  }
  else
  {
    commandToRun = remoteCommandForQProcess(command, robot->getBestIP(context()));
  }

  ProcessRunner r(context(), commandToRun);
  r.run();

  if(r.error())
  {
    context().errorLine(robot->name + ": ssh command \"" + command + "\" failed.");
    return false;
  }
  return true;
}
