#include "SSHCmd.h"
#include "cmdlib/Context.h"
#include "cmdlib/ProcessRunner.h"
#include "tools/ShellTools.h"
#include "Platform/File.h"

SSHCmd::SSHCmd() :
  RobotCommand("ssh")
{}

Task* SSHCmd::perRobotExecution(Context& context, Robot& robot) const
{
  return new SSHTask(context, &robot);
}

bool SSHCmd::SSHTask::execute()
{
  std::string commandToRun;
#ifdef WINDOWS
  commandToRun = "cmd /c start " + connectCommand(robot->getBestIP(context()));
#elif defined LINUX
  commandToRun = "xterm -hold -e " + connectCommand(robot->getBestIP(context()));
#elif defined MACOS
  commandToRun = std::string(File::getBHDir()) + "/Make/macOS/loginFromBush " + robot->getBestIP(context());
#endif // WINDOWS

  ProcessRunner r(context(), commandToRun);
  r.run();

  if(r.error())
  {
    context().errorLine(QString::fromStdString(robot->name) + ": ssh failed.");
    return false;
  }
  return true;
}
