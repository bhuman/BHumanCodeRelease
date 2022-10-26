#include "SimCmd.h"
#include "cmdlib/Commands.h"
#include "cmdlib/Context.h"
#include "cmdlib/ProcessRunner.h"
#include "cmds/CompileCmd.h"
#include "models/Robot.h"
#include "tools/Platform.h"
#include "Platform/File.h"
#include <QString>

SimCmd::SimCmd() :
  Command("sim")
{}

bool SimCmd::execute(Context& context) const
{
  const std::string buildConfig = "Develop";
  const std::string simulatorExecutable = getSimulatorExecutable(buildConfig);
  const std::string remoteRobotScene = std::string(File::getBHDir()) + "/Config/Scenes/RemoteRobot.ros2";
  const std::string connectConPath = "Scenes/Includes/connect.con";

  File simFile(simulatorExecutable, "r");
  if(!simFile.exists())
  {
    if(!context.execute(Commands::getInstance().get("compile"), new CompileArgs(QString::fromStdString(buildConfig), "SimRobot")))
      return false;
  }
  if(context.getSelectedRobots().size() == 1 && context.getSelectedRobots()[0])
  {
    const Robot& robot = *context.getSelectedRobots()[0];
    const std::string bestIp = robot.getBestIP(context);
    if(bestIp.empty())
    {
      context.errorLine("\"" + QString::fromStdString(robot.name) + "\" is not reachable.");
      return false;
    }
    const std::string cmd = "sc Remote " + bestIp;
    context.printLine("Overriding connect.con to connect to " + QString::fromStdString(robot.name) + " (" + QString::fromStdString(bestIp) +")");
    File connectCon(connectConPath, "w");
    connectCon.write(cmd.c_str(), cmd.size());
  }

  ProcessRunner r(context, simulatorExecutable, {QString::fromStdString(remoteRobotScene)});
  r.run();
  if(r.error())
  {
    context.errorLine("Failed.");
    return false;
  }
  return true;
}

std::string SimCmd::getSimulatorExecutable(const std::string& buildConfig)
{
  std::string simulatorExecutable = std::string(File::getBHDir()) + "/Build/"
                                    + platformDirectory() + "/SimRobot/" + buildConfig + "/SimRobot";
#ifdef MACOS
  simulatorExecutable += ".app/Contents/MacOS/SimRobot";
#elif defined WINDOWS
  simulatorExecutable += ".exe";
#endif

  return simulatorExecutable;
}
