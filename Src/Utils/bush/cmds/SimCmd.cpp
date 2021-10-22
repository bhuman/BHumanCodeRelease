#include "Utils/bush/cmds/SimCmd.h"
#include "Utils/bush/cmdlib/Context.h"
#include "Utils/bush/cmdlib/Commands.h"
#include "Utils/bush/cmdlib/ProcessRunner.h"
#include "Utils/bush/models/Robot.h"
#include "Utils/bush/tools/Platform.h"
#include "Platform/File.h"
#include <sstream>

SimCmd SimCmd::theSimCmd;

SimCmd::SimCmd()
{
  Commands::getInstance().addCommand(this);
}

std::string SimCmd::getName() const
{
  return "sim";
}

std::string SimCmd::getDescription() const
{
  return "Connects to robots via simulator. Requires a Simulator built with Develop configuration.";
}

bool SimCmd::execute(Context& context, const std::vector<std::string>&)
{
  const std::string buildConfig = "Develop";
  const std::string simulatorExecutable = getSimulatorExecutable(buildConfig);
  const std::string remoteRobotScene = std::string(File::getBHDir()) + "/Config/Scenes/RemoteRobot.ros2";
  const std::string connectConPath = "Scenes/Includes/connect.con";

  File simFile(simulatorExecutable, "r");
  if(!simFile.exists())
  {
    bool compileStatus = Commands::getInstance().execute(&context, "compile " + buildConfig + " SimRobot");
    if(!compileStatus)
      return false;
  }
  if(context.getSelectedRobots().size() == 1 && context.getSelectedRobots()[0])
  {
    const Robot& robot = *context.getSelectedRobots()[0];
    const std::string bestIp = robot.getBestIP(context);
    const std::string cmd = "sc Remote " + bestIp;
    context.printLine("Overriding connect.con to connect to " + robot.name + " (" + bestIp +")");
    File connectCon(connectConPath, "w");
    connectCon.write(cmd.c_str(), cmd.size());
   }

  ProcessRunner r(context, "\"" + simulatorExecutable + "\" \"" + remoteRobotScene + "\"");
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
