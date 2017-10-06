#include "Utils/bush/cmds/SimCmd.h"
#include "Utils/bush/cmdlib/Context.h"
#include "Utils/bush/cmdlib/Commands.h"
#include "Utils/bush/cmdlib/ProcessRunner.h"
#include "Utils/bush/tools/Platform.h"
#include "Platform/File.h"
#include <cstdlib>

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

bool SimCmd::execute(Context& context, const std::vector<std::string>& params)
{
  const std::string buildConfig = "Develop";
  const std::string simulatorExecutable = getSimulatorExecutable(buildConfig);
  const std::string remoteRobotScene = std::string(File::getBHDir()) + "/Config/Scenes/RemoteRobot.ros2";

  File simFile(simulatorExecutable, "r");
  if(!simFile.exists())
  {
    bool compileStatus = Commands::getInstance().execute(&context, "compile " + buildConfig + " SimRobot");
    if(!compileStatus)
      return false;
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
