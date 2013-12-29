#include "Platform/File.h"
#include "Utils/bush/cmdlib/ProcessRunner.h"
#include "Utils/bush/cmds/UpdateWirelessCmd.h"
#include "Utils/bush/Session.h"
#include "Utils/bush/tools/Filesystem.h"
#include "Utils/bush/tools/ShellTools.h"
#include "Utils/bush/tools/StringTools.h"
#include <algorithm>

//The connman profiles contain placeholders where the robots actual
//team id and part number should be.
//The team id and part number make up the last 2 pieces of the
//ip address.
const std::string TEAM_ID("%teamID%");
const std::string ROBOT_PART("%robotPart%");


UpdateWirelessCmd::UpdateWirelessTask::UpdateWirelessTask(Context& context,
    Robot* robot, const std::string& command)
  : RobotTask(context, robot),
    command(command)
{ }

bool UpdateWirelessCmd::UpdateWirelessTask::execute()
{
  ProcessRunner r(context(), fromString(command));
  r.run();

  if(r.error())
  {
    context().errorLine("UpdateWireless of \"" + robot->name + "\" failed!");
    return false;
  }
  else
    return true;
}

UpdateWirelessCmd::UpdateWirelessCmd()
{
  Commands::getInstance().addCommand(this);
}

std::string UpdateWirelessCmd::getName() const
{
  return "updateWireless";
}

std::string UpdateWirelessCmd::getDescription() const
{
  return "updates the wireless configurations on selected robots.";
}

bool UpdateWirelessCmd::preExecution(Context& context, const std::vector<std::string>& params)
{
  if(!params.empty())
  {
    context.errorLine("No parameters allowed.");
    return false;
  }
  else
    return true;
}

Task* UpdateWirelessCmd::perRobotExecution(Context& context, Robot& robot)
{
  std::string fromDir = std::string(File::getBHDir()) + "/Install/Network/Profiles/";
  std::string toDir = "/home/nao/";
  return new UpdateWirelessTask(context, &robot, scpCommandToRobot(fromDir, robot.getBestIP(), toDir));
}

UpdateWirelessCmd UpdateWirelessCmd::theUpdateWirelessCmd;
