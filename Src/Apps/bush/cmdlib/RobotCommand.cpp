#include "cmdlib/RobotCommand.h"
#include "cmdlib/Context.h"
#include "Session.h"

bool RobotCommandTools::checkSelectedRobots(Context& context)
{
  if(context.getSelectedRobots().empty())
  {
    context.errorLine("No robots selected.");
    return false;
  }
  return true;
}

bool RobotCommandTools::checkReachable(Context& context, Robot* robot, bool& status)
{
  if(!Session::getInstance().isReachable(context, robot))
  {
    context.errorLine("\"" + QString::fromStdString(robot->name) + "\" is not reachable.");
    return (status = false);
  }
  return true;
}
