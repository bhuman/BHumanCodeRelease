/**
 * @file DeleteLogsCmd.cpp
 * @author Andreas Stolpmann
 */

#include "DeleteLogsCmd.h"
#include "Platform/File.h"
#include "Utils/bush/Initializer.h"
#include "Utils/bush/cmdlib/Commands.h"
#include "Utils/bush/cmdlib/ProcessRunner.h"
#include "Utils/bush/tools/Platform.h"
#include "Utils/bush/tools/ShellTools.h"
#include "Utils/bush/tools/StringTools.h"

#include <QMessageBox>

DeleteLogsCmd DeleteLogsCmd::theDeleteLogsCmd;

DeleteLogsCmd::DeleteLogsCmd()
{
  Commands::getInstance().addCommand(this);
}

std::string DeleteLogsCmd::getName() const
{
  return "deleteLogs";
}

std::string DeleteLogsCmd::getDescription() const
{
  return "Deletes all *.log files from the robot. Stops and restarts the B-Human if necessary";
}

bool DeleteLogsCmd::preExecution(Context& context, const std::vector<std::string>& params)
{
  if(params.size() > 0)
  {
    context.printLine("This command does not have any parameters!");
    return false;
  }

  return true;
}

Task* DeleteLogsCmd::perRobotExecution(Context& context, Robot& robot)
{
  return new DeleteLogsCmd::DeleteLogsTask(context, &robot);
}

bool DeleteLogsCmd::DeleteLogsTask::execute()
{
  QStringList args;
  args.push_back("--just-delete");
  args.push_back(fromString(robot->name));
  args.push_back(fromString(robot->getBestIP(context())));

  ProcessRunner r(context(), getCommand(), args);
  r.run();

  if(r.error())
    context().errorLine("Deleting failed!");
  else
    context().printLine("Success! (" + robot->name + ")");

  return true;
}

DeleteLogsCmd::DeleteLogsTask::DeleteLogsTask(Context& context,
    Robot* robot)
  : RobotTask(context, robot)
{}

QString DeleteLogsCmd::DeleteLogsTask::getCommand()
{
#ifdef WINDOWS
  return fromString(std::string(File::getBHDir()) + "/Make/" + makeDirectory() + "/downloadLogs.cmd");
#else
  return fromString(std::string(File::getBHDir()) + "/Make/" + makeDirectory() + "/downloadLogs");
#endif
}
