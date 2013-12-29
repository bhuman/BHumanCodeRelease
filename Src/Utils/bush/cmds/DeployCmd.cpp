#include <iostream>
#include "Platform/File.h"
#include <QString>
#include <QStringList>
#include "Utils/bush/agents/PingAgent.h"
#include "Utils/bush/cmdlib/Context.h"
#include "Utils/bush/cmdlib/Commands.h"
#include "Utils/bush/cmdlib/ProcessRunner.h"
#include "Utils/bush/cmds/DeployCmd.h"
#include "Utils/bush/tools/StringTools.h"
#include "Utils/bush/tools/Platform.h"
#include "Utils/bush/Session.h"
#include "Utils/bush/models/Team.h"

DeployCmd DeployCmd::theDeployCmd;

DeployCmd::DeployCmd()
{
  Commands::getInstance().addCommand(this);
}

std::string DeployCmd::getName() const
{
  return "deploy";
}

std::string DeployCmd::getDescription() const
{
  return "Deploys code to selected robots. (Uses the copyfiles script)";
}

std::vector<std::string> DeployCmd::complete(const std::string& cmdLine) const
{
  std::vector<std::string> commandWithArgs = split(cmdLine);

  if(commandWithArgs.size() == 1)
    return getBuildConfigs();
  else
    return getBuildConfigs(commandWithArgs[1]);
}

bool DeployCmd::preExecution(Context &context, const std::vector<std::string> &params)
{
  team = context.getSelectedTeam();
  if(!team)
  {
    context.errorLine("No team selected!");
    return false;
  }

  buildConfig = fromString(team->buildConfig);
  if(params.size() > 0)
    buildConfig = fromString(params[0]);

  // compile and deploy if compiling was successful
  return context.execute("compile " + toString(buildConfig));
}

Task* DeployCmd::perRobotExecution(Context &context, Robot &robot)
{
  /* Since the PingAgent knows the roundtrip time for all robots, maybe we can
   * adjust the timeout of rsync to determine faster if the connection is
   * lost.
   */
  QString command = getCommand();

  QStringList args = QStringList();
  args.push_back(QString("-nc"));
  args.push_back(buildConfig);
  args.push_back(fromString(robot.getBestIP()));

  ProcessRunner r(context, command, args);
  r.run();
  if(r.error())
    context.errorLine("Deploy of \"" + robot.name + "\" failed!");
  else
    context.printLine("Success! (" + robot.name + ")");
  return NULL;
}

bool DeployCmd::postExecution(Context &context, const std::vector<std::string> &params)
{
  // deploy finished, change the settings now
  // TODO: pass the location
  return context.execute("updateSettings");
}

#ifdef WIN32
QString DeployCmd::getCommand()
{
  return fromString(std::string(File::getBHDir()) + "/Make/" + makeDirectory() + "/copyfiles.cmd");
}
#else
QString DeployCmd::getCommand()
{
  return fromString(std::string(File::getBHDir()) + "/Make/" + makeDirectory() + "/copyfiles");
}
#endif
