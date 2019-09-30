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

DeployCmd::DeployTask::DeployTask(Context& context, const QString& buildConfig, Team* team, Robot* robot)
  : RobotTask(context, robot),
    buildConfig(buildConfig),
    team(team)
{}

bool DeployCmd::DeployTask::execute()
{
  /* Since the PingAgent knows the roundtrip time for all robots, maybe we can
   * adjust the timeout of rsync to determine faster if the connection is
   * lost.
   */
  QString command = DeployCmd::getCommand();

  QStringList args = QStringList();
  args.push_back(QString("-nc"));
  args.push_back(QString("-nr"));
  args.push_back(buildConfig);
  args.push_back(fromString(robot->getBestIP(context())));
  if(team->getPlayersPerNumber()[team->getPlayerNumber(*robot) - 1][0] == robot)
    args.push_back(QString("-b"));
  args.push_back(QString("-t"));
  args.push_back(QString::number(team->number));
  args.push_back(QString("-o"));
  args.push_back(QString::number(team->port));
  args.push_back(QString("-c"));
  args.push_back(team->color.c_str());
  args.push_back(QString("-p"));
  args.push_back(QString::number(team->getPlayerNumber(*robot)));
  args.push_back(QString("-s"));
  args.push_back(team->scenario.c_str());
  args.push_back(QString("-l"));
  args.push_back(team->location.c_str());
  args.push_back(QString("-w"));
  args.push_back(team->wlanConfig.c_str());
  args.push_back(QString("-v"));
  args.push_back(QString::number(team->volume));
  args.push_back(QString("-m"));
  args.push_back(QString::number(team->magicNumber));

  ProcessRunner r(context(), command, args);
  r.run();
  if(r.error())
  {
    context().errorLine("Deploy of \"" + robot->name + "\" failed!");
    return false;
  }
  else
  {
    context().printLine("Success! (" + robot->name + ")");
    return true;
  }
}

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

bool DeployCmd::preExecution(Context& context, const std::vector<std::string>& params)
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
  if(team->compile)
  {
    if(!context.execute("compile " + toString(buildConfig)))
      return false;
  }
  return true;
}

Task* DeployCmd::perRobotExecution(Context& context, Robot& robot)
{
  return new DeployTask(context, buildConfig, team, &robot);
}

#ifdef WINDOWS
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
