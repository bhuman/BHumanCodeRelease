#include "DeployCmd.h"
#include "cmdlib/Commands.h"
#include "cmdlib/Context.h"
#include "cmdlib/ProcessRunner.h"
#include "cmds/CompileCmd.h"
#include "models/Team.h"
#include "tools/Platform.h"
#include "Platform/File.h"
#include <QString>
#include <QStringList>

DeployCmd::DeployCmd() :
  RobotCommand("deploy")
{}

DeployCmd::DeployTask::DeployTask(Context& context, const QString& buildConfig, const Team* team, Robot* robot) :
  RobotTask(context, robot),
  buildConfig(buildConfig),
  team(team)
{}

bool DeployCmd::DeployTask::execute()
{
  QStringList args;
  args.push_back(QString("-nc"));
  args.push_back(QString("-nr"));
  args.push_back(buildConfig);
  args.push_back(QString::fromStdString(robot->getBestIP(context())));
  if(team->getPlayersPerNumber()[team->getPlayerNumber(*robot) - 1][0] == robot)
    args.push_back(QString("-b"));
  args.push_back(QString("-t"));
  args.push_back(QString::number(team->number));
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

  ProcessRunner r(context(), getCommand(), args);
  r.run();
  if(r.error())
  {
    context().errorLine("Deploy of \"" + QString::fromStdString(robot->name) + "\" failed!");
    return false;
  }
  else
  {
    context().printLine("Success! (" + QString::fromStdString(robot->name) + ")");
    return true;
  }
}

QString DeployCmd::DeployTask::getCommand()
{
#ifdef WINDOWS
  return QString::fromStdString(std::string(File::getBHDir()) + "/Make/" + makeDirectory() + "/deploy.cmd");
#else
  return QString::fromStdString(std::string(File::getBHDir()) + "/Make/" + makeDirectory() + "/deploy");
#endif
}

bool DeployCmd::preExecution(Context& context, const DeployArgs& args) const
{
  const Team* team = context.getSelectedTeam();
  if(!team)
  {
    context.errorLine("No team selected!");
    return false;
  }

  // compile and deploy if compiling was successful
  if(team->compile)
  {
    if(!context.execute(Commands::getInstance().get("compile"), new CompileArgs(args.config, "Nao")))
      return false;
  }

  return true;
}

Task* DeployCmd::perRobotExecution(Context& context, Robot& robot, const DeployArgs& args) const
{
  return new DeployTask(context, args.config, context.getSelectedTeam(), &robot);
}
