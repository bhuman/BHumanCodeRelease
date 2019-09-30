#include "Utils/bush/cmds/CompileCmd.h"
#include "Utils/bush/cmdlib/Context.h"
#include "Utils/bush/tools/Filesystem.h"
#include "Utils/bush/tools/StringTools.h"
#include "Utils/bush/models/Team.h"
#include "Platform/File.h"
#include "Utils/bush/tools/Platform.h"

CompileCmd CompileCmd::theCompileCmd;

CompileCmd::CompileCmd()
{
  Commands::getInstance().addCommand(this);
}

std::string CompileCmd::getName() const
{
  return "compile";
}

std::string CompileCmd::getDescription() const
{
  return "[ <config> [ <project> ] ]\nCompiles a project with a specified build configuration. [Default: Develop and Nao]";
}

std::vector<std::string> CompileCmd::complete(const std::string& cmdLine) const
{
  std::vector<std::string> commandWithArgs = split(cmdLine);

  if(commandWithArgs.size() == 1)
    return getBuildConfigs();
  else if(commandWithArgs.size() == 2 && *--cmdLine.end() != ' ')
    return getBuildConfigs(commandWithArgs[1]);
  else if(commandWithArgs.size() == 2)
    return Filesystem::getProjects("");
  else
    return Filesystem::getProjects(commandWithArgs[2]);
}

CompileCmd::CompileTask::CompileTask(Context& context,
                                     const std::string& label,
                                     const QString& command,
                                     const QStringList& args)
  : Task(context),
    r(context, command, args),
    label(label)
{}

bool CompileCmd::CompileTask::execute()
{
  r.run();
  if(context().isCanceled())
  {
    context().cleanupFinished();
    return true;
  }
  bool status = true;
  if(r.error())
  {
    context().errorLine("Failed to compile.");
    status = false;
  }
  return status;
}

void CompileCmd::CompileTask::cancel()
{
  r.stop();
}

void CompileCmd::CompileTask::setContext(Context* context)
{
  r.setContext(*context);
  Task::setContext(context);
}

std::string CompileCmd::CompileTask::getLabel()
{
  return label;
}

#ifdef LINUX
#define LABEL "make"
#elif defined MACOS
#define LABEL "xcodebuild"
#elif defined WINDOWS
#define LABEL "vcxproj"
#endif

bool CompileCmd::execute(Context& context, const std::vector<std::string>& params)
{
  QString command = getCommand();
  QStringList args;

  if(params.size() > 2)
  {
    context.errorLine("Too many parameters specified.");
    return false;
  }
  else if(params.empty())
  {
    Team* team = context.getSelectedTeam();
    if(team && team->buildConfig.length() > 0)
      args = getParams(fromString(team->buildConfig), "Nao");
    else
      args = getParams("Develop", "Nao");
  }
  else if(params.size() == 1)
    args = getParams(fromString(params[0]), "Nao");
  else
    args = getParams(fromString(params[0]), fromString(params[1]));

  context.executeDetached(new CompileTask(context,
                                          LABEL,
                                          command,
                                          args));
  return context.waitForChildren();
}

#ifdef WINDOWS
QString CompileCmd::getCommand()
{
  return QString(getVisualStudioPath().c_str()) + "MSBuild\\Current\\Bin\\MSBuild.exe";
}

QStringList CompileCmd::getParams(const QString& config, const QString& project)
{
  QStringList args;
  const QString makeDir(fromString(makeDirectory()));
  args << fromString(std::string(File::getBHDir())).replace("/", "\\") + "\\Make\\" + makeDir + "\\B-Human.sln";
  args << QString("/t:" + project) << QString("/p:Configuration=" + config)
       << QString("/nologo") // Do not show MSBuild version and copyright information
       << QString("/m")  // Use as many parallel processes as possible
       << QString("/v:m"); // Set logging verbosity to minimal
  return args;
}
#elif defined MACOS
QString CompileCmd::getCommand()
{
  return fromString(std::string(File::getBHDir())) + "/Make/macOS/compile";
}

QStringList CompileCmd::getParams(const QString& config, const QString& project)
{
  QStringList args;
  args << project << config;
  return args;
}
#else
QString CompileCmd::getCommand()
{
  return fromString("bash"); // hack for those who use 64bit systems and bash functions to set right CFLAGS
}

QStringList CompileCmd::getParams(const QString& config, const QString& project)
{
  QStringList args;
  args << "-c";

  QString mke = "make -C " + fromString(std::string(File::getBHDir()) + "/Make/Linux");
  mke += fromString(" CONFIG=") + config + " " + project;

  args << mke;

  return args;
}
#endif // WINDOWS
