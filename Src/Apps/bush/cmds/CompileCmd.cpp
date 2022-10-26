#include "CompileCmd.h"
#include "cmdlib/Context.h"
#include "tools/Platform.h"
#include "Platform/File.h"
#include <QString>

CompileCmd::CompileCmd() :
  Command("compile")
{}

CompileCmd::CompileTask::CompileTask(Context& context,
                                     const QString& config,
                                     const QString& target) :
  Task(context),
  r(context, getCommand(), getParams(config, target))
{}

bool CompileCmd::CompileTask::execute()
{
  r.run();
  if(context().isCanceled())
  {
    context().cleanupFinished();
    return true;
  }
  if(r.error())
  {
    context().errorLine("Failed to compile.");
    return false;
  }
  return true;
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
#ifdef MACOS
  return "xcodebuild";
#else
  return "cmake";
#endif
}

QString CompileCmd::CompileTask::getCommand()
{
#ifdef MACOS
  return QString(File::getBHDir()) + "/Make/macOS/compile";
#else
  return "cmake";
#endif
}

QStringList CompileCmd::CompileTask::getParams(const QString& config, const QString& target)
{
  QStringList args;
#ifdef MACOS
  args << target << config;
#else
  args << "--build";
#ifdef WINDOWS
  args << QString(File::getBHDir()) + "/Build/Windows/CMake";
#else
  args << QString(File::getBHDir()) + "/Build/Linux/CMake/" + config;
#endif // WINDOWS
  args << "--target" << target;
  args << "--config" << config;
#ifdef WINDOWS
  args << "--" << "/nologo" // Do not show MSBuild version and copyright information
               << "/v:m" // Set logging verbosity to minimal
               << "/m"; // Use as many parallel processes as possible
#endif // WINDOWS
#endif // MACOS
  return args;
}

bool CompileCmd::execute(Context& context, const CompileArgs& args) const
{
  context.executeDetached(new CompileTask(context, args.config, args.target));
  return context.waitForChildren();
}
