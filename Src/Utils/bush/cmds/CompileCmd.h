#pragma once

#include <QObject>
#include "Utils/bush/cmdlib/CommandAdapter.h"
#include "Utils/bush/cmdlib/Commands.h"
#include "Utils/bush/cmdlib/Context.h"
#include "Utils/bush/cmdlib/ProcessRunner.h"

class CompileCmd : public CommandAdapter
{
  class CompileTask : public Task
  {
    ProcessRunner r;
    const std::string label;
  public:
    CompileTask(Context& context,
                const std::string& label,
                const QString& command,
                const QStringList& args);
    bool execute();
    void cancel();
    void setContext(Context* context);
    std::string getLabel();
  };

  CompileCmd();
  virtual std::string getName() const;
  virtual std::string getDescription() const;
  virtual std::vector<std::string> complete(const std::string& cmdLine) const;
  virtual bool execute(Context& context, const std::vector<std::string>& params);
  QString getCommand();
  QStringList getParams(const QString& config, const QString& project);
public:

  static CompileCmd theCompileCmd;
};
