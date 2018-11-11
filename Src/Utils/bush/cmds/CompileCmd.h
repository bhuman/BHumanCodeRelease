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
    bool execute() override;
    void cancel() override;
    void setContext(Context* context) override;
    std::string getLabel() override;
  };

  CompileCmd();
  std::string getName() const override;
  std::string getDescription() const override;
  std::vector<std::string> complete(const std::string& cmdLine) const override;
  bool execute(Context& context, const std::vector<std::string>& params) override;
  QString getCommand();
  QStringList getParams(const QString& config, const QString& project);

public:

  static CompileCmd theCompileCmd;
};
