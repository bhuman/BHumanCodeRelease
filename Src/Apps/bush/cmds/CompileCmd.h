#pragma once

#include "cmdlib/Command.h"
#include "cmdlib/Context.h"
#include "cmdlib/ProcessRunner.h"
#include <QString>
#include <QStringList>

struct CompileArgs : CommandArgs
{
  CompileArgs(const QString& config, const QString& target) :
    config(config), target(target)
  {}

  QString config;
  QString target;
};

class CompileCmd : public Command<CompileCmd, CompileArgs>
{
public:
  CompileCmd();

private:
  class CompileTask : public Task
  {
  public:
    CompileTask(Context& context,
                const QString& config,
                const QString& target);

  private:
    bool execute() override;
    void cancel() override;
    void setContext(Context* context) override;
    std::string getLabel() override;

    static QString getCommand();
    static QStringList getParams(const QString& config, const QString& target);

    ProcessRunner r;
    const std::string label;
  };

  bool execute(Context& context, const CompileArgs& args) const override;
};
