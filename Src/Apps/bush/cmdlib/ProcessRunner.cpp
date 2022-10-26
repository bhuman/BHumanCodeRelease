/**
 * @file ProcessRunner.cpp
 *
 * Contains the implementation of ProcessRunner.
 *
 * @author <a href="mailto::ojlr@informatik.uni-bremen.de">Ole Jan Lars Riemann</a>
 */

#include "cmdlib/ProcessRunner.h"
#include "cmdlib/Context.h"
#include <QString>

ProcessRunner::ProcessRunner(const QString& command)
  : process(0),
    program(command),
    arguments(0),
    context(0)
{}

ProcessRunner::ProcessRunner(const QString& program, const QStringList& arguments)
  : process(0),
    program(program),
    arguments(new QStringList(arguments)),
    context(0)
{}

ProcessRunner::ProcessRunner(Context& context, const std::string& program, const QStringList& arguments)
  : process(0),
    program(QString::fromStdString(program)),
    arguments(new QStringList(arguments)),
    context(&context)
{}

ProcessRunner::ProcessRunner(Context& context, const QString& program, const QStringList& args)
  : process(0),
    program(program),
    arguments(new QStringList(args)),
    context(&context)
{}

ProcessRunner::ProcessRunner(Context& context, const std::string& command)
  : process(0),
    program(QString::fromStdString(command)),
    arguments(0),
    context(&context)
{}

ProcessRunner::ProcessRunner(Context& context, const QString& command)
  : process(0),
    program(command),
    arguments(0),
    context(&context)
{}

ProcessRunner::~ProcessRunner()
{
  if(process)
    process->deleteLater();
  if(arguments)
    delete arguments;
}

ProcessRunner& ProcessRunner::operator=(const ProcessRunner& other)
{
  if(this != &other)
  {
    if(arguments)
      delete arguments;

    process = other.process;
    program = other.program;
    if(other.arguments)
      arguments = new QStringList(*other.arguments);
    else
      arguments = 0;
  }
  return *this;
}

void ProcessRunner::run()
{
  if(!process)
    process = new QProcess();

  connect(process, &QProcess::readyReadStandardError, this, &ProcessRunner::updateError, Qt::DirectConnection);
  connect(process, &QProcess::readyReadStandardOutput, this, &ProcessRunner::updateText, Qt::DirectConnection);

  if(arguments)
    process->start(program, *arguments);
  else
    process->startCommand(program);

  interact(process);
  process->waitForFinished(-1); // no timeout

  process->close();

  disconnect(process, &QProcess::readyReadStandardError, this, &ProcessRunner::updateError);
  disconnect(process, &QProcess::readyReadStandardOutput, this, &ProcessRunner::updateText);
}

void ProcessRunner::stop()
{
#ifdef WINDOWS
  process->kill();
#else
  process->terminate();
#endif
}

QString ProcessRunner::getOutput()
{
  return output.join("\n");
}

void ProcessRunner::updateError()
{
  QByteArray data = process->readAllStandardError();
  QString errorOutput(data);
  output += errorOutput;
  if(context)
    context->error(errorOutput);
}

void ProcessRunner::updateText()
{
  QByteArray data = process->readAllStandardOutput();
  QString stdOutput(data);
  output += stdOutput;
  if(context)
    context->print(stdOutput);
}
