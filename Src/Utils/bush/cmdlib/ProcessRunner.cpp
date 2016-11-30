/**
 * @file ProcessRunner.cpp
 *
 * Contains the implementation of ProcessRunner and RemoteWriteProcessRunner
 *
 * @author <a href="mailto::ojlr@informatik.uni-bremen.de">Ole Jan Lars Riemann</a>
 */

#include "Utils/bush/cmdlib/ProcessRunner.h"
#include "Utils/bush/cmdlib/Context.h"
#include "Utils/bush/tools/StringTools.h"
#include <iostream>
#include <QTextCodec>

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
    program(fromString(program)),
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
    program(fromString(command)),
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
#ifdef WINDOWS
  QProcessEnvironment env = env.systemEnvironment();
  env.insert("CYGWIN", "nodosfilewarning");
  process->setProcessEnvironment(env);
#endif

  connect(process, SIGNAL(readyReadStandardError()), this, SLOT(updateError()), Qt::DirectConnection);
  connect(process, SIGNAL(readyReadStandardOutput()), this, SLOT(updateText()), Qt::DirectConnection);

  if(arguments)
    process->start(program, *arguments);
  else
    process->start(program);

  interact(process);
  process->waitForFinished(-1); // no timeout

  process->close();

  disconnect(process, SIGNAL(readyReadStandardError()), this, SLOT(updateError()));
  disconnect(process, SIGNAL(readyReadStandardOutput()), this, SLOT(updateText()));
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
#ifdef WINDOWS
    context->print(toString(QTextCodec::codecForName("IBM 850")->toUnicode(data)));
#else
    context->error(toString(errorOutput));
#endif
}

void ProcessRunner::updateText()
{
  QByteArray data = process->readAllStandardOutput();
  QString stdOutput(data);
  output += stdOutput;
  if(context)
#ifdef WINDOWS
    context->print(toString(QTextCodec::codecForName("IBM 850")->toUnicode(data)));
#else
    context->print(toString(stdOutput));
#endif
}

RemoteWriteProcessRunner::RemoteWriteProcessRunner(Context& context,
    const QString& program,
    const QStringList& arguments,
    const QByteArray& data)
  : ProcessRunner(context, program, arguments),
    data(data)
{}

void RemoteWriteProcessRunner::interact(QProcess* process)
{
  if(process->write(data) < 0)
    context->errorLine("Cannot write to remote file.");
  process->closeWriteChannel();
}
