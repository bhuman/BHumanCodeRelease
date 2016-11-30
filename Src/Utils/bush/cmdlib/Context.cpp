#include "Utils/bush/cmdlib/Context.h"
#include "Utils/bush/cmdlib/Commands.h"
#include "Utils/bush/cmdlib/Command.h"
#include "Utils/bush/ui/Console.h"
#include "Utils/bush/tools/StringTools.h"

ContextRunnable::ContextRunnable(QObject* parent)
  : QThread(parent)
{}

void ContextRunnable::run()
{
  bool status = execute();
  emit sFinished(status);
}

CommandRunnable::CommandRunnable(Context* context, const std::string& cmdLine)
  : ContextRunnable(context),
    context(context),
    cmdLine(cmdLine)
{}

bool CommandRunnable::execute()
{
  return Commands::getInstance().execute(context, cmdLine);
}

TaskRunnable::TaskRunnable(QObject* parent, Task* task)
  : ContextRunnable(parent),
    task(task)
{}

bool TaskRunnable::execute()
{
  return task->execute();
}

Context::Context(Context* parent, const std::string& cmdLine)
  : AbstractConsole(parent),
    parent(parent),
    cmdLine(new std::string(cmdLine)),
    task(0),
    thread(0),
    selectedRobots(parent->getSelectedRobots()),
    selectedTeam(parent->getSelectedTeam()),
    cmds(),
    detached(false),
    canceled(false),
    status(true),
    requestApplicationShutdown(false)
{}

Context::Context(Context* parent, Task* task)
  : AbstractConsole(parent),
    parent(parent),
    cmdLine(0),
    task(task),
    thread(0),
    selectedRobots(parent->getSelectedRobots()),
    selectedTeam(parent->getSelectedTeam()),
    cmds(),
    detached(true),
    canceled(false),
    status(true),
    requestApplicationShutdown(false)
{}

Context::Context(Context* parent, const std::string& cmdLine, bool detach)
  : AbstractConsole(parent),
    parent(parent),
    cmdLine(new std::string(cmdLine)),
    task(0),
    thread(0),
    selectedRobots(parent->getSelectedRobots()),
    selectedTeam(parent->getSelectedTeam()),
    cmds(),
    detached(detach),
    canceled(false),
    status(true),
    requestApplicationShutdown(false)
{}

Context::Context(const std::vector<Robot*>& selectedRobots,
                 Team* selectedTeam)
  : AbstractConsole(0),
    parent(0),
    cmdLine(0),
    task(0),
    thread(0),
    selectedRobots(selectedRobots),
    selectedTeam(selectedTeam),
    cmds(),
    detached(false),
    canceled(false),
    status(true),
    requestApplicationShutdown(false)
{}

Context::~Context()
{
  if(cmdLine) delete cmdLine;
  if(task && task->isAutoDelete()) delete task;
  if(thread) thread->deleteLater();
  for(size_t i = 0; i < cmds.size(); ++i)
  {
    Context* c = cmds[i];
    c->wait();
    delete c;
  }
}

bool Context::run()
{
  if(isDetached())
  {
    if(task)
      thread = new TaskRunnable(this, task);
    else
      thread = new CommandRunnable(this, *cmdLine);

    /* Use DirectConnection here since events with QueuedConnection are only
     * delivered if the receiver thread returns to the event loop, which is not
     * given in every bush thread. */
    connect(thread, SIGNAL(sFinished(bool)),
            this, SLOT(threadFinished(bool)),
            Qt::DirectConnection);

    thread->start();
    return true;
  }
  else
  {
    bool status = Commands::getInstance().execute(this, *cmdLine);
    emit sFinished(status);
    return status;
  }
}

bool Context::execute(const std::string& cmdLine)
{
  Context* context = new Context(this, cmdLine);

  // inform the visualization about what is going on
  emit sExecute(context, fromString(cmdLine));

  cmds.push_back(context);
  bool status = context->run();

  cmds.pop_back();
  context->deleteLater();
  return status;
}

Context* Context::executeDetached(const std::string& cmdLine)
{
  Context* context = new Context(this, cmdLine, true);

  emit sExecute(context, fromString(cmdLine));

  cmds.push_back(context);
  context->run();
  return context;
}

Context* Context::executeDetached(Task* task)
{
  Context* context = new Context(this, task);
  task->setContext(context);

  emit sExecute(context, fromString(task->getLabel()));

  cmds.push_back(context);
  context->run();
  return context;
}

void Context::wait()
{
  if(thread)
    thread->wait();
}

bool Context::waitForChildren()
{
  bool status = true;
  for(size_t i = 0; i < cmds.size(); ++i)
  {
    Context* c = cmds[i];
    c->wait();
    status &= c->getStatus();
  }
  return status;
}

void Context::shutdown()
{
  requestApplicationShutdown = true;
  if(parent)
    parent->shutdown();
}

void Context::cancel()
{
  canceled = true;

  for(size_t i = 0; i < cmds.size(); ++i)
    cmds[i]->cancel();

  if(task && thread && thread->isRunning())
    task->cancel();
}

void Context::threadFinished(bool status)
{
  disconnect(thread);
  thread->deleteLater();
  thread = 0;

  this->status = status;
  emit sFinished(status);
}
