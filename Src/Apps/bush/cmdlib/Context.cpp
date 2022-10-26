#include "cmdlib/Context.h"
#include "cmdlib/Commands.h"
#include "cmdlib/Command.h"
#include "Platform/BHAssert.h"
#include <QString>

ContextRunnable::ContextRunnable(QObject* parent) :
  QThread(parent)
{}

void ContextRunnable::run()
{
  bool status = execute();
  emit sFinished(status);
}

TaskRunnable::TaskRunnable(QObject* parent, Task* task) :
  ContextRunnable(parent),
  task(task)
{}

bool TaskRunnable::execute()
{
  return task->execute();
}

Context::Context(Context* parent, const CommandBase* command, const CommandArgs* args) :
  AbstractConsole(parent),
  parent(parent),
  command(command),
  commandArgs(args),
  selectedRobots(parent->getSelectedRobots()),
  selectedTeam(parent->getSelectedTeam())
{}

Context::Context(Context* parent, Task* task) :
  AbstractConsole(parent),
  parent(parent),
  task(task),
  selectedRobots(parent->getSelectedRobots()),
  selectedTeam(parent->getSelectedTeam()),
  detached(true)
{}

Context::Context(const std::vector<Robot*>& selectedRobots,
                 Team* selectedTeam) :
  AbstractConsole(nullptr),
  selectedRobots(selectedRobots),
  selectedTeam(selectedTeam)
{}

Context::~Context()
{
  delete commandArgs;
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
  ASSERT(!command != !task);
  ASSERT(!command == isDetached());
  if(isDetached())
  {
    thread = new TaskRunnable(this, task);

    /* Use DirectConnection here since events with QueuedConnection are only
     * delivered if the receiver thread returns to the event loop, which is not
     * given in every bush thread. */
    connect(thread, &ContextRunnable::sFinished,
            this, &Context::threadFinished,
            Qt::DirectConnection);

    thread->start();
    return true;
  }
  else
  {
    bool status = command->call(*this, commandArgs);
    emit sFinished(status);
    return status;
  }
}

bool Context::execute(const CommandBase* command, const CommandArgs* args)
{
  Context* context = new Context(this, command, args);

  // inform the visualization about what is going on
  emit sExecute(context, QString::fromStdString(command->name));

  cmds.push_back(context);
  bool status = context->run();

  cmds.pop_back();
  context->deleteLater();
  return status;
}

Context* Context::executeDetached(Task* task)
{
  Context* context = new Context(this, task);
  task->setContext(context);

  emit sExecute(context, QString::fromStdString(task->getLabel()));

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
  thread = nullptr;

  this->status = status;
  emit sFinished(status);
}
