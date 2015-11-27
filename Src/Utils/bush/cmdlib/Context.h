#pragma once

#include <vector>
#include <string>
#include <QThread>
#include "Utils/bush/cmdlib/AbstractConsole.h"

class Context;
struct Robot;
class Team;

/** A Interface which allows to define some code, which should be executed in a
 * deferent thread.
 */
class Task
{
  /** Indicates if the Task object should be deleted automatically right after
   * the end of the execution.
   * Default: true
   * NOTE: If it is set to false, the creator of the object is in charge to
   * clean up.
   * IMPORTANT: Disable it if your Task subclass is a QObject or disconnect
   * everything before the end of execute()
   */
  const bool autoDelete;

  /** The context which should be used to interact with the environment. */
  Context* mContext;
public:

  Context& context() { return *mContext; }
  virtual void setContext(Context* context) { mContext = context; }

  /** Constructs a Task.
   * @see autoDelete
   */
  explicit Task(Context& context, bool autoDelete = true)
    : autoDelete(autoDelete),
      mContext(&context)
  {}

  /** Implement your Code here.
   * @return Should return false in case of an error, true otherwise.
   */
  virtual bool execute() = 0;

  virtual void cancel() {}

  /** Interfaces need virtual a destructor. */
  virtual ~Task() = default;

  /** @see autoDelete */
  bool isAutoDelete() { return autoDelete; }

  virtual std::string getLabel() { return ""; }
};

class ContextRunnable : public QThread
{
  Q_OBJECT

public:
  explicit ContextRunnable(QObject* parent);
  void run();

protected:
  virtual bool execute() = 0;

signals:
  void sFinished(bool status);
};

class CommandRunnable : public ContextRunnable
{
  Context* context;
  const std::string cmdLine;

  bool execute();

public:
  CommandRunnable(Context* context, const std::string& cmdLine);
};

class TaskRunnable : public ContextRunnable
{
  Task* task;

  bool execute();

public:
  TaskRunnable(QObject* parent, Task* task);
};

class Context : public AbstractConsole
{
  Q_OBJECT

  Context* parent;
  std::string* cmdLine;
  Task* task;
  ContextRunnable* thread;
  std::vector<Robot*> selectedRobots;
  Team* selectedTeam;
  std::vector<Context*> cmds;

  const bool detached;
  volatile bool canceled;

  // do not copy it
  Context(const Context& other) : detached(false) {}
  Context& operator=(const Context& other) { return *this; }

  Context(Context* parent, const std::string& cmdLine);
  Context(Context* parent, Task* task);
  Context(Context* parent, const std::string& cmdLine, bool detach);

  bool run();
protected:
  volatile bool status;
  volatile bool requestApplicationShutdown;

public:

  Context(const std::vector<Robot*>& selectedRobots,
          Team* selectedTeam);
  ~Context();
  bool execute(const std::string& cmdLine);
  Context* executeDetached(const std::string& cmdLine);
  Context* executeDetached(Task* task);

  void wait();
  bool waitForChildren();
  std::vector<Robot*> getSelectedRobots() const { return selectedRobots; }
  Team* getSelectedTeam() const { return selectedTeam; }
  bool isDetached() const { return detached; }
  bool getStatus() const { return status; }
  bool isCanceled() const { return canceled; }
  void cleanupFinished() { emit sCancelFinished(); }
  void shutdown();
  bool isShutdown() { return requestApplicationShutdown; }

public slots:
  void threadFinished(bool status);
  void cancel();

signals:
  void sExecute(Context* context, const QString& cmdLine);
  void sFinished(bool status);
  void sCancelFinished();
};
