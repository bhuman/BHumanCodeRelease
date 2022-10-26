#pragma once

#include <vector>
#include <string>
#include <QThread>
#include "cmdlib/AbstractConsole.h"

struct CommandArgs;
class CommandBase;
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

class TaskRunnable : public ContextRunnable
{
  Task* task;

  bool execute() override;

public:
  TaskRunnable(QObject* parent, Task* task);
};

class Context : public AbstractConsole
{
  Q_OBJECT

  Context* parent = nullptr;
  const CommandBase* command = nullptr;
  const CommandArgs* commandArgs = nullptr;
  Task* task = nullptr;
  ContextRunnable* thread = nullptr;
  std::vector<Robot*> selectedRobots;
  Team* selectedTeam;
  std::vector<Context*> cmds;

  const bool detached = false;
  volatile bool canceled = false;
  volatile bool status = true;

  // do not copy it
  Context(const Context&) = delete;
  Context& operator=(const Context&) = delete;

  Context(Context* parent, const CommandBase* command, const CommandArgs* args);
  Context(Context* parent, Task* task);

  bool run();

public:

  Context(const std::vector<Robot*>& selectedRobots,
          Team* selectedTeam);
  ~Context();
  bool execute(const CommandBase* command, const CommandArgs* args);
  Context* executeDetached(Task* task);

  void wait();
  bool waitForChildren();
  const std::vector<Robot*>& getSelectedRobots() const { return selectedRobots; }
  Team* getSelectedTeam() const { return selectedTeam; }
  bool isDetached() const { return detached; }
  bool getStatus() const { return status; }
  bool isCanceled() const { return canceled; }
  void cleanupFinished() { emit sCancelFinished(); }

public slots:
  void threadFinished(bool status);
  void cancel();

signals:
  void sExecute(Context* context, const QString& cmdLine);
  void sFinished(bool status);
  void sCancelFinished();
};
