/**
 * @file ProcessRunner.h
 *
 * Contains the definitions of ProcessRunner and RemoteWriteProcessRunner.
 *
 * @author <a href="mailto::ojlr@informatik.uni-bremen.de">Ole Jan Lars Riemann</a>
 */

#pragma once

#include <QString>
#include <QProcess>
#include <QProcessEnvironment>
#include <string>

class Context;

/**
 * @class ProcessRunner Handles the output of a QProcess in a simple way.
 *
 * Use one of the constructors to define, which parameters should be passed to
 * the run method of the run method of the QProcess object and call run.
 * The status and the return value can be read with error() or directly at the
 * QProcess object, which can be requested with getProcess().
 *
 * NOTE: You have to call run right after the construction of an object of this
 * class and before you call any other method.
 */
class ProcessRunner : public QObject
{
  Q_OBJECT

  /** The process started by the runner. */
  QProcess* process;

  /** The program which should be executed. It represents the entire command
   * line if arguments is <em>0</em>.
   */
  QString program;

  /** The argument list of the program. Can be 0. */
  QStringList* arguments;

  /** Contains the output of the process after a call of run. */
  QStringList output;

  /** Initializes the runner. */
  void init();

public:

  /**
   * Constructs a new ProcessRunner which executes the delivered command.
   * @param command The command line which should be executed in a new process.
   */
  explicit ProcessRunner(const QString& command);

  /**
   * Constructs a new ProcessRunner which excecutes the delivered program with
   * the given arguments.
   * @param program The program which should be executed.
   * @param arguments The arguments which should be passed to the program.
   */
  ProcessRunner(const std::string& program, const QStringList& arguments);

  /**
   * Constructs a new ProcessRunner which excecutes the delivered program with
   * the given arguments.
   * @param program The program which should be executed.
   * @param arguments The arguments which should be passed to the program.
   */
  ProcessRunner(const QString& program, const QStringList& arguments);

  /**
   * Constructs a new ProcessRunner which executes the delivered program in the
   * given context with the delivered arguments.
   * @param context The context, which should be used to print the output of the
   *                process.
   * @param program The program which should be executed.
   * @param arguments The arguments which should be passed to the program.
   */
  ProcessRunner(Context& context, const std::string& program, const QStringList& arguments);

  /**
   * Constructs a new ProcessRunner which executes the delivered program in the
   * given context with the delivered arguments.
   * @param context The context, which should be used to print the output of the
   *                process.
   * @param program The program which should be executed.
   * @param arguments The arguments which should be passed to the program.
   */
  ProcessRunner(Context& context, const QString& program, const QStringList& arguments);

  /**
   * Constructs a new ProcessRunner which executes the delivered command line
   * in a new process.
   * @param context The context, which should be used to print the output of the
   *                process.
   * @param context The context, which should be used to print the output of the
   *                process.
   * @param command The command line which should be executed in a new process.
   */
  ProcessRunner(Context& context, const std::string& command);

  /**
   * Constructs a new ProcessRunner which executes the delivered command line
   * in a new process.
   * @param context The context, which should be used to print the output of the
   *                process.
   * @param context The context, which should be used to print the output of the
   *                process.
   * @param command The command line which should be executed in a new process.
   */
  ProcessRunner(Context& context, const QString& command);

  /**
   * The destructor which deletes the argument list and the process, if they are
   * not 0. For the destruction of the process, deleteLater() is used to prevent
   * segmentation faults within the delivery of asynchronous signals.
   */
  ~ProcessRunner();

  ProcessRunner& operator=(const ProcessRunner& other);

  /**
   * Constructs a new QProcess, connects the output signals to the context, if
   * present and calls one of the run methods of the QProcess.
   * If no arguments are given, the QProcess run method without an argument list
   * is called and it is assumed that the delivered command line string fulfill
   * the requirements of the method.
   */
  void run();

  /**
   * Tells the QProcess, to terminate.
   * On different platforms this will be handled on different ways.
   */
  void stop();

  /**
   * Indicates if an error occurred during the execution.
   * @return True if an error occurred, false otherwise.
   */
  bool error() const
  {
    return process->exitStatus() == QProcess::CrashExit
           || process->exitCode() != 0;
  }

  /**
   * Returns a pointer to the QProcess object, which is used by the runner.
   * Do not delete it since this is done by the runner.
   * @return The underlying QProcess object.
   */
  QProcess* getProcess()
  {
    if(!process)
      process = new QProcess();
    return process;
  };

  /**
   * @return The output of the process.
   */
  QString getOutput();

  void setContext(Context& context) { this->context = &context; }

protected:
  /**
   * The context which is used to print the output of the process.
   * The context can be 0, which causes the runner to be quiet.
   */
  Context* context;

  /**
   * Overwrite this method to write something to stdin of the started process.
   * It is called by the run method in the right moment.
   * Do not call something like waitForFinished or close on the process since
   * run will do that after the call of interact.
   */
  virtual void interact(QProcess* process) {}

public slots:

  /**
   * Will be connected to the readyReadStandardError signal of the created
   * process. Calls errorLine() of the context, if it is not 0 and appends the
   * output of the process to output.
   */
  void updateError();

  /**
   * Will be connected to the readyReadStandardOutput signal of the created
   * process. Calls printLine() of the context, if it is not 0 and appends the
   * output of the process to output.
   */
  void updateText();
};

/**
 * @class RemoteWriteProcessRunner A ProcessRunner which can be used to pass
 * data directly to stdin of the executed process.
 */
class RemoteWriteProcessRunner : public ProcessRunner
{
  /**
   * The data, which should be passed to the process.
   */
  const QByteArray& data;

protected:

  /**
   * This method is called by run() and passes the data to the process.
   * @param process The process to which the data should be passed to.
   */
  void interact(QProcess* process);

public:
  /**
   * Constructs a new RemoteWriteProcessRunner.
   * @param context The context, which should be used to print the output of the
   *                process.
   * @param program The program which should be executed.
   * @param arguments The arguments which should be passed to the program.
   * @param data The data which should be passed to the process.
   */
  RemoteWriteProcessRunner(Context& context,
                           const QString& program,
                           const QStringList& arguments,
                           const QByteArray& data);
};
