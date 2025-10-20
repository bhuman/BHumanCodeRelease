/**
 * @file SimulatedNao/ConsoleRoboCupCtrl.h
 *
 * This file declares the class ConsoleRoboCupCtrl.
 *
 * @author Thomas Röfer
 */

#pragma once

#include <set>
#include <QDir>
#include <QString>

#include "BHToolBar.h"
#include "RoboCupCtrl.h"
#include "RobotConsole.h"

class ConsoleView;
class RemoteConsole;

/**
 * The class implements the SimRobot controller for RoboCup.
 */
class ConsoleRoboCupCtrl : public RoboCupCtrl
{
public:
  DECLARE_SYNC;
  bool calculateImage = true; /**< Decides whether images are calculated by the simulator. */
  unsigned calculateImageFps; /**< Declares the simulated image frame rate. */
  unsigned globalNextImageTimestamp = 0;  /**< The theoretical timestamp of the next image to be calculated shared among all robots to synchronize image calculation. */

private:
  SystemCall::Mode mode; /**< The mode of the robot currently constructed. */
  std::string logFile; /**< The log file name of the most recently created log replay robot. */
  ConsoleView* consoleView; /**< The scene graph object that describes the console widget. */
  std::list<RobotConsole*> selected; /**< The currently selected simulated robots. */
  std::list<RemoteConsole*> remoteRobots; /**< The list of all remote robots. */
  std::list<std::string> textMessages; /**< A list of all text messages received in the current frame. */
  bool newLine = true; /**< States whether the last line of text was finished by a new line. */
  int nesting = 0; /**< The number of recursion level during the execution of console files. */
  std::string callPath; /**< The directory that is the base path for the command "call". */
  std::set<std::string> completion; /**< A list for command completion. */
  std::set<std::string>::const_iterator currentCompletionIndex; /**< Points to the last string that was used for auto completion */
  const DebugRequestTable* debugRequestTable = nullptr; /**< Points to the debug request table used for tab-completion. */
  const ModuleInfo* moduleInfo = nullptr; /**< Points to the solution info used for tab-completion. */
  const std::unordered_map<std::string, RobotConsole::ThreadData>* threadData = nullptr; /**< Thread data used for tab-completion. */
  const RobotConsole::Views* imageViews = nullptr; /**< Points to the map of image views used for tab-completion. */
  const RobotConsole::Views* fieldViews = nullptr; /**< Points to the map of field views used for tab-completion. */
  const RobotConsole::PlotViews* plotViews = nullptr; /**< Points to the map of plot views used for tab-completion. */
  BHToolBar toolBar; /**< The toolbar shown for this controller. */
  QString statusText; /**< The text to be printed in the status bar. */

public:
  /**
   * @param application The interface to SimRobot.
   */
  ConsoleRoboCupCtrl(SimRobot::Application& application);

private:
  ~ConsoleRoboCupCtrl();

public:
  /**
   * The function returns the mode in which the current robot runs.
   * @return The mode for the current robot.
   */
  SystemCall::Mode getMode() const;

  /**
   * Sets a representation.
   * @param representationName The name of the Representation to set.
   * @param representation The representation.
   */
  void setRepresentation(const std::string& representationName, const Streamable& representation);

  /**
   * The function is called when a console command has been entered.
   * @param command The command.
   * @param console Use this console to execute the command.
   * @return Robot selection changed, i.e. parameter console should not be used anymore.
   */
  bool executeConsoleCommand(std::string command, RobotConsole* console = nullptr);

  /**
   * The function is called when the tabulator key is pressed.
   * It can replace the given command line by a new one.
   * @param command The command.
   * @param forward Complete in forward direction.
   * @param nextSection Progress to next section in the command.
   */
  void completeConsoleCommand(std::string& command, bool forward, bool nextSection);

  /**
   * The function is called when a key between a and z or A and z is pressed.
   * If there is only one option how the given command line can be completed
   * it will do that.
   * @param command The current command line.
   */
  void completeConsoleCommandOnLetterEntry(std::string& command);

  /**
   * The function forces an update of the command completion table.
   */
  void updateCommandCompletion() { completion.clear(); }

  /**
   * The function prints a string into the console window.
   * @param text The text to be printed.
   */
  void print(const std::string& text);

  /**
   * The function prints a string into the console window.
   * Future text will be printed on the next line.
   * @param text The text to be printed.
   */
  void printLn(const std::string& text);

  /**
   * The function prints a text as part of a list to the console if it contains a required subtext.
   * @param text The text to print. On the console, it will be followed by a space.
   * @param required The subtext that is search for.
   * @param newLine Should the text be finished by a carriage return?
   */
  void list(const std::string& text, const std::string& required, bool newLine = false);

  /**
   * The function prints a string into the status bar.
   * @param text The text to be printed.
   */
  void printStatusText(const QString& text);

  /**
   * The function translates a debug request string into a simplified version.
   * @param text The text to translate.
   * @return A string that does not contain spaces anymore.
   */
  std::string translate(const std::string& text) const;

  /**
   * Determines the threads that understand a certain debug request.
   * @param threadData The data to search the debug request.
   * @param debugRequest The debug request in translated form, i.e. in camelcase without spaces.
   * @return The names of the threads that understand the request.
   */
  std::vector<std::string> getThreadsFor(const std::unordered_map<std::string, RobotConsole::ThreadData>& threadData,
                                         const std::string& debugRequest) const;

  /**
   * The function sets the DebugRequestTable used by the command completion.
   * @param drt The new debug request table.
   */
  void setDebugRequestTable(const DebugRequestTable& drt) { debugRequestTable = &drt; }

  /**
   * The function sets the solution info used by the command completion.
   * @param moduleInfo The new solution info.
   */
  void setModuleInfo(const ModuleInfo& moduleInfo) { this->moduleInfo = &moduleInfo; }

  /**
   * The function sets the thread data used by the command completion.
   * @param threadData The new thread data.
   */
  void setThreadData(const std::unordered_map<std::string, RobotConsole::ThreadData>& threadData) { this->threadData = &threadData; }

  /**
   * The function sets the map of image views used by the command completion.
   * @param imageViews The map of image views.
   */
  void setImageViews(const RobotConsole::Views& imageViews) { this->imageViews = &imageViews; }

  /**
   * The function sets the map of field views used by the command completion.
   * @param fieldViews The map of field views.
   */
  void setFieldViews(const RobotConsole::Views& fieldViews) { this->fieldViews = &fieldViews; }

  /**
   * The function sets the map of plot views used by the command completion.
   * @param plotViews The map of plot views.
   */
  void setPlotViews(const RobotConsole::PlotViews& plotViews) { this->plotViews = &plotViews; }

  /**
   * The function read text from the stream and prints it to the console.
   * @param stream The text stream.
   */
  void echo(In& stream);

private:
  /**
   * The function executes the specified file.
   * @param name1 The file to execute.
   * @param name2 The file to execute if name1 does not exist.
   * @param printError Print error message if file is not found.
   * @param console Use this console to execute all commands in the file.
   * @return Robot selection changed, i.e. parameter console should not be used anymore.
   */
  bool executeFile(const std::string& name1, const std::string& name2,
                   bool printError, RobotConsole* console);

  /**
   * The function adds a robot with a certain name to the set of selected robots.
   * @param name The name of the robot.
   * @return Does a robot with the specified name exist?
   */
  bool selectRobot(const std::string& name);

  /**
   * The function prints a help text.
   * @param stream The text stream.
   */
  void help(In& stream);

  /**
   * The function handles the console input for the "sc" command.
   * @param stream The stream containing the parameters of "sc".
   * @return Returns true if the parameters were correct.
   */
  bool startRemote(In& stream);

  /**
   * The function handles the console input for the "sl" command.
   * @param stream The stream containing the parameters of "sl".
   * @return Returns true if the parameters were correct.
   */
  bool startLogFile(In& stream);

  /**
   * The function handles the console input for the "ci" command.
   * @param stream The stream containing the parameters of "ci".
   * @return Returns true if the parameters were correct.
   */
  bool calcImage(In&);

  /**
   * The function creates the map for command completion.
   */
  void createCompletion();

  /**
   * Extracts the part of s that shall be used for completion
   */
  std::string handleCompletionString(size_t pos, const std::string& s);

  /**
   * The function adds the tab-completion entries for a command followed by a file name.
   * @param command The command.
   * @param pattern The pattern for the files following the command. The pattern may include a path.
   * @param removeExtension Remove the extensions of the file names.
   */
  void addCompletionFiles(const std::string& command, const std::string& pattern, bool removeExtension = true);

  /**
   * The function is called to initialize the module.
   */
  bool compile() override;

  /**
   * The function is called to create connections to other scene graph objects from other modules.
   */
  void link() override;

  /**
   * The function is called from SimRobot in each simulation step.
   */
  void update() override;

  /**
   * The function is called from SimRobot when Shift+Ctrl+letter was pressed or released.
   * @param key 0..9 for the keys 0..9, 10 for the decimal point, and
   * above for Shift+Ctrl+A-Z.
   * @param pressed Whether the key was pressed or released.
   */
  void pressedKey(int key, bool pressed) override;

  /**
   * The function is called when a movable object has been selected.
   * @param obj The object.
   */
  void selectedObject(const SimRobot::Object& obj) override;

  /**
   * Create the user menu for this module.
   */
  QMenu* createUserMenu() const override { return toolBar.createUserMenu(); }

  /**
   * Show dialog and replace ${}-substrings with user input.
   * ${IP address:} for text field,
   * ${IP address:,192.168.5.11,192.168.5.12,192.168.5.13} for selection.
   */
  void showInputDialog(std::string& command);

  void writeFastConfiguration(std::string fileName);

  void writePerceptOracleConfig(std::string fileName);
};
