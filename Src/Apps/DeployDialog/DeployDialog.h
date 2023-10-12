/**
 * @file DeployDialog.h
 *
 * This file declares a class that  represents the main window of the deploy dialog.
 * It also hosts all processes that either ping the robots or poll status updates from them.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "SettingsArea.h"
#include <QProcess>

class DeployDialog : public QDialog
{
  Q_OBJECT

  std::map<std::string, Robot> robots; /**< Static information about all known robots. */
  Presets presets; /**< Presets for the settings teams should be deployed with. */
  RobotsTable* table; /**< The table widget that shows the information about the robots. */
  SettingsArea* settingsArea; /**< The area that displays all the settings. */
  std::vector<QProcess*> processes; /**< All background processes that retrieve information from the robots. */
  static constexpr QColor warnColor = QColor(240, 160, 0); /**< Use orange, because Qt::yellow is too bright for dark mode. */

  static constexpr int statusUpdateTime = 5000; /**< The minimum time between two status updates. */
  static constexpr int pingWarnThreshold = 250; /**< Starting at this value, it is warned about the ping time (in ms). */
  static constexpr int pingCriticalThreshold = 500; /**< Starting at this value, the ping time is critically long (in ms). */
  static constexpr int batteryWarnThreshold = 70; /**< Below this value, it is warned about the battery level. */
  static constexpr int temperatureWarnThreshold = 57; /**< Starting at this value, it is warned about the temperature (in degrees celsius). */
  static constexpr int temperatureCriticalThreshold = 76; /**< Starting at this value, the temperature is critial (in degrees celsius). */

  /**
   * Sets B-Human's "Config" directory as the current one.
   * @param argv0 The first command line argument, i.e. the executable path.
   */
  void gotoConfigDirectory(const char* argv0);

  /**
   * Create the table that shows the information about the robots.
   * @param substitutes The ordering of the substitute robots.
   * @return The robots table widget created.
   */
  RobotsTable* createRobotsTable(const QStringList& substitutes);

  /** Create background processes that ping the robots and gather status information. */
  void createProcesses();

  /**
   * Prepare a command that is executed on the robot.
   * @param ip The ip address of the robot.
   * @param command The command to execute on the robot.
   * @return A command that can be started as a process locally.
   */
  static std::string remoteCommand(const std::string& ip, std::string command);

  /**
   * Prepare a command that is executed locally.
   * @param command The command to be executed.
   * @return A command that can be started as a process.
   */
  static std::string localCommand(const std::string& command);

public:
  /**
   * The constructor sets up the dialog.
   * @param argc The number of command line arguments (including the executable's path).
   * @param argv The command line arguments (including the executable's path).
   */
  DeployDialog(int argc, char* argv[]);

  /** Destructor. */
  ~DeployDialog();

  /**
   * The function is called when the dialog is closed. It writes the output and saves the settings
   * if necessary. It may also refuse to close the dialog.
   * @param reason Either \c Accepted ("Deploy") or \c Rejected ("Cancel").
   */
  void done(int reason) override;
};
