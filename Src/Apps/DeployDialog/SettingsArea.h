/**
 * @file SettingsArea.h
 *
 * This file declares a class that represents the settings area on the right side
 * of the dialog. It consists of two tab groups and a few buttons.
 *
 * @author Thomas Röfer
 */

#pragma once

#include <functional>
#include <map>
#include <QDialog>
#include <QLineEdit>
#include <QWidget>
#include <QSettings>
#include "Presets.h"
#include "Robot.h"
#include "RobotsTable.h"
#include "Teams.h"

class SettingsArea : public QWidget
{
  Q_OBJECT

  static constexpr int settingsFieldWidth = 120; /**< The fixed width of some controls. */
  Presets& presets; /**< The presets for the different teams. */
  RobotsTable* table; /**< The table with information about the state of the robots. */
  int presetIndex; /**< The index of the selected preset. */
  enum Mode {robots, image, logs} mode = robots; /**< The deployment mode. */
  bool restart; /**< Restart bhuman after deploying? */
  bool deleteLogs; /**< Delete internal logs while deploying? */
  int playerNumber; /**< Default player number for image. */
  bool reboot; /**< Reboot after flashing image? */
  bool usbCheck; /**< Check for USB drive before starting. */
  bool date; /**< Add date to name of image file. */
  enum LogsMode {download, downloadAndDelete, justDelete} logsMode; /**< The mode for downloading logs. */
  bool close; /**< Close dialog after action. */
  std::map<std::string, int> teams; /**< All team names with their team number. */
  Presets::Preset* selectedPreset = nullptr; /**< The currently selected preset. */

  /**
   * Create a tab group for presets.
   * @return The tab group.
   */
  QWidget* createPresetTabs();

  /**
   * Create a tab for a preset.
   * @param preset The preset.
   * @return The tab.
   */
  QWidget* createPresetTab(Presets::Preset* preset);

  /**
   * Create a tab for deploying to robots.
   * @return The tab.
   */
  QWidget* createRobotsTab();

  /**
   * Create a tab for deploying to an image.
   * @return The tab.
   */
  QWidget* createImageTab();

  /**
   * Create a tab for handling logs.
   * @param updateDeployButton A function that updates the text of the deploy button.
   * @return The tab.
   */
  QWidget* createLogsTab(const std::function<void()>& updateDeployButton);

public:
  /**
   * Constructs the settings area.
   * @param presets The deploy presets for the different teams.
   * @param dialog The deploy dialog. Only used to call \c accept and \c reject.
   * @param table The table containing the information about the robots. Is informed about the selected preset.
   * @param settings The settings to read.
   */
  SettingsArea(Presets& presets, QDialog* dialog, RobotsTable* table, const QSettings& settings);

  /**
   * Write parameters for deploy script to console.
   * @param robots Network information about all robots.
   */
  void writeOutput(std::map<std::string, Robot>& robots) const;

  /**
   * Have the settings been modified compared to the ones that were initially loaded?
   * @param settings The original settings.
   * @return Have they changed?
   */
  bool modified(const QSettings& settings) const;

  /**
   * Save settings.
   * @param settings The object that stores them.
   * @return Close dialog afterwards?
   */
  bool save(QSettings& settings) const;
};
