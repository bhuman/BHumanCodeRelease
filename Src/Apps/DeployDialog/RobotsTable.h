/**
 * @file RobotsWidget.h
 *
 * This file declares a class that implements a table of all robots. It shows one
 * robot per row with status information. The table is split in a static part and a
 * dynamic part. The static part always has the same number of rows and
 * represents the different player numbers that exist in a team. Robots in this
 * part will be deployed. The dynamic part contains robot thats were not assigned
 * to a player number yet. It grows and shrinks with the number of robots placed
 * into it. Each preset has its own static part. These rows are hidden if the
 * corresponding preset is not currently selected.
 *
 * @author Thomas Röfer
 */

#pragma once

#include <QDragMoveEvent>
#include <QDropEvent>
#include <QHeaderView>
#include <QTableWidget>
#include "Presets.h"
#include "Robot.h"

class RobotsTable : public QTableWidget
{
  Q_OBJECT

  static constexpr int rowInsertTolerance = 6; /**< Number of pixel around row separator dropped rows are inserted. A guessed value that works on macOS. */
  static constexpr int maxPingTime = 2000; /**< The maximum ping time. Times longer or equal to it are considered "not reachable". */
  int sourceRow = -1; /**< The row selected during drag and drop. */
  Presets::Preset* selectedPreset = nullptr; /**< The currently selected preset. */

  /**
   * Determine the next row that is not hidden.
   * @param row The row the successor of which is determined.
   * @return The next non-hidden row. If there is now \c rowCount is returned.
   */
  int nextVisibleRow(int row) const;

  /**
   * Return the index of the first row of a preset.
   * @param preset The number of the preset.
   * @return The index of the first row.
   */
  int getPresetRow(int preset) const;

protected:
  /**
   * Remember selected row and clean selection.
   * @param event Qt's event while dragging.
   */
  void dragEnterEvent(QDragEnterEvent* event) override;

  /**
   * Set the overwrite mode depending on whether dragging over the static or the
   * dynamic area. The mode decides whether an insertion line is shown or not
   * between rows.
   * @param event Qt's event while dragging.
   */
  void dragMoveEvent(QDragMoveEvent* event) override;

  /**
   * Handle a dropped row.
   * @param event Qt's event when dropping.
   */
  void dropEvent(QDropEvent* event) override;

  /**
   * Clear selection if a row is just clicked without dragging.
   * @param event Qt's mouse release event.
   */
  void mouseReleaseEvent(QMouseEvent* event) override;

  /**
   * Handle changes in the global color palette (dark vs. light mode).
   * @param event Qt's event when something changes.
   */
  void changeEvent(QEvent* event) override;

public:
  /**
   * Creates a new table for robot information.
   * @param rows The initual number of rows for the table.
   */
  RobotsTable(int rows);

  /**
   * Computed preferred size for this table. This involves some guessing.
   * @param The preferred size.
   */
  QSize sizeHint() const override;

  /**
   * Set the selected preset.
   * @param selectedPreset The currently selected preset.
   * @param index The index of the preset, starting with 0 for the first one.
   */
  void setSelectedPreset(Presets::Preset* selectedPreset, int index);

  /**
   * Set the value of a cell for a specific robot.
   * @param name The name of the robot. The row is derived from this name.
   * @param column The column to be set for the robot.
   * @param value The value that is set.
   * @param color The background color of the cell. An invalid (empty) color uses the system default.
   * @param toolTip The tool tip for the cell.
   */
  void setCell(const std::string& name, int column, const std::string& value, const QColor& color = QColor(), const std::string& toolTip = "");

  /**
   * Get the ip with the shorter ping time for a certain robot.
   * @param robot The robot.
   * @return The ip with the shorter ping time or the empty string
   *         if both ping times are too long.
   */
  std::string getBetterIP(const Robot& robot) const;

  /**
   * Write robot-related parameters for deploy script to console.
   * @param robots Network information about all robots.
   * @param justIPs Just write the IPs of the robots.
   */
  void writeOutput(std::map<std::string, Robot>& robots, bool justIPs) const;

  /**
   * Return the names of the robots used as substitutes in the order they
   * appear in.
   * @return The list of robot names.
   */
  QStringList getSubstitutes() const;

  /**
   * Adds rows for a new preset.
   * @param numOfPlayers The number of rows that should be inserted.
   */
  void addPreset(int numOfPlayers);

  /**
   * Remove a preset. The robots assigned to the preset will be moved
   * to the end of the table.
   * @param index The sequential number of the preset.
   */
  void removePreset(int index);

  /**
   * Move all robots of a preset.
   * @param from The original index of the preset.
   * @param to The index after it was moved.
   */
  void movePreset(int from, int to);

signals:
  /** The set of assigned robots changed. */
  void robotAssignmentChanged();
};
