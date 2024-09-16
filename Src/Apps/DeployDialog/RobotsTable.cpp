/**
 * @file RobotsTable.cpp
 *
 * This file implements a class that represents a table of all robots. It shows one
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

#include "RobotsTable.h"
#include <iostream>
#include <QComboBox>
#include "../../Util/SimRobot/Src/SimRobot/Theme.h"

RobotsTable::RobotsTable(int rows) : QTableWidget(rows, 11)
{
  const char* headers[][2] =
  {
    {"icons8-web-address-50", "Host id"},
    {"icons8-wired-network-50", "LAN ping time in ms"},
    {"icons8-wi-fi-50", "Wireless ping time in ms"},
    {"icons8-charged-battery-50", "Battery charge in %"},
    {"icons8-temperature-50", "Max joint temperature in °C"},
    {"icons8-log-50", "Number of logs on internal drive"},
    {"icons8-usb-memory-stick-50", "Number of logs on USB drive if present"},
    {"icons8-measurement-tool-50", "New calibration present?"},
    {"b-human", "Status of bhuman service"},
    {"icons8-file-preview-50", "Crash dump present?"}
  };

  setFocusPolicy(Qt::NoFocus);
  setSelectionBehavior(QAbstractItemView::SelectRows);
  setSelectionMode(QAbstractItemView::SingleSelection);
  setDragDropMode(QAbstractItemView::InternalMove);
  setContextMenuPolicy(Qt::CustomContextMenu);
  verticalHeader()->setSectionResizeMode(QHeaderView::Fixed);

  setHorizontalHeaderItem(0, new QTableWidgetItem("  Robot"));
  for(int column = 1; column < columnCount(); ++column)
  {
    setHorizontalHeaderItem(column, new QTableWidgetItem(""));
    QIcon icon(QString(":/Icons/") + headers[column - 1][0] + ".png");
    icon.setIsMask(true);
    model()->setHeaderData(column, Qt::Horizontal, QVariant::fromValue(Theme::updateIcon(this, icon)), Qt::DecorationRole);
    model()->setHeaderData(column, Qt::Horizontal, headers[column - 1][1], Qt::ToolTipRole);
  }
  horizontalHeader()->setSectionResizeMode(QHeaderView::Fixed);
  model()->setHeaderData(0, Qt::Horizontal, QVariant(static_cast<int>(Qt::AlignLeft | Qt::AlignVCenter)), Qt::TextAlignmentRole);

#ifdef MACOS
  QString left("10");
  QString right("10");
#elif defined WINDOWS
  QString left("10");
  QString right("15");
#elif QT_VERSION < QT_VERSION_CHECK(6, 4, 0)
  QString left("19");
  QString right("0");
#else
  QString left("10");
  QString right("15");
#endif

  // The stylesheet for ::section:first is strange, because it influences all headers.
  // It seems as if it would remove the padding between icons and (non-existing) text.
  horizontalHeader()->setStyleSheet("::section {padding-left: " + left + "px; padding-right: " + right + "px}"
                                    "::section:first {padding-left: 0px}");
  setStyleSheet(QString("QTableWidget {gridline-color: ") + (Theme::isDarkMode(this) ? "#404040" : "#e0e0e0") + "}");
}

int RobotsTable::nextVisibleRow(int row) const
{
  do
    ++row;
  while(row < rowCount() && isRowHidden(row));
  return row;
}

int RobotsTable::getPresetRow(int preset) const
{
  int row;
  for(row = 0; row < rowCount() && verticalHeaderItem(row) && verticalHeaderItem(row)->text() != ""; ++row)
  {
    if(verticalHeaderItem(row)->text() == "1")
      --preset;
    if(preset == -1)
      break;
  }
  return row;
}

void RobotsTable::dragEnterEvent(QDragEnterEvent* event)
{
  if(selectedRanges().size())
    sourceRow = selectedRanges()[0].topRow();
  clearSelection();
  QTableWidget::dragEnterEvent(event);
}

void RobotsTable::dragMoveEvent(QDragMoveEvent* event)
{
  const QModelIndex index = indexAt(event->position().toPoint());
  if(index.isValid())
  {
    const bool below = indexAt(event->position().toPoint() + QPoint(0, rowInsertTolerance)).row() != index.row();
    const int nextRow = nextVisibleRow(index.row());
    setDragDropOverwriteMode(verticalHeaderItem(index.row())->text() != ""
                             && (!below || (nextRow != rowCount() && verticalHeaderItem(nextRow)->text() != "")));
  }
  QTableWidget::dragMoveEvent(event);
}

void RobotsTable::dropEvent(QDropEvent* event)
{
  const QModelIndex index = indexAt(event->position().toPoint());
  if(index.isValid())
  {
    selectRow(sourceRow);
    int targetRow = index.row();
    const bool below = indexAt(event->position().toPoint() + QPoint(0, rowInsertTolerance)).row() != targetRow;
    const int nextRow = nextVisibleRow(index.row());
    const bool sourceEmpty = !item(sourceRow, 0) || item(sourceRow, 0)->text() == "";
    const bool targetEmpty = !item(targetRow, 0) || item(targetRow, 0)->text() == "";
    const bool sourceDynamic = verticalHeaderItem(sourceRow)->text() == "";
    const bool targetDynamic = verticalHeaderItem(targetRow)->text() == ""
                               || (below && (nextRow == rowCount() || verticalHeaderItem(nextRow)->text() == ""));

    if(!sourceEmpty) // Suppress dropping empty rows.
    {
      // Should the target be inserted rather than overwritten? This can only be the case
      // in the dynamic area and if the drop position is close to the border between two
      // rows. We can either insert before the current row or after it. The latter is
      // also transformed into an insertion before the target row.
      bool insert = false;
      if(targetDynamic)
      {
        insert = indexAt(event->position().toPoint() - QPoint(0, rowInsertTolerance - 1)).row() != targetRow;
        if(below)
        {
          insert = true;
          targetRow = nextRow;
        }
      }

      // The source can only be removed if it is in the dynamic area. In addition, there
      // should not be information in the target it has to be swapped with.
      const bool remove = sourceDynamic && (insert || (!targetDynamic && targetEmpty));

      QList<QTableWidgetItem*> backup;

      // If the source row it not removed, the original contents of the target row have
      // to be written back there. Make a backup of the target row.
      if(!remove)
        for(int column = 0; column < columnCount(); ++column)
        {
          QTableWidgetItem* item = this->item(targetRow, column);
          backup.append(item ? item->clone() : nullptr);
        }

      // If the target row should be inserted, create a new one that is overwritten by
      // the event handler in the base class.
      if(insert)
      {
        insertRow(targetRow);
        setVerticalHeaderItem(targetRow, new QTableWidgetItem(""));
        if(sourceRow >= targetRow)
          ++sourceRow;

        // Qt does not copy items if target is at the lower window border, so we always copy them.
        for(int column = 0; column < columnCount(); ++column)
          setItem(targetRow, column, item(sourceRow, column) ? item(sourceRow, column)->clone() : nullptr);
      }
      else if(!targetDynamic)
      {
        const int number = verticalHeaderItem(targetRow)->text().toInt() - 1;
        selectedPreset->players[number] = item(sourceRow, 0)->text().toStdString();
      }

      // QTableWidget only seems to support overwriting.
      setDragDropOverwriteMode(true);

      // Simulate dropping on the first column, because otherwise the data is pasted
      // across two rows.
      const QRect rect = visualRect(index.sibling(targetRow, 0));
      QDropEvent e(QPointF(rect.x(), rect.y()),
                   event->dropAction(),
                   event->mimeData(),
                   event->buttons(),
                   event->modifiers(),
                   event->type());
      QTableWidget::dropEvent(&e);

      // If necessary, remove source row or copy the target row back to source
      // if there was one.
      if(remove)
        removeRow(sourceRow);
      else if(!insert)
      {
        for(int column = 0; column < columnCount(); ++column)
          setItem(sourceRow, column, backup[column]);
        if(!sourceDynamic)
        {
          const int number = verticalHeaderItem(sourceRow)->text().toInt() - 1;
          selectedPreset->players[number] = backup[0] ? backup[0]->text().toStdString() : "_";
        }
      }
      else if(!sourceDynamic)
      {
        const int number = verticalHeaderItem(sourceRow)->text().toInt() - 1;
        selectedPreset->players[number] = "_";
      }

      // The original event was not passed to base class, so it must be accepted here.
      event->accept();

      // Clear selection, because it hides color information.
      clearSelection();
      robotAssignmentChanged();
      return;
    }
  }

  // If dropping is rejected, it must be ignored.
  clearSelection();
  event->ignore();
  event->setDropAction(Qt::IgnoreAction);
  QTableWidget::dropEvent(event);
}

void RobotsTable::mouseReleaseEvent(QMouseEvent* event)
{
  QTableWidget::mouseReleaseEvent(event);
  clearSelection();
}

void RobotsTable::changeEvent(QEvent* event)
{
  if(event->type() == QEvent::PaletteChange)
  {
    for(int column = 1; column < columnCount(); ++column)
    {
      QIcon icon = model()->headerData(column, Qt::Horizontal, Qt::DecorationRole).value<QIcon>();
      model()->setHeaderData(column, Qt::Horizontal, QVariant::fromValue(Theme::updateIcon(this, icon)), Qt::DecorationRole);
      setStyleSheet(QString("QTableWidget {gridline-color: ") + (Theme::isDarkMode(this) ? "#404040" : "#e0e0e0") + "}");
    }
#ifdef MACOS
    if(parent())
    {
      const QColor color = palette().text().color();
      for(QComboBox* comboBox : parent()->findChildren<QComboBox*>())
        comboBox->setStyleSheet("color: " + color.name(QColor::HexArgb) + ";");
    }
#endif
  }
  QTableWidget::changeEvent(event);
}

void RobotsTable::setSelectedPreset(Presets::Preset* selectedPreset, int index)
{
  this->selectedPreset = selectedPreset;
  int preset = -1;
  for(int row = 0; row < rowCount() && verticalHeaderItem(row) && verticalHeaderItem(row)->text() != ""; ++row)
  {
    if(verticalHeaderItem(row)->text() == "1")
      ++preset;
    if(index == preset)
      showRow(row);
    else
      hideRow(row);
  }
  robotAssignmentChanged();
}

QSize RobotsTable::sizeHint() const
{
  int width = verticalHeader()->width();
#ifndef MACOS
  width += style()->pixelMetric(QStyle::PM_ScrollBarExtent);
#endif
  for(int column = 0; column < columnCount(); ++column)
    width += columnWidth(column);
  int height = horizontalHeader()->height();
  for(int row = 0; row < rowCount(); ++row)
    height += rowHeight(row);
  return QSize(width + 2, height + 2);
}

void RobotsTable::setCell(const std::string& name, int column, const std::string& value, const QColor& color, const std::string& toolTip)
{
  for(int row = 0; row < rowCount(); ++row)
  {
    const QTableWidgetItem* cell = item(row, 0);
    if(cell && name == cell->text().toStdString())
    {
      QTableWidgetItem* item = new QTableWidgetItem(value.c_str());
      item->setTextAlignment(Qt::AlignCenter);
      if(color.isValid())
        item->setBackground(color);
      item->setData(Qt::ToolTipRole, toolTip.c_str());
      setItem(row, column, item);
    }
  }
}

std::string RobotsTable::getBetterIP(const Robot& robot) const
{
  for(int row = 0; row < rowCount(); ++row)
  {
    const QTableWidgetItem* cell = item(row, 0);
    if(cell && robot.name == cell->text().toStdString())
    {
      const auto getPing = [&](int column)
      {
        const std::string value = item(row, column) ? item(row, column)->text().toStdString() : "";
        return value.empty() ? maxPingTime : std::atoi(value.c_str());
      };
      const int lanPing = getPing(2);
      const int wifiPing = getPing(3);
      return lanPing >= maxPingTime && wifiPing >= maxPingTime ? "" : lanPing <= wifiPing ? robot.lan : robot.wlan;
    }
  }
  return "";
}

void RobotsTable::writeOutput(std::map<std::string, Robot>& robots, bool justIPs) const
{
  int numOfPlayers = 0;
  int numOfReachablePlayers = 0;
  for(const std::string& player : selectedPreset->players)
    if(player != "_")
    {
      ++numOfPlayers;
      if(!getBetterIP(robots[player]).empty())
        ++numOfReachablePlayers;
    }

  for(size_t number = 0; number < selectedPreset->players.size(); ++number)
  {
    const std::string& player = selectedPreset->players[number];
    if(player != "_")
    {
      std::string ip = getBetterIP(robots[player]);
      if(ip.empty())
      {
        if(numOfReachablePlayers == 1)
          continue;
        else
          ip = robots[player].lan;
      }
      if(justIPs)
        std::cout << ip << " ";
      else if(numOfPlayers == 1)
        std::cout << ip << " -p " << number + 1 << " ";
      else
        std::cout << "-r " << number + 1 << " " << ip << " ";
    }
  }

  if(numOfPlayers == 1 && !justIPs)
    std::cout << "-k ";
}

QStringList RobotsTable::getSubstitutes() const
{
  QStringList substitutes;
  for(int row = 0; row < rowCount(); ++row)
    if(verticalHeaderItem(row)->text() == "")
      substitutes.append(item(row, 0)->text());
  return substitutes;
}

void RobotsTable::addPreset(int numOfPlayers)
{
  int row;
  for(row = 0; row < rowCount() && verticalHeaderItem(row) && verticalHeaderItem(row)->text() != ""; ++row);
  while(numOfPlayers > 0)
  {
    insertRow(row);
    setVerticalHeaderItem(row, new QTableWidgetItem(QString::number(numOfPlayers--)));
  }
  robotAssignmentChanged();
}

void RobotsTable::removePreset(int index)
{
  int row = getPresetRow(index);
  do
  {
    if(item(row, 0) && item(row, 0)->text() != "")
    {
      const int targetRow = rowCount();
      insertRow(targetRow);
      setVerticalHeaderItem(targetRow, new QTableWidgetItem(""));
      for(int column = 0; column < columnCount(); ++column)
        setItem(targetRow, column, item(row, column) ? item(row, column)->clone() : nullptr);
    }
    removeRow(row);
  }
  while(row < rowCount() && verticalHeaderItem(row) && verticalHeaderItem(row)->text() != ""
        && verticalHeaderItem(row)->text() != "1");
  robotAssignmentChanged();
}

void RobotsTable::movePreset(int from, int to)
{
  int fromRow = getPresetRow(from);
  int toRow = getPresetRow(from < to ? to + 1 : to);
  do
  {
    insertRow(toRow);

    if(from > to)
      ++fromRow;
    setVerticalHeaderItem(toRow, verticalHeaderItem(fromRow) ? verticalHeaderItem(fromRow)->clone() : nullptr);
    for(int column = 0; column < columnCount(); ++column)
      setItem(toRow, column, item(fromRow, column) ? item(fromRow, column)->clone() : nullptr);
    removeRow(fromRow);
    if(from > to)
      ++toRow;
  }
  while(fromRow < rowCount() && verticalHeaderItem(fromRow) && verticalHeaderItem(fromRow)->text() != ""
        && verticalHeaderItem(fromRow)->text() != "1");
  robotAssignmentChanged();
}
