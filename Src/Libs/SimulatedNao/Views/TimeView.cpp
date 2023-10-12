/**
 * @file SimulatedNao/Views/TimeView.cpp
 *
 * Implementation of class TimeView
 *
 * @author Colin Graf
 * @author Arne Böckmann
 */

#include "TimeView.h"
#include "SimulatedNao/RoboCupCtrl.h"
#include "SimulatedNao/RobotConsole.h"
#include "Platform/Time.h"

#include <algorithm>
#include <QApplication>
#include <QClipboard>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMenu>
#include <QSettings>
#include <QTableWidgetItem>
#include <QVBoxLayout>

/**A simple QTableWidgetItem that enables correct sorting of numbers*/
class NumberTableWidgetItem : public QTableWidgetItem
{
  bool operator<(const QTableWidgetItem& other) const override
  {
    return this->text().toFloat() < other.text().toFloat();
  }
};

struct Row
{
  QTableWidgetItem* name; //QTableWidgetItem uses lexical sorting, that is ok for the name column
  NumberTableWidgetItem* min;
  NumberTableWidgetItem* max;
  NumberTableWidgetItem* avg;
};

TimeWidget::TimeWidget(TimeView& timeView) : timeView(timeView)
{
  setFocusPolicy(Qt::StrongFocus);

  table = new QTableWidget();
  table->setColumnCount(4);
  QStringList headerNames;
  headerNames << "Stopwatch" << "Min" << "Max" << "Avg";
  table->setHorizontalHeaderLabels(headerNames);
  table->verticalHeader()->setVisible(false);
  table->verticalHeader()->setSectionResizeMode(QHeaderView::Fixed);
  table->verticalHeader()->setDefaultSectionSize(15);
  table->horizontalHeader()->setSectionResizeMode(3, QHeaderView::Stretch);
  table->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table->setAlternatingRowColors(true);
  table->setSortingEnabled(true);
  table->setShowGrid(false);

  //initial sort by name
  QVBoxLayout* layout = new QVBoxLayout(this);
  QHBoxLayout* filterLayout = new QHBoxLayout();
  filterLayout->addWidget(new QLabel(" Filter: "));
  QLineEdit* filterEdit = new QLineEdit();
  filterEdit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  filterLayout->addWidget(filterEdit);
  filterLayout->setContentsMargins(QMargins());
  frequency = new QLabel(" Frequency: ");
  layout->setSpacing(0);
  layout->addSpacing(2);
  layout->addWidget(frequency);
  layout->addSpacing(2);
  layout->addLayout(filterLayout);
  layout->addWidget(table);
  layout->setContentsMargins(QMargins());
  this->setLayout(layout);
  lastTimeInfoTimestamp = Time::getCurrentSystemTime();
  lastUpdate = Time::getCurrentSystemTime();
  QObject::connect(filterEdit, &QLineEdit::textChanged, this, &TimeWidget::filterChanged);

  QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
  settings.beginGroup(timeView.fullName);
  table->horizontalHeader()->restoreState(settings.value("HeaderState").toByteArray());
  table->sortItems(settings.value("SortBy").toInt(), static_cast<Qt::SortOrder>(settings.value("SortOrder").toInt()));
  filter = settings.value("Filter").toString();
  filterEdit->setText(filter);
  settings.endGroup();
}

TimeWidget::~TimeWidget()
{
  QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
  settings.beginGroup(timeView.fullName);
  settings.setValue("HeaderState", table->horizontalHeader()->saveState());
  settings.setValue("SortBy", table->horizontalHeader()->sortIndicatorSection());
  settings.setValue("SortOrder", table->horizontalHeader()->sortIndicatorOrder());
  settings.setValue("Filter", filter);
  settings.endGroup();
}

QWidget* TimeWidget::getWidget() { return this; }

void TimeWidget::update()
{
  SYNC_WITH(timeView.console);
  {
    if(Time::getTimeSince(lastUpdate) < 200) //only update 5 times per second
      return;
    lastUpdate = Time::getCurrentSystemTime();

    // if timing info is lost for more than 1s, clear timing table
    if(timeView.console.mode != SystemCall::logFileReplay && lastUpdate > lastTimeInfoTimestamp + 1000 && table->rowCount() > 0)
    {
      table->setRowCount(0);
      items.clear();
      table->update();
      return;
    }

    if(timeView.info.timestamp == lastTimeInfoTimestamp)
      return;
    lastTimeInfoTimestamp = timeView.info.timestamp;

    float avgFrequency = -1.f;
    float minDuration = -1.f;
    float maxDuration = -1.f;
    timeView.info.getThreadStatistics(avgFrequency, minDuration, maxDuration);
    frequency->setText(" Freq: " + QString::number(avgFrequency, 'f', 1)
                       + ", Min: " + QString::number(minDuration, 'f', 1) +
                       "ms, Max: " + QString::number(maxDuration, 'f', 1) + "ms");

    table->setUpdatesEnabled(false);
    table->setSortingEnabled(false);//disable sorting while updating to avoid race conditions
    for(const auto& [id, info] : timeView.info.infos)
    {
      std::string name = timeView.info.getName(id);
      Row* currentRow = nullptr;
      if(items.find(id) != items.end())
      {
        //already know this one
        currentRow = items[id];
      }
      else
      {
        //new item
        currentRow = new Row;
        currentRow->avg = new NumberTableWidgetItem();
        currentRow->max = new NumberTableWidgetItem();
        currentRow->min = new NumberTableWidgetItem();
        currentRow->name = new QTableWidgetItem();
        const int rowCount = table->rowCount();
        table->setRowCount(rowCount + 1);
        table->setItem(rowCount, 0, currentRow->name);
        table->setItem(rowCount, 1, currentRow->min);
        table->setItem(rowCount, 2, currentRow->max);
        table->setItem(rowCount, 3, currentRow->avg);
        items[id] = currentRow;
      }
      float minTime = -1, maxTime = -1, avgTime = -1;
      timeView.info.getStatistics(info, minTime, maxTime, avgTime);
      // this row timed out, remove it from the list
      if(info.timestamp + 1000 < lastUpdate)
        maxTime = 0;
      currentRow->avg->setText(QString::number(avgTime));
      currentRow->min->setText(QString::number(minTime));
      currentRow->max->setText(QString::number(maxTime));
      currentRow->name->setText(QString(name.c_str())); //refresh name every time to eliminate unknown
    }
  }
  applyFilter();
  table->setSortingEnabled(true);
  table->setUpdatesEnabled(true);
  table->update();
}

void TimeWidget::filterChanged(const QString& newFilter)
{
  filter = newFilter;
}

void TimeWidget::applyFilter()
{
  for(int i = 0; i < table->rowCount(); ++i)
  {
    QTableWidgetItem* item = table->item(i, 0); //assuming that column 0 is the name column
    QTableWidgetItem* maximum = table->item(i, 2); //assuming that column 2 is the maximum column
    table->setRowHidden(i, !item || !maximum || maximum->text() == "0" || !item->text().toLower().contains(filter.trimmed().toLower()));
  }
}

QMenu* TimeWidget::createEditMenu() const
{
  QMenu* menu = new QMenu(tr("&Timing"));

  QIcon copyIcon(":/Icons/icons8-copy-to-clipboard-50.png");
  copyIcon.setIsMask(true);
  QAction* copyAction = menu->addAction(copyIcon, tr("&Copy"));
  copyAction->setShortcut(QKeySequence(QKeySequence::Copy));
  copyAction->setStatusTip(tr("Copy the timing data to the clipboard"));
  connect(copyAction, &QAction::triggered, this, &TimeWidget::copy);

  return menu;
}

void TimeWidget::copy()
{
  QApplication::clipboard()->clear();

  QItemSelectionModel* selected = table->selectionModel();
  QModelIndexList indices = selected->selectedIndexes();

  if(indices.size() < 1)
    return;
  std::sort(indices.begin(), indices.end());

  const auto first = indices.takeFirst();
  QString selectedText = table->model()->data(first).toString();
  auto previousRow = first.row();
  for(const QModelIndex& current : indices)
  {
    // Add separator for this element based on row or element change
    if(current.row() != previousRow)
    {
      previousRow = current.row();
      selectedText.append(QLatin1Char('\n'));
    }
    else
      selectedText.append(";");
    selectedText.append(table->model()->data(current).toString());
  }

  QApplication::clipboard()->setText(selectedText);
}

TimeView::TimeView(const QString& fullName, RobotConsole& console, const TimeInfo& info) :
  fullName(fullName), icon(":/Icons/icons8-stopwatch-50.png"), console(console), info(info)
{
  icon.setIsMask(true);
}

SimRobot::Widget* TimeView::createWidget()
{
  return new TimeWidget(*this);
}
