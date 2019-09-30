/**
 * @file Controller/Views/TimeView.cpp
 *
 * Implementation of class TimeView
 *
 * @author Colin Graf
 * @author Arne Böckmann
 */

#include "TimeView.h"
#include "Controller/RoboCupCtrl.h"
#include "Controller/RobotConsole.h"
#include "Platform/Time.h"

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
  QObject::connect(filterEdit, SIGNAL(textChanged(QString)), this, SLOT(filterChanged(QString)));

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
    if(timeView.console.logFile == "" && lastUpdate > lastTimeInfoTimestamp + 1000 && table->rowCount() > 0)
    {
      table->setRowCount(0);
      items.clear();
      table->update();
      return;
    }

    if(timeView.info.timestamp == lastTimeInfoTimestamp)
      return;
    lastTimeInfoTimestamp = timeView.info.timestamp;

    float avgFrequency = -1.0;
    float minDuration = -1.0;
    float maxDuration = -1.0;
    timeView.info.getThreadStatistics(avgFrequency, minDuration, maxDuration);
    frequency->setText(" Freq: " + QString::number(avgFrequency, 'f', 1)
                       + ", Min: " + QString::number(minDuration, 'f', 1) +
                       "ms, Max: " + QString::number(maxDuration, 'f', 1) + "ms");

    table->setUpdatesEnabled(false);
    table->setSortingEnabled(false);//disable sorting while updating to avoid race conditions
    for(const auto& infoPair : timeView.info.infos)
    {
      std::string name = timeView.info.getName(infoPair.first);
      Row* currentRow = nullptr;
      if(items.find(infoPair.first) != items.end())
      {
        //already know this one
        currentRow = items[infoPair.first];
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
        items[infoPair.first] = currentRow;
      }
      float minTime = -1, maxTime = -1, avgTime = -1;
      timeView.info.getStatistics(infoPair.second, minTime, maxTime, avgTime);
      // this row timed out, remove it from the list
      if(infoPair.second.timestamp + 1000 < lastUpdate)
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

  QAction* copyAction = menu->addAction(QIcon(":/Icons/page_copy.png"), tr("&Copy"));
  copyAction->setShortcut(QKeySequence(QKeySequence::Copy));
  copyAction->setStatusTip(tr("Copy the timing data to the clipboard"));
  connect(copyAction, SIGNAL(triggered()), this, SLOT(copy()));

  return menu;
}

void TimeWidget::copy()
{
  QApplication::clipboard()->clear();

  QItemSelectionModel* selected = table->selectionModel();
  QModelIndexList indices = selected->selectedIndexes();

  if(indices.size() < 1)
    return;
  qSort(indices);

  QModelIndex previous = indices.first();
  indices.removeFirst();
  QString selected_text;
  QModelIndex current;

  Q_FOREACH(current, indices)
  {
    QVariant data = table->model()->data(previous);
    QString text = data.toString();
    selected_text.append(text);

    // Add last character for this element based on row or element change
    if(current.row() != previous.row())
      selected_text.append(QLatin1Char('\n'));
    else
      selected_text.append(";");
    previous = current;
  }

  // add last element
  selected_text.append(table->model()->data(current).toString());

  QApplication::clipboard()->setText(selected_text);
}

TimeView::TimeView(const QString& fullName, RobotConsole& console, const TimeInfo& info) :
  fullName(fullName), icon(":/Icons/tag_green.png"), console(console), info(info)
{}

SimRobot::Widget* TimeView::createWidget()
{
  return new TimeWidget(*this);
}
