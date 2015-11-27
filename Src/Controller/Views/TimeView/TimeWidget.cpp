/*
 * File:   TimeWidget.cpp
 * Author: Arne Böckmann
 *
 * Created on May 21, 2013, 8:15 PM
 */

#include "Controller/RobotConsole.h"
#include "TimeWidget.h"
#include <QTableWidgetItem>
#include <QHeaderView>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QSettings>
#include "TimeView.h"
#include "Platform/Thread.h"

/**A simple QTableWidgetItem that enables correct sorting of numbers*/
class NumberTableWidgetItem : public QTableWidgetItem
{
  bool operator<(const QTableWidgetItem &other) const override
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

TimeWidget::TimeWidget(TimeView& timeView) : timeView(timeView), lastTimeInfoTimeStamp(0)
{
  table = new QTableWidget();
  table->setColumnCount(4);
  QStringList headerNames;
  headerNames << "Stopwatch" << "Min" << "Max" << "Avg";
  table->setHorizontalHeaderLabels(headerNames);
  table->verticalHeader()->setVisible(false);
  table->verticalHeader()->setResizeMode(QHeaderView::Fixed);
  table->verticalHeader()->setDefaultSectionSize(15);
  table->horizontalHeader()->setResizeMode(3, QHeaderView::Stretch);
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
  lastTimeInfoTimeStamp = SystemCall::getCurrentSystemTime();
  lastUpdate = SystemCall::getCurrentSystemTime();
  QObject::connect(filterEdit,SIGNAL(textChanged(QString)),this,SLOT(filterChanged(QString)));

  QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
  settings.beginGroup(timeView.fullName);
  table->horizontalHeader()->restoreState(settings.value("HeaderState").toByteArray());
  table->sortItems(settings.value("SortBy").toInt(), (Qt::SortOrder) settings.value("SortOrder").toInt());
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

QWidget* TimeWidget::getWidget() {return this;}
void TimeWidget::update()
{
  SYNC_WITH(timeView.console);
  {
    if(timeView.info.timeStamp == lastTimeInfoTimeStamp)
    {
      return;
    }
    lastTimeInfoTimeStamp = timeView.info.timeStamp;

    if(SystemCall::getTimeSince(lastUpdate) < 200) //only update 5 times per second
    {
      return;
    }
    lastUpdate = SystemCall::getCurrentSystemTime();

    float avgFrequency = -1.0;
    timeView.info.getProcessStatistics(avgFrequency);
    frequency->setText(" Frequency: " + QString::number(avgFrequency));

    table->setUpdatesEnabled(false);
    table->setSortingEnabled(false);//disable sorting while updating to avoid race conditions
    for(TimeInfo::Infos::const_iterator i = timeView.info.infos.begin(), end = timeView.info.infos.end(); i != end; ++i)
    {
      std::string name = timeView.info.getName(i->first);
      Row* currentRow = nullptr;
      if(items.find(i->first) != items.end())
      {//already know this one
        currentRow = items[i->first];
      }
      else
      {//new item
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
        items[i->first] = currentRow;
      }
      float minTime = -1, maxTime = -1, avgTime = -1;
      timeView.info.getStatistics(i->second, minTime, maxTime, avgTime);
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
