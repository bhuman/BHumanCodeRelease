/**
 * @file AnnotationView.cpp
 * @author Andreas Stolpmann
 */

#include "AnnotationView.h"
#include "Controller/RobotConsole.h"
#include "Platform/SystemCall.h"
#include "Platform/Time.h"

#include <algorithm>
#include <QCheckBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QRegExp>
#include <QSettings>
#include <QTableWidgetItem>
#include <QVBoxLayout>

class NumberTableWidgetItem : public QTableWidgetItem
{
  bool operator<(const QTableWidgetItem& other) const override
  {
    return this->text().toFloat() < other.text().toFloat();
  }

public:
  unsigned number;
  void setNumber(unsigned n)
  {
    number = n;
    setText(QString::number(n));
  }
};

struct Row
{
  NumberTableWidgetItem* timeStamp;
  QTableWidgetItem* name;
  QTableWidgetItem* annotation;

  Row() : timeStamp(new NumberTableWidgetItem), name(new QTableWidgetItem), annotation(new QTableWidgetItem) {}
  ~Row()
  {
    if(timeStamp)
      delete timeStamp;
    if(name)
      delete name;
    if(annotation)
      delete annotation;
  }
};

AnnotationWidget::AnnotationWidget(AnnotationView& view, SystemCall::Mode mode)
  : view(view), timeOfLastUpdate(0)
{
  table = new QTableWidget();
  table->setColumnCount(3);
  QStringList headerNames;
  headerNames << "Frame" << "Name" << "Annotation";
  table->setHorizontalHeaderLabels(headerNames);
  table->verticalHeader()->setVisible(false);
  table->verticalHeader()->setSectionResizeMode(QHeaderView::Fixed);
  table->verticalHeader()->setDefaultSectionSize(15);
  table->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);
  table->horizontalHeader()->resizeSection(0, 60);
  table->horizontalHeader()->resizeSection(1, 120);
  table->horizontalHeader()->resizeSection(2, 200);
  table->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table->setAlternatingRowColors(true);
  table->setSortingEnabled(true);
  table->setShowGrid(false);

  if(mode == SystemCall::Mode::logfileReplay)
    QObject::connect(table, SIGNAL(cellDoubleClicked(int, int)), this, SLOT(jumpFrame(int, int)));

  QVBoxLayout* layout = new QVBoxLayout(this);
  QHBoxLayout* filterLayout = new QHBoxLayout();
  filterLayout->addWidget(new QLabel(" Filter: "));
  QLineEdit* filterEdit = new QLineEdit();
  filterEdit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  filterLayout->addWidget(filterEdit);
  filterLayout->setContentsMargins(QMargins());
  layout->setSpacing(0);
  layout->addSpacing(2);
  layout->addLayout(filterLayout);

  QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
  settings.beginGroup(view.fullName);

  QHBoxLayout* checkBoxLayout = new QHBoxLayout();
  checkBoxLayout->setAlignment(Qt::AlignLeft);

  stopCheckBox = new QCheckBox(this);
  stopCheckBox->setText("Stop on Annotation");
  stopCheckBox->setToolTip("Stops the Simulation if a new Annotation arrives which meets the Filter.");
  stopCheckBox->setChecked(settings.value("StopCheckBoxState").toBool());
  QObject::connect(stopCheckBox, SIGNAL(stateChanged(int)), this, SLOT(stopCheckBoxStateChanged(int)));
  checkBoxLayout->addWidget(stopCheckBox);

  useRegExCheckBox = new QCheckBox(this);
  useRegExCheckBox->setText("Allow Regular Expression");
  useRegExCheckBox->setToolTip("Allows the use of regular expressions in the filter box.");
  useRegExCheckBox->setChecked(settings.value("UseRegExCheckBoxState").toBool());
  QObject::connect(useRegExCheckBox, SIGNAL(stateChanged(int)), this, SLOT(useRegExCheckBoxStateChanged(int)));
  checkBoxLayout->addWidget(useRegExCheckBox);

  layout->addSpacing(2);
  layout->addLayout(checkBoxLayout);

  layout->addWidget(table);
  layout->setContentsMargins(QMargins());
  this->setLayout(layout);
  QObject::connect(filterEdit, SIGNAL(textChanged(QString)), this, SLOT(filterChanged(QString)));
  table->horizontalHeader()->restoreState(settings.value("HeaderState").toByteArray());
  table->sortItems(settings.value("SortBy").toInt(), (Qt::SortOrder) settings.value("SortOrder").toInt());
  filter = settings.value("Filter").toString();
  filterEdit->setText(filter);
  settings.endGroup();
}

AnnotationWidget::~AnnotationWidget()
{
  QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
  settings.beginGroup(view.fullName);
  settings.setValue("HeaderState", table->horizontalHeader()->saveState());
  settings.setValue("SortBy", table->horizontalHeader()->sortIndicatorSection());
  settings.setValue("SortOrder", table->horizontalHeader()->sortIndicatorOrder());
  settings.setValue("Filter", filter);
  settings.setValue("StopCheckBoxState", stopCheckBox->isChecked());
  settings.setValue("UseRegExCheckBoxState", useRegExCheckBox->isChecked());
  settings.endGroup();

  for(auto& i : rows)
    if(i.second)
      delete i.second;
}

QWidget* AnnotationWidget::getWidget()
{
  return this;
}

void AnnotationWidget::update()
{
  if(timeOfLastUpdate >= view.info.timeOfLastMessage)
    return;
  timeOfLastUpdate = Time::getCurrentSystemTime();

  table->setUpdatesEnabled(false);
  table->setSortingEnabled(false); //disable sorting while updating to avoid race conditions

  {
    SYNC_WITH(view.info);
    for(const AnnotationInfo::AnnotationData& data : view.info.newAnnotations)
    {
      const QString name = data.name.c_str();
      const QString annotation = data.annotation.c_str();

      if(data.name == "CLEAR")
      {
        table->clear();
        rows.clear();
      }
      else if(rows.find(data.annotationNumber) == rows.end())
      {
        Row* currentRow = new Row();
        rows[data.annotationNumber] = currentRow;
        const int rowCount = table->rowCount();
        table->setRowCount(rowCount + 1);
        table->insertRow(rowCount);
        table->setItem(rowCount, 0, currentRow->timeStamp);
        table->setItem(rowCount, 1, currentRow->name);
        table->setItem(rowCount, 2, currentRow->annotation);

        currentRow->timeStamp->setNumber(data.frame);
        currentRow->name->setText(name);
        currentRow->annotation->setText(annotation);
      }
    }
    view.info.newAnnotations.clear();
  }

  applyFilter();
  table->setSortingEnabled(true);
  table->setUpdatesEnabled(true);
  table->update();
}

void AnnotationWidget::applyFilter()
{
  for(int i = 0; i < table->rowCount(); ++i)
  {
    QTableWidgetItem* nameItem = table->item(i, 1);
    QTableWidgetItem* annotationItem = table->item(i, 2);

    bool hidden = true;

    if(nameItem && annotationItem)
    {
      const QString name = nameItem->text().toLower();
      const QString annotation = annotationItem->text().toLower();

      if(useRegExCheckBox->isChecked())
      {
        QRegExp regex(filter);
        if(regex.exactMatch(name) || regex.exactMatch(annotation))
          hidden = false;
      }
      else
      {
        if(name.contains(filter) || annotation.contains(filter))
          hidden = false;
      }
    }

    table->setRowHidden(i, hidden);
  }
}

void AnnotationWidget::stopCheckBoxStateChanged(int state)
{
  view.stopOnFilter = state == Qt::CheckState::Checked;
}

void AnnotationWidget::useRegExCheckBoxStateChanged(int state)
{
  view.filterIsRegEx = state == Qt::CheckState::Checked;
  SYNC_WITH(view.info);
  applyFilter();
}

void AnnotationWidget::filterChanged(const QString& newFilter)
{
  filter = newFilter.trimmed().toLower();
  SYNC_WITH(view.info);
  view.filter = filter;
  applyFilter();
}

void AnnotationWidget::jumpFrame(int row, int column)
{
  NumberTableWidgetItem* item = (NumberTableWidgetItem*)table->item(row, 0);
  int frame = item->number;
  view.logPlayer.gotoFrame(std::max(std::min(frame, view.logPlayer.numberOfFrames - 1), 0));
}

AnnotationView::AnnotationView(const QString& fullName, AnnotationInfo::ProcessAnnotationInfo& info, LogPlayer& logPlayer, SystemCall::Mode mode, SimRobot::Application* application) :
  fullName(fullName), icon(":/Icons/tag_green.png"), info(info), logPlayer(logPlayer), mode(mode), application(application)
{}

SimRobot::Widget* AnnotationView::createWidget()
{
  return new AnnotationWidget(*this, mode);
}
