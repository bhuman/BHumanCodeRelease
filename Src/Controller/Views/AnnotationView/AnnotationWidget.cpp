/**
* @file AnnotationWidget.cpp
* @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
*/

#include "AnnotationWidget.h"
#include "AnnotationView.h"
#include "Controller/RobotConsole.h"
#include "Controller/Representations/AnnotationInfo.h"
#include "Platform/SystemCall.h"

#include <algorithm>

#include <QSettings>
#include <QTableWidgetItem>
#include <QHeaderView>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QCheckBox>

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

AnnotationWidget::AnnotationWidget(AnnotationView& view) : view(view), timeOfLastUpdate(0)
{
  table = new QTableWidget();
  table->setColumnCount(3);
  QStringList headerNames;
  headerNames << "Frame" << "Name" << "Annotation";
  table->setHorizontalHeaderLabels(headerNames);
  table->verticalHeader()->setVisible(false);
  table->verticalHeader()->setResizeMode(QHeaderView::Fixed);
  table->verticalHeader()->setDefaultSectionSize(15);
  table->horizontalHeader()->setResizeMode(3, QHeaderView::Stretch);
  table->horizontalHeader()->resizeSection(0, 60);
  table->horizontalHeader()->resizeSection(1, 120);
  table->horizontalHeader()->resizeSection(2, 200);
  table->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table->setAlternatingRowColors(true);
  table->setSortingEnabled(true);
  table->setShowGrid(false);

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

  stopCheckBox = new QCheckBox(this);
  stopCheckBox->setText("Stop on Annotation");
  stopCheckBox->setToolTip("Stops the Simulation if a new Annotation arrives which meets the Filter.");
  stopCheckBox->setChecked(settings.value("StopCheckBoxState").toBool());
  layout->addSpacing(2);
  layout->addWidget(stopCheckBox);
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
  timeOfLastUpdate = SystemCall::getCurrentSystemTime();

  table->setUpdatesEnabled(false);
  table->setSortingEnabled(false); //disable sorting while updating to avoid race conditions

  {
    SYNC_WITH(view.info);
    for(const AnnotationInfo::AnnotationData& data : view.info.newAnnotations)
    {
      const QString name = data.name.c_str();
      const QString annotation = data.annotation.c_str();

      if(stopCheckBox->isChecked() &&
         (name.toLower().contains(filter) || annotation.toLower().contains(filter)))
        view.application->simStop();

      if(rows.find(data.annotationNumber) == rows.end())
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
    QTableWidgetItem* moduleName = table->item(i, 1);
    QTableWidgetItem* annotation = table->item(i, 2);
    if((moduleName && moduleName->text().toLower().contains(filter)) ||
       (annotation && annotation->text().toLower().contains(filter)))
      table->setRowHidden(i, false);
    else
      table->setRowHidden(i, true);
  }
}

void AnnotationWidget::filterChanged(const QString& newFilter)
{
  filter = newFilter.trimmed().toLower();
  SYNC_WITH(view.info);
  applyFilter();
}

void AnnotationWidget::jumpFrame(int row, int column)
{
  if(SystemCall::getMode() == SystemCall::Mode::logfileReplay)
  {
    NumberTableWidgetItem* item = (NumberTableWidgetItem*) table->item(row, 0);
    int frame = item->number;
    view.logPlayer.gotoFrame(std::max(std::min(frame - 1, view.logPlayer.numberOfFrames - 1), 0));
  }
}
