/**
 * @file AnnotationView.cpp
 * @author Andreas Stolpmann
 */

#include "AnnotationView.h"
#include "SimulatedNao/LogPlayer.h"
#include "SimulatedNao/RoboCupCtrl.h"
#include "SimulatedNao/RobotConsole.h"
#include "Platform/SystemCall.h"
#include "Platform/Time.h"
#ifdef MACOS
#include "AppleHelper/Helper.h"
#endif

#include <algorithm>
#include <QCheckBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QRegularExpression>
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

  NumberTableWidgetItem(unsigned number) : QTableWidgetItem(QString::number(number)), number(number) {}
};

AnnotationWidget::AnnotationWidget(AnnotationView& view, SystemCall::Mode mode)
  : view(view), timeOfLastUpdate(0)
{
  setFocusPolicy(Qt::StrongFocus);

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

  if(mode == SystemCall::Mode::logFileReplay)
    QObject::connect(table, &QTableWidget::cellDoubleClicked, [&](int row, int)
    {
      NumberTableWidgetItem* item = static_cast<NumberTableWidgetItem*>(table->item(row, 0));
      view.console.handleConsole(("log goto " + std::to_string(item->number)).c_str());
    });

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
  stopCheckBox->setChecked(view.stopOnFilter = settings.value("StopCheckBoxState").toBool());
  QObject::connect(stopCheckBox, &QCheckBox::stateChanged, [&](int state) {view.stopOnFilter = state == Qt::CheckState::Checked;});
  checkBoxLayout->addWidget(stopCheckBox);
#ifdef MACOS
  checkBoxLayout->addSpacing(16);
#endif

  useRegExCheckBox = new QCheckBox(this);
  useRegExCheckBox->setText("Allow Regular Expression");
  useRegExCheckBox->setToolTip("Allows the use of regular expressions in the filter box.");
  useRegExCheckBox->setChecked(view.filterIsRegEx = settings.value("UseRegExCheckBoxState").toBool());
  QObject::connect(useRegExCheckBox, &QCheckBox::stateChanged, [&](int state)
  {
    view.filterIsRegEx = state == Qt::CheckState::Checked;
    SYNC_WITH(view.info);
    applyFilter();
  });
  checkBoxLayout->addWidget(useRegExCheckBox);

  layout->addSpacing(2);
  layout->addLayout(checkBoxLayout);

  layout->addWidget(table);
  layout->setContentsMargins(QMargins());
  this->setLayout(layout);
  QObject::connect(filterEdit, &QLineEdit::textChanged, [&](const QString& newFilter)
  {
    filter = newFilter.trimmed().toLower();
    SYNC_WITH(view.info);
    view.filter = filter;
    applyFilter();
  });
  table->horizontalHeader()->restoreState(settings.value("HeaderState").toByteArray());
  table->sortItems(settings.value("SortBy").toInt(), static_cast<Qt::SortOrder>(settings.value("SortOrder").toInt()));
  filter = settings.value("Filter").toString();
  filterEdit->setText(filter);
  settings.endGroup();

  view.info.registerListener(this);
}

AnnotationWidget::~AnnotationWidget()
{
  view.info.unregisterListener(this);

  QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
  settings.beginGroup(view.fullName);
  settings.setValue("HeaderState", table->horizontalHeader()->saveState());
  settings.setValue("SortBy", table->horizontalHeader()->sortIndicatorSection());
  settings.setValue("SortOrder", table->horizontalHeader()->sortIndicatorOrder());
  settings.setValue("Filter", filter);
  settings.setValue("StopCheckBoxState", stopCheckBox->isChecked());
  settings.setValue("UseRegExCheckBoxState", useRegExCheckBox->isChecked());
  settings.endGroup();
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

  {
    SYNC_WITH(view.info);
    if(view.info.newAnnotations.size() > 5) // Adding multiple rows is slow with sorting activated
      table->setSortingEnabled(false);

    for(const AnnotationInfo::AnnotationData& data : view.info.newAnnotations)
    {
      const QString name = data.name.c_str();
      const QString annotation = data.annotation.c_str();

      if(data.name == "CLEAR")
      {
        table->clearContents();
        annotationNumbers.clear();
      }
      else if(annotationNumbers.find(data.annotationNumber) == annotationNumbers.end())
      {
        annotationNumbers.insert(data.annotationNumber);
        const int rowCount = table->rowCount();
        table->insertRow(rowCount);
        QTableWidgetItem* item = new NumberTableWidgetItem(data.frame);
        table->setItem(rowCount, 0, item);
        table->setItem(item->row(), 1, new QTableWidgetItem(name));
        table->setItem(item->row(), 2, new QTableWidgetItem(annotation));
      }
    }
    view.info.newAnnotations.clear();
  }

  applyFilter();
  table->setSortingEnabled(true);
  table->setUpdatesEnabled(true);
  table->update();
}

void AnnotationWidget::handleAnnotation(const AnnotationInfo::AnnotationData& data)
{
  if(view.stopOnFilter)
  {
    const QString name = QString(data.name.c_str()).toLower();
    const QString annotation = QString(data.annotation.c_str()).toLower();

    if(view.filterIsRegEx)
    {
      QRegularExpression regex(view.filter);
      if(regex.globalMatch(name).hasNext() || regex.globalMatch(annotation).hasNext())
        simStop();
    }
    else if(name.contains(view.filter) || annotation.contains(view.filter))
      simStop();
  }
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
        QRegularExpression regex(filter);
        if(regex.globalMatch(name).hasNext() || regex.globalMatch(annotation).hasNext())
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

void AnnotationWidget::simStop()
{
#ifdef MACOS
  runInMainThread([this]{view.application->simStop();});
#else
  view.application->simStop();
#endif
}

AnnotationView::AnnotationView(const QString& fullName, AnnotationInfo& info, RobotConsole& console,LogPlayer& logPlayer,
                               SystemCall::Mode mode, SimRobot::Application* application) :
  fullName(fullName), icon(":/Icons/icons8-price-tag-50.png"), info(info), console(console), logPlayer(logPlayer), mode(mode), application(application)
{
  icon.setIsMask(true);
}

SimRobot::Widget* AnnotationView::createWidget()
{
  return new AnnotationWidget(*this, mode);
}
