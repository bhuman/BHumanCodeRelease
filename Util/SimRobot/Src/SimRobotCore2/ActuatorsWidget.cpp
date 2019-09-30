/**
* @file ActuatorsWidget.cpp
* Implementation of class ActuatorsWidget and ActuatorWidget
*/

#include <QLabel>
#include <QVBoxLayout>
#include <QSlider>
#include <QLineEdit>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QResizeEvent>
#include <QScrollArea>
#include <QSettings>

#include "Simulation/Simulation.h"
#include "Simulation/UserInput.h"
#include "Tools/Math/Constants.h"
#include "ActuatorsWidget.h"
#include "CoreModule.h"

static inline float toDeg(float angleInRad)
{ return angleInRad * 180.0f / pi;}
static inline float toRad(float angleInDegree)
{ return angleInDegree * pi / 180.0f;}

SimRobot::Widget* ActuatorsObject::createWidget()
{
  return new ActuatorsWidget();
}

/**
* @class FlowLayout
* A vertical flow layout that horizontally spreads items if there is enough space.
* It is based on http://qt-project.org/doc/qt-4.8/layouts-flowlayout.html
*/
class FlowLayout : public QLayout
{
public:
  ~FlowLayout();
  void addItem(QLayoutItem *item) {itemList.append(item);}
  int horizontalSpacing() const {return 0;}
#ifdef MACOS
  int verticalSpacing() const {return -5;}
#else
  int verticalSpacing() const {return 0;}
#endif
  Qt::Orientations expandingDirections() const {return Qt::Horizontal;}
  int count() const {return itemList.size();}
  QLayoutItem* itemAt(int index) const {return itemList.value(index);}
  QSize minimumSize() const;
  void setGeometry(const QRect& rect);
  QSize sizeHint() const {return minimumSize();}
  QLayoutItem* takeAt(int index);
  int doLayout(const QRect& rect, bool testOnly) const;

private:
  QList<QLayoutItem*> itemList;
};

FlowLayout::~FlowLayout()
{
  QLayoutItem *item;
  while((item = takeAt(0)))
    delete item;
}

QLayoutItem* FlowLayout::takeAt(int index)
{
  if(index >= 0 && index < itemList.size())
    return itemList.takeAt(index);
  else
    return nullptr;
}

void FlowLayout::setGeometry(const QRect &rect)
{
  QLayout::setGeometry(rect);
  doLayout(rect, false);
}

QSize FlowLayout::minimumSize() const
{
  QSize size;
  for(QLayoutItem* item : itemList)
    size += size.expandedTo(item->minimumSize());

  return size + QSize(2 * margin(), 2 * margin());
}

int FlowLayout::doLayout(const QRect& rect, bool testOnly) const
{
  // determine number of rows and max item width
  int y = rect.y();
  int numOfRows = 1;
  int lineWidth = 0;
  for(const QLayoutItem* item : itemList)
  {
    lineWidth = qMax(item->sizeHint().width(), lineWidth);
    int nextY = y + item->sizeHint().height() + verticalSpacing();
    if(nextY > rect.bottom())
    {
      y = rect.y();
      nextY = y + item->sizeHint().height() + verticalSpacing();
      ++numOfRows;
    }

    y = nextY;
  }

  // calc width for all items
  lineWidth = qMax(rect.width() / numOfRows, lineWidth);

  // place items
  int x = rect.x();
  y = rect.y();
  for(QLayoutItem* item : itemList)
  {
    int nextY = y + item->sizeHint().height() + verticalSpacing();
    if(nextY > rect.bottom())
    {
      y = rect.y();
      x = x + lineWidth;
      nextY = y + item->sizeHint().height() + verticalSpacing();
    }

    if(!testOnly)
      item->setGeometry(QRect(QPoint(x, y), QSize(lineWidth, item->sizeHint().height())));

    y = nextY;
  }
  return x + lineWidth;
}

ActuatorWidget::ActuatorWidget(SimRobotCore2::ActuatorPort* actuator, QWidget* parent) : QWidget(parent),
  actuatorName(actuator->getFullName()), actuator(actuator), value(0)
{
  isAngle = actuator->getUnit().contains(tr("°"));

  QStringList nameList = actuatorName.split(".");
  QLabel* label = new QLabel(nameList[nameList.size() - 2], this);
  slider = new QSlider(Qt::Horizontal, this);
  slider->setMinimumWidth(40);
  txbValue = new QDoubleSpinBox(this);
  cbxSet = new QCheckBox(tr(""), this);
  btnExit = new QPushButton(tr("x"), this);
#ifdef MACOS
  btnExit->setMaximumWidth(btnExit->height() / 2 + 8);
  btnExit->setMaximumHeight(btnExit->height() / 2 + 8);
  btnExit->setStyleSheet("QPushButton {background-color: #FAFAFA; padding: 2px; margin: 1px 1px 9px 2px; border: none; border-radius: 2px;} QPushButton:pressed {background-color: #3683F9; color: white;}");
#else
  btnExit->setMaximumWidth(btnExit->height() / 2);
  btnExit->setMaximumHeight(btnExit->height() / 2);
#endif

  QHBoxLayout *layout = new QHBoxLayout(this);

  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(5);
  layout->addWidget(btnExit);
#ifdef MACOS
  layout->addSpacing(5);
#endif
  layout->addWidget(label);
  layout->addWidget(slider, 0, Qt::AlignVCenter);
  layout->addWidget(txbValue);
  layout->addWidget(cbxSet);

  connect(slider, SIGNAL(valueChanged(int)), this, SLOT(valueChanged(int)));
  connect(txbValue, SIGNAL(valueChanged(double)), this, SLOT(valueChanged(double)));
  connect(btnExit, SIGNAL(released()), this, SIGNAL(releasedClose()));

  float minVal, maxVal;
  int factor;
  actuator->getMinAndMax(minVal, maxVal);

  if(isAngle)
  {
    minVal = toDeg(minVal);
    maxVal = toDeg(maxVal);
    factor = 10;
  }
  else
    factor = 1000;

  slider->setRange(static_cast<int>(minVal * factor), static_cast<int>(maxVal * factor));
  slider->setValue(static_cast<int>(value * factor));

  txbValue->setRange(minVal, maxVal);
  txbValue->setDecimals(isAngle ? 1 : 3);
  txbValue->setSingleStep(isAngle ? 0.1 : 0.001);
  txbValue->setValue(value);
  txbValue->setAlignment(Qt::AlignRight);

  // restore layout
  QSettings* settings = &CoreModule::application->getLayoutSettings();
  settings->beginGroup(actuatorName);
  valueChanged(settings->value("Value", 0.0).toDouble());
  cbxSet->setChecked(settings->value("Set", true).toBool());
  settings->endGroup();
}

ActuatorWidget::~ActuatorWidget()
{
  UserInput::InputPort* input = dynamic_cast<UserInput::InputPort*>(actuator);
  if(input)
    input->data.floatValue = input->defaultValue; // setValue would clip value

  QSettings* settings = &CoreModule::application->getLayoutSettings();
  settings->beginGroup(actuatorName);
  settings->setValue("Set", cbxSet->checkState() == Qt::Checked);
  settings->setValue("Value", value);
  settings->endGroup();
}

void ActuatorWidget::valueChanged(int value)
{
  float factor = isAngle ? 0.1f : 0.001f;
  txbValue->setValue(value * factor);
  this->value = static_cast<float>(value) * factor;
}

void ActuatorWidget::valueChanged(double value)
{
  slider->setValue(static_cast<int>(value * (isAngle ? 10 : 1000)));
  this->value = static_cast<float>(value);
}

void ActuatorWidget::adoptActuator()
{
  if(cbxSet->checkState() == Qt::Checked)
  {
    float value = static_cast<float>(this->value);
    if(isAngle)
      value = toRad(value);
    actuator->setValue(value);
  }
  else
  {
    UserInput::InputPort* input = dynamic_cast<UserInput::InputPort*>(actuator);
    if(input)
      input->data.floatValue = input->defaultValue; // setValue would clip value
  }
}

ActuatorsWidget* ActuatorsWidget::actuatorsWidget = 0;

ActuatorsWidget::ActuatorsWidget()
{
  Q_ASSERT(!actuatorsWidget);
  actuatorsWidget = this;

  //setFocusPolicy(Qt::StrongFocus);
  QVBoxLayout* outerlayout = new QVBoxLayout(this);
  layout = new FlowLayout;
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);
  outerlayout->setContentsMargins(0, 0, 0, 0);
  QScrollArea* scrollArea = new QScrollArea(this);
  clientArea = new QWidget;
  clientArea->setLayout(layout);
  outerlayout->addWidget(scrollArea);
  scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  scrollArea->setWidgetResizable(true);

  // load layout
  QSettings& settings = CoreModule::application->getLayoutSettings();
  settings.beginGroup("Actuators");
  QStringList openedActuators = settings.value("OpenedActuators").toStringList();
  settings.endGroup();
  for(const QString& actuator : openedActuators)
    openActuator(actuator);

  scrollArea->setWidget(clientArea);
}

ActuatorsWidget::~ActuatorsWidget()
{
  actuatorsWidget = 0;

  // save layout
  QSettings& settings = CoreModule::application->getLayoutSettings();
  settings.beginGroup("Actuators");
  settings.setValue("OpenedActuators", actuatorNames);
  settings.endGroup();
}

void ActuatorsWidget::resizeEvent(QResizeEvent* event)
{
  QWidget::resizeEvent(event);
  int width = layout->doLayout(QRect(0, 0, event->size().width(), event->size().height()), true);
  clientArea->setMinimumSize(QSize(width - 2, event->size().height()));
}

void ActuatorsWidget::openActuator(const QString& actuatorName)
{
  if(actuators.contains(actuatorName))
  {
    ActuatorWidget *widget = actuators.value(actuatorName);
    widget->setFocus();
    return;
  }

  SimRobotCore2::ActuatorPort* actuator = (SimRobotCore2::ActuatorPort*)CoreModule::application->resolveObject(actuatorName, SimRobotCore2::actuatorPort);
  if(!actuator)
    return;
  ActuatorWidget* widget = new ActuatorWidget(actuator, this);
  connect(widget, SIGNAL(releasedClose()), this, SLOT(closeActuator()));
  layout->addWidget(widget);
  actuators.insert(actuatorName, widget);
  actuatorNames.append(actuatorName);
}

void ActuatorsWidget::adoptActuators()
{
  for(ActuatorWidget* widget : actuators)
    widget->adoptActuator();
}

void ActuatorsWidget::closeActuator()
{
  ActuatorWidget* actuator = qobject_cast<ActuatorWidget*>(sender());
  if(!actuator)
    return;

  layout->removeWidget(actuator);
  actuators.remove(actuator->actuatorName);
  actuatorNames.removeOne(actuator->actuatorName);
  delete actuator;

  if(actuators.count() == 0)
    CoreModule::application->closeObject(CoreModule::module->actuatorsObject);
}
