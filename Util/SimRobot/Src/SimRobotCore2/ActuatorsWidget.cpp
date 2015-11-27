/**
* @file ActuatorsWidget.cpp
* Implementation of class ActuatorsWidget and ActuatorWidget
*/

#include <QLabel>
#include <QVBoxLayout>
#include <QSlider>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QCheckBox>
#include <QResizeEvent>
#include <QScrollArea>
#include <QSettings>

#include "Simulation/Simulation.h"
#include "Simulation/UserInput.h"
#include "ActuatorsWidget.h"
#include "CoreModule.h"

static inline float toDeg(float angleInRad)
{ return (float) (angleInRad * 180.0f / M_PI);}
static inline float toRad(float angleInDegree)
{ return (float) (angleInDegree * M_PI / 180.0f);}

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
  int verticalSpacing() const {return 0;}
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
    return 0;
}

void FlowLayout::setGeometry(const QRect &rect)
{
  QLayout::setGeometry(rect);
  doLayout(rect, false);
}

QSize FlowLayout::minimumSize() const
{
  QSize size;
  QLayoutItem* item;
  foreach(item, itemList)
    size = size.expandedTo(item->minimumSize());

  return size + QSize(2 * margin(), 2 * margin());
}

int FlowLayout::doLayout(const QRect& rect, bool testOnly) const
{
  // determine number of rows and max item width
  int y = rect.y();
  int numOfRows = 1;
  int lineWidth = 0;
  QLayoutItem* item;
  foreach(item, itemList)
  {
    lineWidth = qMax(item->sizeHint().width(), lineWidth);
    int nextY = y + item->sizeHint().height();
    if(nextY > rect.bottom())
    {
      y = rect.y();
      nextY = y + item->sizeHint().height();
      ++numOfRows;
    }

    y = nextY;
  }

  // calc width for all items
  lineWidth = qMax(rect.width() / numOfRows, lineWidth);

  // place items
  int x = rect.x();
  y = rect.y();
  foreach(item, itemList)
  {
    int nextY = y + item->sizeHint().height();
    if(nextY > rect.bottom())
    {
      y = rect.y();
      x = x + lineWidth;
      nextY = y + item->sizeHint().height();
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
  txbValue = new QSpinBox(this);
  cbxSet = new QCheckBox(tr(""), this);
  btnExit = new QPushButton(tr("x"), this);
#ifdef OSX
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
#ifdef OSX
  layout->addSpacing(5);
#endif
  layout->addWidget(label);
  layout->addWidget(slider, 0, Qt::AlignVCenter);
  layout->addWidget(txbValue);
  layout->addWidget(cbxSet);

  connect(slider, SIGNAL(valueChanged(int)), this, SLOT(valueChanged(int)));
  connect(txbValue, SIGNAL(valueChanged(int)), this, SLOT(valueChanged(int)));
  connect(btnExit, SIGNAL(released()), this, SIGNAL(releasedClose()));

  float minVal, maxVal;
  actuator->getMinAndMax(minVal, maxVal);

  if(isAngle)
  {
    minVal = toDeg(minVal);
    maxVal = toDeg(maxVal);
  }
  else
  {
    minVal *= 100;
    maxVal *= 100;
  }

  slider->setRange((int)minVal, (int)maxVal);
  slider->setValue(value);

  txbValue->setRange((int)minVal, (int)maxVal);
  txbValue->setValue(value);

  // restore layout
  QSettings* settings = &CoreModule::application->getLayoutSettings();
  settings->beginGroup(actuatorName);
  valueChanged(settings->value("Value", int(0)).toInt());
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
  txbValue->setValue(value);
  slider->setValue(value);
  this->value = value;
}

void ActuatorWidget::adoptActuator()
{
  if(cbxSet->checkState() == Qt::Checked)
  {
    float value = (float) this->value;
    if(isAngle)
      value = toRad(value);
    else
      value /= 100.f;
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
  foreach(QString actuator, openedActuators)
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
  foreach(ActuatorWidget* widget, actuators)
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
