/**
* @file ActuatorsWidget.h
* Declaration of class ActuatorsObject, ActuatorsWidget and ActuatorWidget
* @author Colin Graf
*/

#pragma once

#include <QWidget>
#include <QHash>
#include <QIcon>

#include "SimRobotCore2.h"

class QSlider;
class QSpinBox;
class QPushButton;
class QVBoxLayout;
class QCheckBox;
class ObjectDescription;
class ActuatorWidget;
class FlowLayout;

/**
* @class ActuatorsObject
* A hidden scene graph object for the centralized actuators widget
*/
class ActuatorsObject : public SimRobot::Object
{
public:

  /** Default constructor */
  ActuatorsObject() : name("Actuators") {}

protected:
  QString name;
  QIcon icon;

  virtual const QString& getFullName() const {return name;}
  virtual const QIcon* getIcon() const {return &icon;}
  virtual SimRobot::Widget* createWidget();
};

/**
* @class ActuatorsWidget
* The implementation of the centralized actuators widget
*/
class ActuatorsWidget : public QWidget, public SimRobot::Widget
{
  Q_OBJECT

public:
  static ActuatorsWidget* actuatorsWidget; /**< The only instance of the centralized actuators widget */

  /** Default constructor */
  ActuatorsWidget();

  /** Destructor */
  virtual ~ActuatorsWidget();

  /** Opens an actuator in the centralized actuators widget */
  void openActuator(const QString& name);

  /** Adopts user controlled actuator values */
  void adoptActuators();

private:
  QHash<QString, ActuatorWidget*> actuators;
  QStringList actuatorNames;
  FlowLayout* layout;
  QWidget* clientArea;

  virtual QWidget* getWidget() {return this;}
  void resizeEvent(QResizeEvent* event);

private slots:
  void closeActuator();
};

/**
* @class ActuatorsObject
* A widget for controlling an actuator
*/
class ActuatorWidget : public QWidget
{
  Q_OBJECT

public:
  QString actuatorName; /**< The scene graph path name of the actuator */

  /** Constructor
  * @param actuator The actuator controlled by the widget
  * @param parent The parent (centralized actuators) widget
  */
  ActuatorWidget(SimRobotCore2::ActuatorPort* actuator, QWidget* parent);

  /** Destructor */
  virtual ~ActuatorWidget();

  /** Adopts a user controlled actuator value */
  void adoptActuator();

signals:
  void releasedClose();

public slots:
  void valueChanged(int value);

private:
  SimRobotCore2::ActuatorPort* actuator;

  bool isAngle;
  QSlider* slider;
  QSpinBox* txbValue;
  QPushButton* btnExit;
  QCheckBox* cbxSet;
  int value;
  bool set;
};
