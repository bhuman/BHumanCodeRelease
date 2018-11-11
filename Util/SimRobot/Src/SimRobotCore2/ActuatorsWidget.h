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
class QDoubleSpinBox;
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

  const QString& getFullName() const override {return name;}
  const QIcon* getIcon() const override {return &icon;}
  SimRobot::Widget* createWidget() override;
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
  ~ActuatorsWidget();

  /** Opens an actuator in the centralized actuators widget */
  void openActuator(const QString& name);

  /** Adopts user controlled actuator values */
  void adoptActuators();

private:
  QHash<QString, ActuatorWidget*> actuators;
  QStringList actuatorNames;
  FlowLayout* layout;
  QWidget* clientArea;

  QWidget* getWidget() override {return this;}
  void resizeEvent(QResizeEvent* event) override;

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
  ~ActuatorWidget();

  /** Adopts a user controlled actuator value */
  void adoptActuator();

signals:
  void releasedClose();

public slots:
  void valueChanged(int value);
  void valueChanged(double value);

private:
  SimRobotCore2::ActuatorPort* actuator;

  bool isAngle;
  QSlider* slider;
  QDoubleSpinBox* txbValue;
  QPushButton* btnExit;
  QCheckBox* cbxSet;
  float value;
  bool set;
};
