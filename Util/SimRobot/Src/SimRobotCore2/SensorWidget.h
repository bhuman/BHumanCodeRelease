/**
* @file SensorWidget.h
* Declaration of class SensorWidget
* @author Colin Graf
*/

#pragma once

#include <QWidget>
#include <QPainter>

#include "SimRobotCore2.h"

class QMenu;
class QMimeData;

/**
* @class SensorWidget
* A class that implements a view for visualizing sensor readings
*/
class SensorWidget : public QWidget, public SimRobot::Widget
{
  Q_OBJECT

public:
   /**
  * Constructor
  * @param sensor The sensor the readings of which should be visualized
  */
  SensorWidget(SimRobotCore2::SensorPort* sensor);

  /** Destructor */
  virtual ~SensorWidget();

private:
  QPainter painter;
  QPen pen;
  SimRobotCore2::SensorPort* sensor;
  SimRobotCore2::SensorPort::SensorType sensorType;
  QList<int> sensorDimensions;
  QMimeData* mineData;

  virtual QWidget* getWidget() {return this;}
  virtual void update();
  virtual QMenu* createEditMenu() const;

  virtual QSize sizeHint() const;
  virtual void paintEvent(QPaintEvent *event);

  void paintBoolSensor();
  void paintFloatArrayWithDescriptionsSensor();
  void paintFloatArrayWithLimitsAndWithoutDescriptions();
  void paint2DFloatArrayWithLimitsAndWithoutDescriptions();

  void setClipboardGraphics(QMimeData& mimeData);
  void setClipboardText(QMimeData& mimeData);

private slots:
  void copy();
};
