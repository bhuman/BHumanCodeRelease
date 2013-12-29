/**
* @file Controller/Views/ViewBike/ViewBike.h
*
* Declaration of class ViewBike
*
* @author <a href="mailto:judy@tzi.de">Judith Müller</a>
*/

#pragma once

#include <QString>
#include <QIcon>

#include "SimRobotCore2.h"
#include "Controller/Visualization/HeaderedWidget.h"
#include "Modules/MotionControl/BIKEParameters.h"

class RobotConsole;
class MotionRequest;
class JointData;
class JointCalibration;
class RobotDimensions;
class ViewBikeWidget;
class SensorData;

class ViewBike : public SimRobot::Object
{
public:
  /**
  * Constructor.
  */
  ViewBike(const QString& fullName, RobotConsole& console, const MotionRequest& motionRequest, const JointData& jointData, const JointCalibration& jointCalibration, const SensorData& sensorData, const RobotDimensions& robotDimensions, const std::string& mr, SimRobotCore2::Body* robot);

  QString fullName;
  QIcon icon;
  RobotConsole& console;
  const MotionRequest& motionRequest;
  const JointData& jointData;
  const JointCalibration& jointCalibration;
  const SensorData& sensorData;
  const RobotDimensions& robotDimensions;
  const std::string& motionRequestCommand;
  SimRobotCore2::Body* robot;

private:
  virtual SimRobot::Widget* createWidget();
  virtual const QString& getFullName() const {return fullName;}
  virtual const QIcon* getIcon() const {return &icon;}

};

class ViewBikeHeaderedWidget : public HeaderedWidget, public SimRobot::Widget
{

  Q_OBJECT

public:
  ViewBikeHeaderedWidget(ViewBike& viewBike);
  ViewBikeWidget* viewBikeWidget;

  virtual QWidget* getWidget() {return this;}
  virtual void update();
  virtual QMenu* createFileMenu() const;
  virtual QMenu* createEditMenu() const;
  virtual bool canClose();
  void addStateToUndoList();

private:
  BIKEParameters parameters;
  QString fileName;
  std::vector<BIKEParameters> undo, redo;

  void writeParametersToFile(const std::string& name);

signals:
  void undoAvailable(bool available);
  void redoAvailable(bool available);
  void saveAvailable(bool available);

private slots:
  void newButtonClicked();
  void loadButtonClicked();
  void saveButtonClicked();
  void saveAsButtonClicked();
  void undoChanges();
  void redoChanges();

};
