/**
* @file Controller/Views/KickView/KickView.h
*
* Declaration of class KickView
*
* @author <a href="mailto:judy@tzi.de">Judith Müller</a>
*/

#pragma once

#include <SimRobotCore2.h>
#include "Controller/Visualization/HeaderedWidget.h"
#include "Modules/MotionControl/KickEngine/KickEngineParameters.h"

#include <QString>
#include <QIcon>

struct MotionRequest;
struct JointCalibration;
struct JointAngles;
struct RobotDimensions;
class RobotConsole;
class KickViewWidget;

class KickView : public SimRobot::Object
{
public:
  /**
  * Constructor.
  */
  KickView(const QString& fullName, RobotConsole& console, const MotionRequest& motionRequest, const JointAngles& jointAngles, 
           const JointCalibration& jointCalibration, const RobotDimensions& robotDimensions, const std::string& mr, SimRobotCore2::Body* robot);

  QString fullName;
  QIcon icon;
  RobotConsole& console;
  const MotionRequest& motionRequest;
  const JointAngles& jointAngles;
  const JointCalibration& jointCalibration;
  const RobotDimensions& robotDimensions;
  const std::string& motionRequestCommand;
  SimRobotCore2::Body* robot;

private:
  virtual SimRobot::Widget* createWidget();
  virtual const QString& getFullName() const {return fullName;}
  virtual const QIcon* getIcon() const {return &icon;}
};

class KickViewHeaderedWidget : public HeaderedWidget, public SimRobot::Widget
{
  Q_OBJECT

public:
  KickViewHeaderedWidget(KickView& kickView);
  KickViewWidget* kickViewWidget;

  virtual QWidget* getWidget() {return this;}
  virtual void update();
  virtual QMenu* createFileMenu() const;
  virtual QMenu* createEditMenu() const;
  virtual bool canClose();
  void addStateToUndoList();

private:
  KickEngineParameters parameters;
  QString fileName;
  std::vector<KickEngineParameters> undo, redo;

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
