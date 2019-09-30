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

struct JointAngles;
struct JointLimits;
struct MotionRequest;
struct RobotDimensions;
class RobotConsole;
class KickViewWidget;

class KickView : public SimRobot::Object
{
public:
  QString fullName;
  QIcon icon;
  RobotConsole& console;
  const MotionRequest& motionRequest;
  const JointAngles& jointAngles;
  const JointLimits& jointLimits;
  const RobotDimensions& robotDimensions;
  const std::string& motionRequestCommand;
  SimRobotCore2::Body* robot;

  KickView(const QString& fullName, RobotConsole& console, const MotionRequest& motionRequest, const JointAngles& jointAngles,
           const JointLimits& jointLimits, const RobotDimensions& robotDimensions, const std::string& mr, SimRobotCore2::Body* robot);

private:
  SimRobot::Widget* createWidget() override;
  const QString& getFullName() const override { return fullName; }
  const QIcon* getIcon() const override { return &icon; }
};

class KickViewHeaderedWidget : public HeaderedWidget, public SimRobot::Widget
{
  Q_OBJECT

public:
  KickViewWidget* kickViewWidget;

  KickViewHeaderedWidget(KickView& kickView);

  QWidget* getWidget() override { return this; }
  void update() override;
  QMenu* createFileMenu() const override;
  QMenu* createEditMenu() const override;
  bool canClose() override;
  void addStateToUndoList();

private:
  KickEngineParameters parameters;
  QString fileName;
  std::vector<KickEngineParameters> undo, redo;

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
