/**
 * @file AnnotationView.h
 * @author Andreas Stolpmann
 */

#pragma once

#include <SimRobot.h>
#include <QIcon>
#include "Platform/SystemCall.h"
#include "Controller/Representations/AnnotationInfo.h"

class AnnotationInfo;
class RobotConsole;
class LogPlayer;

class AnnotationView : public SimRobot::Object
{
public:
  AnnotationView(const QString& fullName, AnnotationInfo::ProcessAnnotationInfo& info, LogPlayer& logPlayer, SystemCall::Mode mode, SimRobot::Application* application);

private:
  const QString fullName; /**< The path to this view in the scene graph */
  const QIcon icon; /**< The icon used for listing this view in the scene graph */
  AnnotationInfo::ProcessAnnotationInfo& info;
  LogPlayer& logPlayer; /**< Used to jump to frames */
  SystemCall::Mode mode; /**< Only jump if replaying logs */
  SimRobot::Application* application;

  bool stopOnFilter = false;
  bool filterIsRegEx = false;
  QString filter = "";

  /**
   * The method returns a new instance of a widget for this direct view.
   * The caller has to delete this instance. (Qt handles this)
   * @return The widget.
   */
  virtual SimRobot::Widget* createWidget();

  virtual const QString& getFullName() const { return fullName; }
  virtual const QIcon* getIcon() const { return &icon; }

  friend class AnnotationWidget; //AnnotationWidget needs access to console
  friend class AnnotationInfo;
};
