/**
* @file AnnotationView.h
* @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
*/

#pragma once

#include <SimRobot.h>
#include <QIcon>

class AnnotationInfo;
class RobotConsole;
class LogPlayer;

class AnnotationView : public SimRobot::Object
{
public:
  AnnotationView(const QString& fullName, AnnotationInfo& info, LogPlayer& logPlayer, SimRobot::Application* application);

private:
  const QString fullName; /**< The path to this view in the scene graph */
  const QIcon icon; /**< The icon used for listing this view in the scene graph */
  AnnotationInfo& info;
  LogPlayer& logPlayer; /**< Used to jump to frames */
  SimRobot::Application* application;

  /**
  * The method returns a new instance of a widget for this direct view.
  * The caller has to delete this instance. (Qt handles this)
  * @return The widget.
  */
  virtual SimRobot::Widget* createWidget();

  virtual const QString& getFullName() const { return fullName; }
  virtual const QIcon* getIcon() const { return &icon; }

  friend class AnnotationWidget; //AnnotationWidget needs access to console
};
