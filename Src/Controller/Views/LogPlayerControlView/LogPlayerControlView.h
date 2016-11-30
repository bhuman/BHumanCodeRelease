#pragma once

#include "SimRobot.h"

#include <QIcon>
#include <QString>

class LogPlayer;
class LogPlayerControlWidget;
class RobotConsole;

class LogPlayerControlView : public SimRobot::Object
{
private:
  const QString fullName; /**< The path to this view in the scene graph */
  LogPlayer& logPlayer;
  RobotConsole& console;
  const QIcon icon; /**< The icon used to list this view in the scene graph */

  friend class LogPlayerControlWidget;

public:
  LogPlayerControlView(const QString& fullName, LogPlayer& logPlayer, RobotConsole& console) :
    fullName(fullName), logPlayer(logPlayer), console(console), icon(":/Icons/tag_green.png")
  {}

  /**
   * The method returns a new instance of a widget for this view.
   * The caller has to delete the returned instance. (Qt will take care of this)
   * @return The widget.
   */
  virtual SimRobot::Widget* createWidget();

  /**
   * Accesses path name to the object in the scene graph
   * @return The path name
   */
  virtual const QString& getFullName() const { return fullName; }

  /**
   * Accesses the icon used to list this view in the scene graph
   * @return The icon
   */
  virtual const QIcon* getIcon() const { return &icon; }
};