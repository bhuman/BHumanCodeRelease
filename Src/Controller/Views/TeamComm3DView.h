/**
* @file Controller/Views/TeamComm3DView.h
* Declaration of class TeamComm3DView
* @author Colin Graf
*/

#pragma once

#include <QString>
#include <QIcon>

#include "SimRobot.h"

/**
* @class TeamComm3DView
* A class to represent a view with information about a sbsl2 behavior.
*/
class TeamComm3DView : public SimRobot::Object
{
public:
  /**
  * Constructor.
  * @param fullName The full name of this view in the scene graph.
  * @param listenerIndex The index of the team listener for this view.
  */
  TeamComm3DView(const QString& fullName, int listenerIndex);

private:
  const QString fullName; /**< The path to this view in the scene graph */
  const QIcon icon; /**< The icon used for listing this view in the scene graph */
  int listenerIndex; /**< The index of the team listener for this view. */

  /**
  * The method returns a new instance of a widget for this direct view.
  * The caller has to delete this instance. (Qt handles this)
  * @return The widget.
  */
  virtual SimRobot::Widget* createWidget();

  virtual const QString& getFullName() const {return fullName;}
  virtual const QIcon* getIcon() const {return &icon;}

  friend class TeamComm3DWidget;
};
