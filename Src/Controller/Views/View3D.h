/**
 * @file Controller/Views/View3D.h
 *
 * Declaration of class View3D
 *
 * @author Thomas Röfer
 */

#pragma once

#include <QString>
#include <QIcon>
#include <SimRobot.h>

#include "Tools/Math/Eigen.h"

/**
 * @class View3D
 *
 * The base class for all 3-D views.
 *
 * @author Thomas Röfer
 */
class View3D : public SimRobot::Object
{
public:
  /**
   * @param fullName The path to this view in the scene graph
   * @param background The background color.
   */
  View3D(const QString& fullName, const Vector3f& background);

protected:
  enum
  {
    cubeId = 1, /**< The id of the cube display list. */
    colorsId /**< The id of the color table display list. */
  };

  const Vector3f& background; /**< The background color. */
  Vector3f lastBackground; /**< The last background color used. */

  /**
   * Update the display lists if required.
   */
  virtual void updateDisplayLists() = 0;

  /**
   * Need the display lists to be updated?
   * @return Yes or no?
   */
  virtual bool needsUpdate() const { return true; }

  /**
   * The function returns the view distance.
   * @return The distance from which the scene is viewed.
   */
  virtual float getViewDistance() const { return 8.0; }

private:
  const QString fullName; /**< The path to this view in the scene graph */
  const QIcon icon; /**< The icon used for listing this view in the scene graph */

  /**
   * The method returns a new instance of a widget for this direct view.
   * The caller has to delete this instance. (Qt handles this)
   * @return The widget.
   */
  SimRobot::Widget* createWidget() override;

  const QString& getFullName() const override { return fullName; }
  const QIcon* getIcon() const override { return &icon; }

  friend class View3DWidget;
};
