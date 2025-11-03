/**
 * @file SimulatedNao/RoboCupCtrl.h
 *
 * This file declares the class RoboCupCtrl.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "SimulatedRobot.h"
#include "TestGameController.h"
#include "Framework/Settings.h"
#include <SimRobotCore3.h>
#include <SimRobotCore2D.h>

#include <QBrush>
#include <QIcon>
#include <QList>
#include <QString>
#include <list>

class ControllerRobot;
class PaintMethods3DOpenGL;

/**
 * The class implements the SimRobot controller for RoboCup.
 */
class RoboCupCtrl : public SimRobot::Module, public SimRobotCore3::CollisionCallback, public SimRobotCore2D::CollisionCallback
{
  class Category : public SimRobot::Object
  {
    QString name;
    QString fullName;
    QIcon icon;

  public:
    Category(const QString& name, const QString& fullName, const char* icon) : name(name), fullName(fullName), icon(icon)
    {
      this->icon.setIsMask(true);
    }

  private:
    const QString& getFullName() const override { return fullName; }
    const QIcon* getIcon() const override { return &icon; }
  };

public:
  static RoboCupCtrl* controller; /**< A pointer to the SimRobot controller. */
  static SimRobot::Application* application; /**< The interface to the SimRobot GUI. */
  TestGameController gameController;
  PaintMethods3DOpenGL* paintMethods3D = nullptr;
  bool is2D = false; /**< Whether the controller is loaded in the 2D simulator (otherwise it is 3D simulation). */
  float simStepLength; /**< The length of one simulation step (in ms). */

protected:
  std::list<ControllerRobot*> robots; /**< The list of all robots. */
  float delayTime = 0.f; /**< Delay simulation to reach this duration of a step. */
  float lastTime = 0.f; /**< The last time execute was called. */

private:
  QList<SimRobot::Object*> views; /**< List of registered views */
  static constexpr float ballFriction = -0.35f; /**< The ball friction acceleration (2D only). */

public:
  RoboCupCtrl(SimRobot::Application& application);

protected:
  ~RoboCupCtrl();

public:
  /**
   * Return the alternate background color for views. It is used when
   * view separate different rows by switching between two different
   * background colors.
   * @return The color.
   */
  static QBrush getAlternateBackgroundColor();

  /**
   * Adds a scene graph object to the scene graph displayed in SimRobot
   * @param object The scene graph object to add
   * @param parent The parent scene graph object (e.g. a category or null)
   * @param flags Some flags for registering the scene graph object (see SimRobot::Flag)
   */
  void addView(SimRobot::Object* object, const SimRobot::Object* parent = nullptr, int flags = 0);

  /**
   * Adds a scene graph object to the scene graph displayed in SimRobot
   * @param object The scene graph object to add
   * @param categoryName The full name of the parent category
   * @param flags Some flags for registering the scene graph object (see SimRobot::Flag)
   */
  void addView(SimRobot::Object* object, const QString& categoryName, int flags = 0);

  /**
   * Removes a view that was previously added with addView(). Note that no parent object
   * is required here, due to asymmetrical registerObject()/unregisterObject() definitions
   * @param object View to remove from scene graph
   */
  void removeView(SimRobot::Object* object);

  /**
   * Adds a category to the scene graph displayed in SimRobot that can be used for grouping views
   * @param name The name of the category
   * @param parent The parent scene graph object (e.g. a category or null)
   * @param icon The icon used to list the category in the scene graph
   * @return The category
   */
  SimRobot::Object* addCategory(const QString& name, const SimRobot::Object* parent = nullptr, const char* icon = nullptr);

  /**
   * Adds a category to the scene graph displayed in SimRobot that can be used for grouping views
   * @param name The name of the category
   * @param parentName The name of the parent scene graph object (e.g. a category)
   * @return The category
   */
  SimRobot::Object* addCategory(const QString& name, const QString& parentName);

  /**
   * Determines the robot type from a special object present in the scene.
   * @return The robot type.
   */
  Settings::RobotType getRobotType() const;

protected:
  /**
   * The function is called to initialize the module.
   */
  bool compile() override;

  /**
   * The function is called in each simulation step.
   */
  void update() override;

  /**
   * The callback function.
   * Called whenever the geometry at which this interface is registered collides with another geometry.
   * @param geom1 The geometry at which the interface has been registered
   * @param geom2 The other geometry
   */
  void collided(SimRobotCore3::Geometry& geom1, SimRobotCore3::Geometry& geom2) override;

  /**
   * The callback function for the 2D core.
   * Called whenever the geometry at which this interface is registered collides with another geometry.
   * @param geom1 The geometry at which the interface has been registered
   * @param geom2 The other geometry
   */
  void collided(SimRobotCore2D::Geometry& geom1, SimRobotCore2D::Geometry& geom2) override;

  /**
   * Has to be called by derived class to start threads.
   */
  void start();

  /**
   * Has to be called by derived class to stop threads.
   */
  void stop();
};
