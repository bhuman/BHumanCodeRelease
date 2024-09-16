/**
 * @file SimulatedNao/Views/ModuleGraphView.h
 * Declaration of a class to represent a view displaying the module layout of the thread.
 * @author Thomas Röfer
 * @author Colin Graf
 */

#pragma once

#include "DotView.h"
#include <string>

class RobotConsole;

/**
 * A class to represent a view displaying the module layout of the thread.
 */
class ModuleGraphViewObject : public DotViewObject
{
public:
  /**
   * @param fullName The path to this view in the scene graph
   * @param console The console object.
   * @param threadName The thread of the modules shown in this view..
   */
  ModuleGraphViewObject(const QString& fullName, RobotConsole& console, const std::string& threadName);

private:
  RobotConsole& console; /**< A reference to the console object. */
  std::string threadName; /**<The thread of the modules shown in this view. */
  unsigned lastModuleInfoTimestamp = 0; /**< Module Info timestamp when the image was created. */

  /**
   * Checks whether the content that will be returned from a \c generateDotFileContent call has changed
   * @return \c true When \c generateDotFileContent will return something new
   */
  bool hasChanged() override;

  /**
   * Returns the content of the dot graph file that will be displayed
   * @return The content of the dot graph file
   */
  QString generateDotFileContent() override;
};
