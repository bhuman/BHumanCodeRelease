/**
 * @file Controller/Views/ModuleGraphView.h
 * Declaration of a class to represent a view displaying the module layout of the thread.
 * @author Thomas Röfer
 * @author Colin Graf
 */

#pragma once

#include "DotView.h"
#include "Tools/Module/Module.h"

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
   * @param category The category of the modules of this view. If numOfCategories, show all categories.
   */
  ModuleGraphViewObject(const QString& fullName, RobotConsole& console,
                        std::unordered_set<ModuleBase::Category> categories = {static_cast<ModuleBase::Category>(ModuleBase::numOfCategories)});

private:
  RobotConsole& console; /**< A reference to the console object. */
  std::string threadIdentifier; /**< The name of the view. */
  std::unordered_set<ModuleBase::Category> categories; /**< The category of the modules of this view. If numOfCategories, show all categories. */
  unsigned lastModulInfoTimestamp = 0; /**< Module Info timestamp when the image was created. */

  /**
   * The method replaces all ' ' by '_'.
   * @param s The input string.
   * @return The string in which spaces were replaced.
   */
  std::string compress(const std::string& s) const;

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
