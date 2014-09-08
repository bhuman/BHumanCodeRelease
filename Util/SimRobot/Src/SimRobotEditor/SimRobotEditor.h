/**
* @file SimRobot/SimRobotEditor.h
* Declaration of interfaces to scene graph objects from SimRobotEditor
* @author Colin Graf
*/

#pragma once

#include "../SimRobot/SimRobot.h"

namespace SimRobotEditor
{
  /**
  * @class Editor
  * The interface to the "Editor" scene graph object
  */
  class Editor : public SimRobot::Object
  {
  public:
    /**
    * Adds a file to the list of files that are editable with the SimRobotEditor module
    * @param filePath The absolute path to the file
    * @param subFileRegExpPattern A regular expression that allows to determine included within a file that is opened for editing (e.g. "call[ ]+([\\\\/a-z0-9\\.\\-_]+)")
    * @return An editor object that can be used to add subfiles or folders
    */
    virtual Editor* addFile(const QString& filePath, const QString& subFileRegExpPattern) = 0;

    /**
    * Adds a folder to the list of files that are editable with the SimRobotEditor module
    * @param name The name of the folder
    * @return An editor object that can be used to add subfiles or folders
    */
    virtual Editor* addFolder(const QString& name) = 0;
  };
}
