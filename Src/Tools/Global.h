/**
 * @file Global.h
 * Declaration of a class that contains pointers to global data.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Platform/Thread.h"

// Only declare prototypes. Don't include anything here, because this
// file is included in many other files.
class AnnotationManager;
class OutMessage;
struct Settings;
class DebugRequestTable;
class DebugDataTable;
class StreamHandler;
class DrawingManager;
class DrawingManager3D;
class ReleaseOptions;
class TimingManager;

/**
 * @class Global
 * A class that contains pointers to global data.
 */
class Global
{
private:
  static thread_local AnnotationManager* theAnnotationManager;
  static thread_local OutMessage* theDebugOut;
  static thread_local OutMessage* theTeamOut;
  static thread_local Settings* theSettings;
  static thread_local DebugRequestTable* theDebugRequestTable;
  static thread_local DebugDataTable* theDebugDataTable;
  static thread_local StreamHandler* theStreamHandler;
  static thread_local DrawingManager* theDrawingManager;
  static thread_local DrawingManager3D* theDrawingManager3D;
  static thread_local TimingManager* theTimingManager;

public:
  /**
   * The method returns a reference to the process wide instance.
   * @return The instance of the annotation manager in this process.
   */
  static AnnotationManager& getAnnotationManager() { return *theAnnotationManager; }

  /**
   * The method returns a reference to the process wide instance.
   * @return The instance of the outgoing debug message queue in this process.
   */
  static OutMessage& getDebugOut() {return *theDebugOut;}

  /**
   * The method returns a reference to the process wide instance.
   * @return The instance of the outgoing team message queue in this process.
   */
  static OutMessage& getTeamOut() {return *theTeamOut;}

  /**
   * The method returns a reference to the process wide instance.
   * @return The instance of the settings in this process.
   */
  static Settings& getSettings() {return *theSettings;}

  /**
   * The method returns whether the settings have already been instantiated.
   * @return Is it safe to use getSettings()?
   */
  static bool settingsExist() {return theSettings != nullptr;}

  /**
   * The method returns a reference to the process wide instance.
   * @return The instance of the debug request table in this process.
   */
  static DebugRequestTable& getDebugRequestTable() {return *theDebugRequestTable;}

  /**
   * The method returns a reference to the process wide instance.
   * @return The instance of the debug data table in this process.
   */
  static DebugDataTable& getDebugDataTable() {return *theDebugDataTable;}

  /**
   * The method returns a reference to the process wide instance.
   * @return The instance of the stream handler in this process.
   */
  static StreamHandler& getStreamHandler() {return *theStreamHandler;}

  /**
   * The method returns a reference to the process wide instance.
   * @return The instance of the drawing manager in this process.
   */
  static DrawingManager& getDrawingManager() {return *theDrawingManager;}

  /**
   * The method returns a reference to the process wide instance.
   * @return The instance of the 3-D drawing manager in this process.
   */
  static DrawingManager3D& getDrawingManager3D() {return *theDrawingManager3D;}

  /**
   * The method returns a reference to the process wide instance.
   * @return the instance of the timing manager in this process.
   */
  static TimingManager& getTimingManager() {return *theTimingManager;}

  friend class Process; // The class Process can set these pointers.
  friend class Cognition; // The class Cognition can set theTeamOut.
  friend struct Settings; // The class Settings can set a default StreamHandler.
  friend class ConsoleRoboCupCtrl; // The class ConsoleRoboCupCtrl can set theStreamHandler.
  friend class RobotConsole; // The class RobotConsole can set theDebugOut.
  friend class Framework;
};
