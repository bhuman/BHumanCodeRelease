/**
 * @file Global.h
 * Declaration of a class that contains pointers to global data.
 * @author Thomas Röfer
 */

#pragma once

// Only declare prototypes. Don't include anything here, because this
// file is included in many other files.
class AnnotationManager;
class OutMessage;
struct Settings;
class DebugRequestTable;
class DebugDataTable;
class DrawingManager;
class DrawingManager3D;
class ReleaseOptions;
class TimingManager;
namespace asmjit
{
  class JitRuntime;
}

/**
 * @class Global
 * A class that contains pointers to global data.
 */
class Global
{
private:
  static thread_local AnnotationManager* theAnnotationManager;
  static thread_local OutMessage* theDebugOut;
  static thread_local Settings* theSettings;
  static thread_local DebugRequestTable* theDebugRequestTable;
  static thread_local DebugDataTable* theDebugDataTable;
  static thread_local DrawingManager* theDrawingManager;
  static thread_local DrawingManager3D* theDrawingManager3D;
  static thread_local TimingManager* theTimingManager;
  static thread_local asmjit::JitRuntime* theAsmjitRuntime;

public:
  /**
   * The method returns a reference to the thread wide instance.
   * @return The instance of the annotation manager in this thread.
   */
  static AnnotationManager& getAnnotationManager() { return *theAnnotationManager; }

  /**
   * The method returns a reference to the thread wide instance.
   * @return The instance of the outgoing debug message queue in this thread.
   */
  static OutMessage& getDebugOut() {return *theDebugOut;}

  /**
   * The method returns whether the outgoing message queue was already instantiated.
   * @return Is it safe to use getDebugOut()?
   */
  static bool debugOutExists() {return theDebugOut != nullptr;}

  /**
   * The method returns a reference to the thread wide instance.
   * @return The instance of the settings in this thread.
   */
  static Settings& getSettings() {return *theSettings;}

  /**
   * The method returns whether the settings have already been instantiated.
   * @return Is it safe to use getSettings()?
   */
  static bool settingsExist() {return theSettings != nullptr;}

  /**
   * The method returns a reference to the thread wide instance.
   * @return The instance of the debug request table in this thread.
   */
  static DebugRequestTable& getDebugRequestTable() {return *theDebugRequestTable;}

  /**
   * The method returns a reference to the thread wide instance.
   * @return The instance of the debug data table in this thread.
   */
  static DebugDataTable& getDebugDataTable() {return *theDebugDataTable;}

  /**
   * The method returns a reference to the thread wide instance.
   * @return The instance of the drawing manager in this thread.
   */
  static DrawingManager& getDrawingManager() {return *theDrawingManager;}

  /**
   * The method returns a reference to the thread wide instance.
   * @return The instance of the 3-D drawing manager in this thread.
   */
  static DrawingManager3D& getDrawingManager3D() {return *theDrawingManager3D;}

  /**
   * The method returns a reference to the thread wide instance.
   * @return the instance of the timing manager in this thread.
   */
  static TimingManager& getTimingManager() { return *theTimingManager; }

  /**
   * The method returns a reference to the thread wide instance.
   * @return the instance of the asmjit runtime in this thread.
   */
  static asmjit::JitRuntime& getAsmjitRuntime() { return *theAsmjitRuntime; }

  friend class ThreadFrame; // The class ThreadFrame can set these pointers.
  friend class Robot; // The class Robot can set theSettings.
  friend class ConsoleRoboCupCtrl; // The class ConsoleRoboCupCtrl can set theSettings.
  friend class RobotConsole; // The class RobotConsole can set theDebugOut.
};
