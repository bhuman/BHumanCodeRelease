/**
 * @file Processes/Cognition.h
 * Declaration of a class that represents a process that receives data from the robot at about 60 Hz.
 */

#pragma once

#include "Tools/ProcessFramework/TeamHandler.h" // include this first to prevent winsock2.h/windows.h conflicts

#include "Tools/ProcessFramework/Process.h"
#include "Tools/Module/Logger.h"
#include "Tools/Module/ModulePackage.h"

/**
 * @class Cognition
 * A class that represents a process that receives data from the robot at about 30 Hz.
 */
class Cognition : public Process
{
private:
  DEBUGGING;
  RECEIVER(MotionToCognition);
  SENDER(CognitionToMotion);
  TEAM_COMM;
  int numberOfMessages;
  ModuleManager moduleManager; /**< The solution manager handles the execution of modules. */
  Logger logger; /**< The logger logs representations in the background. */

public:
  Cognition();

  /**
   * The method is called from the framework once in every frame.
   */
  virtual bool main();

  /**
   * The method is called directly before the first call of main().
   */
  virtual void init();

  /**
   * The method is called when the process is terminated.
   */
  virtual void terminate();

  /**
   * The function handles incoming debug messages.
   * @param message the message to handle.
   * @return Has the message been handled?
   */
  virtual bool handleMessage(InMessage& message);
};
