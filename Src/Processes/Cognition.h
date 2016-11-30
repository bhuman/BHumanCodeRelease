/**
 * @file Processes/Cognition.h
 * Declaration of a class that represents a process that receives data from the robot at about 60 Hz.
 */

#pragma once

#include "Tools/ProcessFramework/TeamHandler.h" // include this first to prevent WinSock2.h/Windows.h conflicts

#include "Tools/Logging/Logger.h"
#include "Tools/Module/ModulePackage.h"
#include "Tools/ProcessFramework/Process.h"

/**
 * @class Cognition
 * A class that represents a process that receives data from the robot at about 30 Hz.
 */
class Cognition : public Process
{
private:
  Receiver<DebugToCognition> theDebugReceiver;
  DebugSender<CognitionToDebug> theDebugSender;
  Receiver<MotionToCognition> theMotionReceiver;
  Sender<CognitionToMotion> theMotionSender;
  MessageQueue theTeamReceiver;
  MessageQueue theTeamSender;
  TeamHandler theTeamHandler;
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
