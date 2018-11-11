/**
 * @file Processes/Motion.h
 * Declaration of a class that represents the process that sends commands to the robot at 100Hz.
 */

#pragma once

#include "Tools/Logging/Logger.h"
#include "Tools/Module/ModulePackage.h"
#include "Tools/ProcessFramework/Process.h"

/**
 * @class Motion
 * A class that represents the process that sends commands to the robot at 100Hz.
 */
class Motion : public Process
{
private:
  Receiver<DebugToMotion> theDebugReceiver;
  DebugSender<MotionToDebug> theDebugSender;
  Receiver<CognitionToMotion> theCognitionReceiver;
  Sender<MotionToCognition> theCognitionSender;
  int numberOfMessages;
  ModuleManager moduleManager; /**< The solution manager handles the execution of modules. */
  Logger logger;

public:
  Motion();

  /**
   * The method is called from the framework once in every frame.
   * @return Should wait for external trigger?
   */
  bool main() override;

  /**
   * The method is called directly before the first call of main().
   */
  void init() override;

  /**
   * The method is called when the process is terminated.
   */
  void terminate() override;

  /**
   * The function handles incoming debug messages.
   * @param message the message to handle.
   * @return Has the message been handled?
   */
  bool handleMessage(InMessage& message) override;
};
