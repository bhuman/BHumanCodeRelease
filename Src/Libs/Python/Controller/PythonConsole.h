/**
 * @file PythonConsole.h
 *
 * This file declares a class that controls a particular robot instance.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Framework/Communication.h"
#include "Framework/ThreadFrame.h"
#include "Streaming/MessageQueue.h"
#include <string>

class Debug;
struct Settings;

class PythonConsole : public ThreadFrame
{
public:
  /**
   * Constructor
   * @param settings The settings for the robot.
   * @param robotName The name of the robot.
   * @param debug The debug thread of the robot instance.
   */
  PythonConsole(const Settings& settings, const std::string& robotName, Debug* debug);

  /** Destructor. */
  ~PythonConsole() override;

  /** Triggers an update of the thread (called from main thread). */
  void update();

protected:
  /**
   * The function determines the priority of the thread.
   * @return The priority of the thread.
   */
  int getPriority() const override { return 0; }

  /**
   * That function is called once before the first main(). It can be used
   * for things that can't be done in the constructor.
   */
  void init() override;

  /**
   * The function is called from the framework once in every frame.
   */
  bool main() override;

  /**
   * The function is called when the thread is terminated.
   */
  void terminate() override {}

private:
  /**
   * The function connects the robot to the returned receiver.
   *
   * @param debug The debug connection of the robot.
   * @return The receiver connected to the robot.
   */
  DebugReceiver<MessageQueue>* connectReceiverWithRobot(Debug* debug);

  /**
   * The function connects the robot to the returned sender.
   *
   * @param debug The debug connection of the robot.
   * @return The sender connected to the robot.
   */
  DebugSender<MessageQueue>* connectSenderWithRobot(Debug* debug) const;
};
