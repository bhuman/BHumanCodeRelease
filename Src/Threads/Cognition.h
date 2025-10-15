/**
 * @file Threads/Cognition.h
 *
 * This file declares the execution unit for the cognition thread.
 *
 * @author Jan Fiedler
 */

#pragma once

#include "Tools/Framework/BHExecutionUnit.h"

/**
 * @class Cognition
 *
 * The execution unit for the cognition thread.
 */
class Cognition : public BHExecutionUnit
{
private:
  static const unsigned minTime = 100000; /**< The earliest time used. */
  unsigned lastUpperFrameTime = minTime; /**< The last timestamp received from the upper camera thread. */
  unsigned lastLowerFrameTime = minTime; /**< The last timestamp received from the lower camera thread. */
  unsigned lastAcceptedTime = minTime; /**< The timestamp of the last image accepted. */
  bool upperIsNew = false; /**< The is unused data from the upper camera thread. */
  bool lowerIsNew = false; /**< The is unused data from the lower camera thread. */
  bool acceptNext = false; /**< Immediately process the frame still waiting. */
  int delayedLogCounter = 0; /**< How many cycles is the acknowledgement of received log data already delayed? */

public:
  thread_local static bool isUpper; /**< The current frame picked is from the upper camera thread. */

  Cognition();
  ~Cognition();
  bool beforeFrame() override;
  void beforeModules() override;
  void afterModules() override;
  bool afterFrame() override;
};
