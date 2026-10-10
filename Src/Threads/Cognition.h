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
  unsigned lastFrameTime = minTime; /**< The last timestamp received from the camera thread. */
  int delayedLogCounter = 0; /**< How many cycles is the acknowledgement of received log data already delayed? */

public:
  Cognition();
  ~Cognition();
  bool beforeFrame() override;
  void beforeModules() override;
  void afterModules() override;
  bool afterFrame() override;
};
