/**
 * @file Threads/Cognition.h
 *
 * This file declares the execution unit for the cognition thread.
 *
 * @author Jan Fiedler
 */

#pragma once

#include "Tools/Framework/SPLMessageHandler.h" // include this first to prevent WinSock2.h/Windows.h conflicts
#include "Tools/Framework/FrameExecutionUnit.h"

/**
 * @class Cognition
 *
 * The execution unit for the cognition thread.
 */
class Cognition : public FrameExecutionUnit
{
private:
  SPLStandardMessageBuffer<MAX_NUMBER_OF_PARALLEL_RECEIVABLE_SPLSTDMSG> inTeamMessages;
  RoboCup::SPLStandardMessage outTeamMessage;
  SPLMessageHandler theSPLMessageHandler;
  unsigned lastUpperFrameTime = 0; /**< The last timestamp received from the upper camera thread. */
  unsigned lastLowerFrameTime = 0; /**< The last timestamp received from the lower camera thread. */
  unsigned lastAcceptedTime = 0; /**< The timestamp of the last image accepted. */
  bool upperIsNew = false; /**< The is unused data from the upper camera thread. */
  bool lowerIsNew = false; /**< The is unused data from the lower camera thread. */
  bool acceptNext = false; /**< Immediately process the frame still waiting. */

public:
  thread_local static bool isUpper; /**< The current frame picked is from the upper camera thread. */

  Cognition();
  ~Cognition();
  bool beforeFrame() override;
  void beforeModules() override;
  void afterModules() override;
  bool afterFrame() override;
};
