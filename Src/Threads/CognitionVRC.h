/**
 * @file CognitionVRC.h
 *
 * This file declares a thread that contains modeling and
 * behavior control for the Visual Referee Challenge. It does
 * not force to alternate beween both perception threads.
 * Instead it prefers the Upper thread (which will send less
 * often), but also accepts everything from the Lower thread.
 * FrameInfo.time received from Upper is modified to be
 * newer than last timestamp received.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Cognition.h"

class CognitionVRC : public Cognition
{
  unsigned lastUpperFrameTime = 0; /**< The last timestamp received from the upper camera thread. */
  unsigned lastLowerFrameTime = 0; /**< The last timestamp received from the lower camera thread. */
  unsigned lastAcceptedTime = 0; /**< The timestamp of the last image accepted. */
  bool upperIsNew = false; /**< The is unused data from the upper camera thread. */
  bool lowerIsNew = false; /**< The is unused data from the lower camera thread. */

public:
  /**
   * The function checks whether there is new data from either
   * the Upper or the Lower thread available and decides which
   * to process first.
   * @return Is new data from either Upper or Lower available?
   */
  bool beforeFrame() override;

  /**
   * The function checks whether it should be waited for a new packet
   * to arrive or whether there is already an unprocessed packet available.
   * @return Should the thread wait for a packet to arrive before continuing?
   */
  bool afterFrame() override;
};
