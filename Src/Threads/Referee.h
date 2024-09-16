/**
 * @file Referee.h
 *
 * This file declares a thread that handles the referee pose detection.
 *
 * @author Ayleen Lührsen
 */

#pragma once

#include "Tools/Framework/BHExecutionUnit.h"

class Referee : public BHExecutionUnit
{
  bool handlerRegistered = false; /**< Was the handler that determines whether debug data was received registered? */
  bool receivedDebugData = true; /**< Was debug data received for the current frame? */
  unsigned lastImageTimestamp = 0; /**< The timestamp of the last image processed. 0 if no image was present. */

public:
  bool beforeFrame() override;
  bool afterFrame() override;
};
