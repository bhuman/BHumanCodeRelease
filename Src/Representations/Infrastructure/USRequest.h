/**
 * @file USRequest.h
 * This file declares a struct that represents a request for controlling
 * which sonar is fired next.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"

/**
 * @struct USRequest
 * A struct that represents a request for controlling
 * which sonar is fired next.
 */
STREAMABLE(USRequest,
{,
  (int)(-1) sendMode, /**< The firing mode for sending. -1 -> don't send. */
  (int)(-1) receiveMode, /**< The firing mode assumed for received readings. -1 -> ignore. */
});
