/**
 * @file RefereeDetectionRequest.h
 *
 * This file defines a representation that modules can require to determine whether
 * the referee should be detected and act accordingly.
 *
 * @author Ayleen Lührsen
 */

#pragma once

#include "Streaming/AutoStreamable.h"

STREAMABLE(RefereeDetectionRequest,
{,
  (bool)(false) detectReferee, /**< Activate the referee detection? */
});
