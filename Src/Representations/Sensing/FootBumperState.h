/**
 * FootBumperState.h
 *
 *  Created on: Mar 14, 2012
 *      Author: arneboe@tzi.de
 *              simont@tzi.de
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(FootBumperState,
{,
  (bool)(false) contactLeft,      /**< do we have foot contact with the left foot? */
  (bool)(false) contactRight,     /**< do we have foot contact with the right foot? */
  (int)(0) contactDurationLeft,   /**< duration (in frames) of the current contact of the left foot. 0 if no contact */
  (int)(0) contactDurationRight,  /**< duration (in frames) of the current contact of the right foot. 0 if no contact */
  (unsigned)(0) lastContactLeft,  /**< timestamp of the last contact detection of the left foot */
  (unsigned)(0) lastContactRight, /**< timestamp of the last contact detection of the right foot */
});
