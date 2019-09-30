/**
 *
*******************************************************************************************
**                                                                                       **
**  THE USE OF THIS REPRESENTATION IS DEPRICATED PLEASE USE EnhancedKeyStates INSTEAD !  **
**                                                                (announced by Jesse)   **
*******************************************************************************************
 * FootBumperState.h
 *
 *  Created on: Mar 14, 2012
 *      Author: arneboe@tzi.de
 *              simont@tzi.de
 */

#pragma once

#include "Tools/RobotParts/Legs.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(FootBumperState,
{
  STREAMABLE(State,
  {,
    (bool)(false) contact,        /**< do we have foot contact with this foot? */
    (int)(0) contactDuration,   /**< duration (in frames) of the current contact of the foot. 0 if no contact */
    (unsigned)(0u) lastContact,  /**< timestamp of the last contact detection of the foot */
  }),

  (State[Legs::numOfLegs]) status,
});
