/**
 * @file BHumanMessageOutputGenerator.h
 *
 * @author <A href="mailto:jesse@tzi.de">Jesse Richter-Klug</A>
 */

#pragma once

#include "Streaming/Function.h"
#include "Streaming/AutoStreamable.h"
#include "Tools/Communication/BHumanMessage.h"

/**
 * A struct that can generate the BHumanMessage,
 *     that this robot wants to send to its teammates.
 */
STREAMABLE_WITH_BASE(BHumanMessageOutputGenerator, BHumanMessage,
{
  FUNCTION(void()) send;
  FUNCTION(bool()) sendThisFrame,

  (unsigned)(0) sentMessages, /**< count of sent messages */
});
