/**
 * @file BHumanMessageOutputGenerator.h
 *
 * @author Jesse Richter-Klug
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
