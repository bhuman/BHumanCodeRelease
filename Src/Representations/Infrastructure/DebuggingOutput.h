/**
 * @file DebuggingOutput.h
 *
 * This file defines a representation that encapsulates a string array for
 * debugging output that can be logged.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(DebuggingOutput,
{,
  (std::vector<std::string>) text,
});
