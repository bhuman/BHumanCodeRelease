/**
 * @file DummyRepresentation.h
 *
 * This file declares a struct that can be provided by a module that
 * actually has no useful representation to provide at all.
 */

#pragma once

#include "Streaming/AutoStreamable.h"

STREAMABLE(DummyRepresentation,
{,
  (int)(0) dummy, /**< Yeah! */
});
