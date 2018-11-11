/**
 * @file DummyRepresentation.h
 *
 * The file declares a struct that can be provided by a module that
 * actually has no useful representation to provide at all.
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(DummyRepresentation,
{,
  (int)(0) dummy, /**< Yeah! */
});

STREAMABLE_WITH_BASE(DummyRepresentation2, DummyRepresentation,
{,
});
