/**
 * @file OptionalImageRequest.h
 *
 * This file defines a representation that represents the request whether an optional image should be send or not
 *
 * @author Ayleen Lührsen
 */

#pragma once

#include "Streaming/AutoStreamable.h"

STREAMABLE(OptionalImageRequest,
{,
  (bool)(false) sendImage,
});
