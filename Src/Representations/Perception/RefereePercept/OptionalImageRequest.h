/**
 * @file OptionalImageRequest.h
 *
 * This file defines a representation that represents the request whether an optional image should be send to the referee thread or not.
 *
 * @author Ayleen Lührsen
 */

#pragma once

#include "Streaming/AutoStreamable.h"

STREAMABLE(OptionalImageRequest,
{,
  (bool)(false) sendImage, /**< If true, send an image to the referee thread and disable some of the regular image processing. */
});
