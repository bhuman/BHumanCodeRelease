/**
 * @file JPEGImageRequest.h
 *
 * This file defines a representation that allows to initiate the compression
 * of the \c CameraImage to a \c JPEGImage in the background.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Streaming/AutoStreamable.h"
#include "Streaming/Function.h"

STREAMABLE(JPEGImageRequest,
{
  FUNCTION(void()) startCompression, /**< Start compressing the \c CameraImage to a \c JPEGImage. */
});
