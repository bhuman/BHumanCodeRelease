/**
 * @file OptionalCameraImage.h
 *
 * This file defines a representation that encapsulates a camera image
 * that can be provided or not.
 *
 * @author Ayleen Lührsen
 */

#pragma once

#include "Representations/Infrastructure/CameraImage.h"

STREAMABLE(OptionalCameraImage,
{,
  (std::optional<CameraImage>) image,
});
