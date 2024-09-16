/**
 * @file PerceptionProviders.h
 *
 * This file declares all modules that provide representations from perception
 * for the current Cognition frame.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"

// Declare aliases of representation for lower and upper camera threads
#define DECLARE(Representation) \
  STREAMABLE_WITH_BASE(Lower##Representation, Representation, {, }); \
  STREAMABLE_WITH_BASE(Upper##Representation, Representation, {, })

// Needed by Cognition.cpp
DECLARE(FrameInfo);

// Define a module for the ImageCoordinateSystem, because it has to integrate CameraInfo
DECLARE(ImageCoordinateSystem);
