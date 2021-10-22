/**
 * @file PerceptionProviders.h
 *
 * This file declares all modules that provide representations from perception
 * for the current Cognition frame.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Threads/Cognition.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Tools/Module/Module.h"

// The perception threads already draw these representations
#undef _MODULE_DRAW
#define _MODULE_DRAW(...)

// Declare aliases of representation for lower and upper camera threads
#define DECLARE(Representation) \
  STREAMABLE_WITH_BASE(Lower##Representation, Representation, {, }); \
  STREAMABLE_WITH_BASE(Upper##Representation, Representation, {, })

// Require both aliases and provide representation
#define SELECTS(Representation) \
  REQUIRES(Lower##Representation), \
  REQUIRES(Upper##Representation), \
  PROVIDES(Representation)

// Needed by Cognition.cpp
DECLARE(FrameInfo);

// Define a module for the ImageCoordinateSystem, because it has to integrate CameraInfo
DECLARE(ImageCoordinateSystem);

MODULE(PerceptionImageCoordinateSystemProvider,
{,
  REQUIRES(CameraInfo),
  SELECTS(ImageCoordinateSystem),
});

class PerceptionImageCoordinateSystemProvider : public PerceptionImageCoordinateSystemProviderBase
{
  void update(ImageCoordinateSystem& theImageCoordinateSystem) override;
};
