/**
 * @file PerceptionProviders.cpp
 *
 * This file implements all modules that provide representations from perception
 * for the current Cognition frame.
 *
 * @author Thomas Röfer
 */

#include "PerceptionProviders.h"

MAKE_MODULE(PerceptionImageCoordinateSystemProvider, infrastructure);

void PerceptionImageCoordinateSystemProvider::update(ImageCoordinateSystem& theImageCoordinateSystem)
{
  if(Cognition::isUpper)
  {
    theImageCoordinateSystem = theUpperImageCoordinateSystem;
    theImageCoordinateSystem.cameraInfo = theCameraInfo;
  }
  else
  {
    theImageCoordinateSystem = theLowerImageCoordinateSystem;
    theImageCoordinateSystem.cameraInfo = theCameraInfo;
  }
}

// Define a module that updates a representation with the newer one of both aliases
#define ALIAS_MODULE(Representation) \
  MODULE(Perception##Representation##Provider, \
  {, \
    SELECTS(Representation), \
  }); \
  \
  class Perception##Representation##Provider : public Perception##Representation##ProviderBase \
  { \
    void update(Representation& the##Representation) override \
    { \
      if(Cognition::isUpper) \
        the##Representation = theUpper##Representation; \
      else \
        the##Representation = theLower##Representation; \
    } \
  }; \
  \
  MAKE_MODULE(Perception##Representation##Provider, infrastructure)

// Also declare alias representations
#define ALIAS(Representation) \
  DECLARE(Representation); \
  \
  ALIAS_MODULE(Representation)

ALIAS_MODULE(FrameInfo);
ALIAS(BallPercept);
ALIAS(BodyContour);
ALIAS(CameraInfo);
ALIAS(CameraMatrix);
ALIAS(CameraStatus);
ALIAS(CirclePercept);
ALIAS(FieldBoundary);
ALIAS(FieldLines);
ALIAS(FieldLineIntersections);
ALIAS(IntersectionsPercept);
ALIAS(LinesPercept);
ALIAS(ObstaclesFieldPercept);
ALIAS(PenaltyMarkPercept);
ALIAS(RobotCameraMatrix);
