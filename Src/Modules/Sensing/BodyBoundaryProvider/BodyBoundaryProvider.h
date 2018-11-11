/**
 * @file BodyBoundaryProvider.h
 * Declaration of module BodyBoundaryProvider.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Sensing/BodyBoundary.h"
#include "Representations/Sensing/RobotModel.h"
#include "Tools/Debugging/DebugDrawings3D.h"

MODULE(BodyBoundaryProvider,
{,
  REQUIRES(RobotModel),
  PROVIDES(BodyBoundary),
  DEFINES_PARAMETERS(
  {,
    (BodyBoundaryParameters) bbp,
    (bool)(false) parametersChanging,
    (bool)(false) debugMode,
  }),
});

/**
 * @class BodyBoundaryProvider
 */
class BodyBoundaryProvider : public BodyBoundaryProviderBase
{
  bool first = true;

  void update(BodyBoundary& bodyBoundary) override;
};
