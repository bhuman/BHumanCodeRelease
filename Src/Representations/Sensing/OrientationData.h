/**
* @file OrientationData.h
* Declaration of class OrientationData.
* @author Colin Graf
*/

#pragma once

#include "Tools/Math/RotationMatrix.h"
#include "Tools/Streams/AutoStreamable.h"

/**
* @class OrientationData
* Encapsulates the orientation and velocity of the torso.
*/
STREAMABLE(OrientationData,
{,
  (RotationMatrix) rotation, /**< The rotation of the torso. */
  (Vector3<>) velocity, /**< The velocity along the x-, y- and z-axis relative to the torso. (in m/s) */
});

class GroundTruthOrientationData  : public OrientationData {};
