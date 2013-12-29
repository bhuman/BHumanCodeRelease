/**
* @file FallDownStateDetector.h
*
* This file declares a module that provides information about the current state of the robot's body.
*
* @author <a href="mailto:maring@informatik.uni-bremen.de">Martin Ring</a>
*/

#pragma once

#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/InertiaSensorData.h"
#include "Tools/Module/Module.h"
#include "Tools/RingBufferWithSum.h"


MODULE(FallDownStateDetector)
  REQUIRES(FilteredSensorData)
  REQUIRES(InertiaSensorData)
  USES(MotionInfo)
  REQUIRES(FrameInfo)
  PROVIDES_WITH_MODIFY_AND_DRAW(FallDownState)
  LOADS_PARAMETER(int, fallTime) /**< The time (in ms) to remain in state 'falling' after a detected fall */
  LOADS_PARAMETER(float, staggeringAngleX) /**< The threshold angle which is used to detect the robot is staggering to the back or front*/
  LOADS_PARAMETER(float, staggeringAngleY) /**< The threshold angle which is used to detect the robot is staggering sidewards*/
  LOADS_PARAMETER(float, fallDownAngleY) /**< The threshold angle which is used to detect a fall to the back or front*/
  LOADS_PARAMETER(float, fallDownAngleX) /**< The threshold angle which is used to detect a sidewards fall */
  LOADS_PARAMETER(float, onGroundAngle) /**< The threshold angle which is used to detect the robot lying on the ground */
END_MODULE


/**
* @class FallDownStateDetector
*
* A module for computing the current body state from sensor data
*/
class FallDownStateDetector: public FallDownStateDetectorBase
{
private:
  /** Executes this module
  * @param fallDownState The data structure that is filled by this module
  */
  void update(FallDownState& fallDownState);

  bool isFalling();
  bool isStaggering();
  bool isCalibrated();
  bool specialSpecialAction();
  bool isUprightOrStaggering(FallDownState& fallDownState);
  FallDownState::Direction directionOf(float angleX, float angleY);
  FallDownState::Sidestate sidewardsOf(FallDownState::Direction dir);

  unsigned lastFallDetected;

  /** Indices for buffers of sensor data */
  ENUM(BufferEntry, accX, accY, accZ);

  /** Buffers for averaging sensor data */
  RingBufferWithSum<float, 15> buffers[numOfBufferEntrys];

public:
  /**
  * Default constructor.
  */
  FallDownStateDetector();
};
