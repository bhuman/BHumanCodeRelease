/**
 * @file RobotStableStateProvider.h
 * TODO
 * @author Philip Reichenberg
 */

#pragma once

#include "Representations/Configuration/FootOffset.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Sensing/FootSupport.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Sensing/RobotStableState.h"
#include "Representations/Sensing/RobotModel.h"
#include "Framework/Module.h"

MODULE(RobotStableStateProvider,
{,
  REQUIRES(FrameInfo),
  REQUIRES(InertialData),
  REQUIRES(FootOffset),
  REQUIRES(FootSupport),
  REQUIRES(MassCalibration),
  USES(MotionInfo),
  REQUIRES(RobotModel),
  PROVIDES(RobotStableState),
  DEFINES_PARAMETERS(
  {,
    (unsigned)(3) predictionSteps,
    (unsigned)(20) turnPointSteps,
  }),
});

class RobotStableStateProvider : public RobotStableStateProviderBase
{
public:
  RobotStableStateProvider();
private:

  void update(RobotStableState& theRobotStableState) override;

  /**
   * Calculate the support polygon
   * @param isLeftPhase is the left foot the swing foot
   */
  void calcSupportPolygon(const bool isLeftPhase);
  /**
   * Calculate the intersection point of the CoM movement vector and the the support polygon edges
   * @param currentCom current CoM
   * @param lastCom previous CoM
   * @param intersection3D the resulting intersection point
   *
   * @return whether the movement vector intersects the support polygon
   */
  bool getTiltingPoint(const Vector3f& currentCom, const Vector3f& lastCom, Vector3f& intersection3D);
  /**
   * Predict the rotation matrix
   * @param state TheRobotStableState
   * @param rotationMatrix current rotation matrix
   * @param isLeftPhase is left foot the swing foot?
   * @param prediction if false, only predict for one frame
   * @param findTurnPoint if true, the prediction is repeated until the velocity in the roll direction changes its sign
   */
  void predictRotation(RobotStableState& state, RotationMatrix rotationMatrix,
                       const RobotModel& model, const bool isLeftPhase,
                       const bool prediction, const bool findTurnPoint);

  void calculateCoMInPositionPercent(RobotStableState& theRobotStableState);

  /**
   * Calculate the % position of the CoM, given the 0%, 50%, 75% and 100% points.
   */
  float calcPercentInFeet(const float refPoint, const float p0, const float p05, const float p075, const float p1);

  std::vector<Vector3f> supportPolygon; // simple support polygon
  Vector2f supportFootCenter; // center of the support polygon
  unsigned int lastUpdate = 0; // Timestamp of last update
  bool lastIsLeftPhase;
  RotationMatrix turnPointMatrix;
  unsigned int lastTurnPointMatrixUpdate = 0;

  MassCalibration lightMassCalibration;

  enum PredictReturnType
  {
    none,
    noIntersection,
    velocityChanged,
  };
};
