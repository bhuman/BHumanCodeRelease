/**
 * @file SelfLocatorBase.h
 * The base class of the self locator. It is defined separately, because
 * it is used in several submodules.
 *
 * @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
 */
#pragma once

#include "Tools/Module/Module.h"
#include "Representations/BehaviorControl/BehaviorControlOutput.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/LocalizationTeamBall.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Modeling/OwnSideModel.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/FieldBoundary.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Perception/LinePercept.h"
#include "Representations/Sensing/ArmContactModel.h"
#include "Representations/Sensing/FallDownState.h"

MODULE(SelfLocator,
{,
  REQUIRES(Odometer),
  REQUIRES(OdometryData),
  REQUIRES(OwnSideModel),
  REQUIRES(OwnTeamInfo),
  REQUIRES(GameInfo),
  REQUIRES(RobotInfo),
  REQUIRES(GoalPercept),
  REQUIRES(LinePercept),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(MotionInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(BallModel),
  REQUIRES(FallDownState),
  REQUIRES(ArmContactModel),
  REQUIRES(FieldBoundary),
  REQUIRES(RobotPose),
  REQUIRES(SideConfidence),
  USES(LocalizationTeamBall),
  USES(BehaviorControlOutput),
  USES(MotionRequest),
  PROVIDES_WITH_OUTPUT_AND_DRAW(RobotPose),
  LOADS_PARAMETERS(
  {,
    (int) numberOfSamples,                 /**< The number of samples used by the self-locator */
    (Pose2D) defaultPoseDeviation,         /**< Standard deviation used for creating new hypotheses */
    (Pose2D) filterProcessDeviation,       /**< The process noise for estimating the robot pose. */
    (Pose2D) odometryDeviation,            /**< The percentage inaccuracy of the odometry. */
    (Vector2<>) odometryRotationDeviation, /**< A rotation deviation of each walked mm. */
    (float) goalAssociationMaxAngle,
    (float) goalAssociationMaxCloseAngle,
    (float) goalAssociationCloseThreshold,
    (float) goalAssociationFarThreshold,
    (float) goalAssociationMaxDistanceClose,
    (float) goalAssociationMaxAngularDistanceClose,
    (float) goalAssociationMaxAngularDistanceFar,
    (float) centerCircleAssociationDistance,
    (float) lineAssociationCorridor, /**< The corridor used for relating seen lines with field lines. */
    (float) cornerAssociationDistance,
    (Vector2<>) robotRotationDeviation, /**< Deviation of the rotation of the robot's torso */
    (Vector2<>) robotRotationDeviationInStand, /**< Deviation of the rotation of the robot's torso when he is standing. */
    (float) standardDeviationGoalpostSamplingDistance,
    (int) templateMaxKeepTime,
    (float) templateUnknownPostAssumptionMaxDistance,
    (float) validityThreshold,
    (float) translationNoise,
    (float) rotationNoise,
    (float) movedDistWeight,
    (float) movedAngleWeight,
    (float) majorDirTransWeight,
    (float) minorDirTransWeight,
    (float) useRotationThreshold, /**< Below this distance from the field center, rotation influences the decision between pose and its mirror (mm). */
    (bool) goalFrameIsPerceivedAsLines,
    (float) minGroundLineLength,
    (float) maxGoalPostDistFromGroundLine,
    (bool) alwaysAssumeOpponentHalf,
    (float) baseValidityWeighting,
    (float) goalieFieldBorderDistanceThreshold,
    (float) goalieNoPerceptsThreshold,
    (int) goalieJumpTimeout,
  }),
});
