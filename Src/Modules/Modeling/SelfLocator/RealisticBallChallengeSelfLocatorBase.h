/**
 * @file RealisticBallChallengeSelfLocatorBase.h
 * The base class of the self locator. It is defined separately, because
 * it is used in several submodules.
 *
 * @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
 */
#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Modeling/OwnSideModel.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/FieldBoundary.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Perception/LinePercept.h"
#include "Representations/Perception/PenaltyMarkPercept.h"

MODULE(RealisticBallChallengeSelfLocator,
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
  REQUIRES(CameraInfo),
  REQUIRES(BallModel),
  REQUIRES(FieldBoundary),
  REQUIRES(RobotPose),
  REQUIRES(SideConfidence),
  REQUIRES(PenaltyMarkPercept),
  USES(MotionRequest),
  PROVIDES(RobotPose),
  LOADS_PARAMETERS(
  {,
    (int) numberOfSamples,                 /**< The number of samples used by the self-locator */
    (Pose2f) defaultPoseDeviation,         /**< Standard deviation used for creating new hypotheses */
    (Pose2f) filterProcessDeviation,       /**< The process noise for estimating the robot pose. */
    (Pose2f) odometryDeviation,            /**< The percentage inaccuracy of the odometry. */
    (Vector2f) odometryRotationDeviation, /**< A rotation deviation of each walked mm. */
    (float) goalAssociationMaxAngle,
    (float) goalAssociationMaxAngularDistance,
    (float) centerCircleAssociationDistance,
    (float) penaltyMarkAssociationDistance,
    (float) lineAssociationCorridor, /**< The corridor used for relating seen lines with field lines. */
    (float) cornerAssociationDistance,
    (float) centerCircleGoalPostMaxDistanceDiff,
    (Vector2f) robotRotationDeviation, /**< Deviation of the rotation of the robot's torso */
    (Vector2f) robotRotationDeviationInStand, /**< Deviation of the rotation of the robot's torso when he is standing. */
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
    (float) positionJumpNotificationDistance,
  }),
});
