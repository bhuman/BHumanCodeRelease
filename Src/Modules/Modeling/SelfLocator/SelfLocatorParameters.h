/**
* @file SelfLocatorParameters.h
*
* Declaration of all parameters of the self-locator
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
*/

#pragma once

#include "Tools/Math/Pose2D.h"
#include "Tools/Streams/AutoStreamable.h"

/**
* A collection of all parameters of the self-locator.
*/
STREAMABLE(SelfLocatorParameters,
{,
  (int) numberOfSamples,                 /**< The number of samples used by the self-locator */
  (Pose2D) defaultPoseDeviation,         /**< Standard deviation used for creating new hypotheses */
  (Pose2D) filterProcessDeviation,       /**< The process noise for estimating the robot pose. */
  (Pose2D) odometryDeviation,            /**< The percentage inaccuracy of the odometry. */
  (Vector2<>) odometryRotationDeviation, /**< A rotation deviation of each walked mm. */
  (float) goalAssociationMaxAngle,
  (float) goalAssociationMaxAngularDistance,
  (float) centerCircleAssociationDistance,
  (float) lineAssociationCorridor, /**< The corridor used for relating seen lines with field lines. */
  (float) cornerAssociationDistance,
  (Vector2<>) robotRotationDeviation, /**< Deviation of the rotation of the robot's torso */
  (Vector2<>) robotRotationDeviationInStand, /**< Deviation of the rotation of the robot's torso when he is standing. */
  (float) standardDeviationBallAngle,
  (float) standardDeviationBallDistance,
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
  (bool) alwaysAssumeOpponentHalf,
  (float) maxBallVelocity,
  (int) minContinuousArmContact,         /**< The minimum time of continuous arm contact to be considered disturbing (in ms) */
  (float) maxDistanceToFieldCenterForMirrorActions,
  (float) baseValidityWeighting,
  (float) sideConfidenceConfident,       /**< Value for side confidence when being definitely in own half */
  (float) sideConfidenceAlmostConfident, /**< Value for side confidence when being sure about own position but not definitely in own half */
  (float) sideConfidenceConfused,        /**< Value for side confidence when bad things have happened */
  (float) goalieFieldBorderDistanceThreshold,
  (float) goalieNoPerceptsThreshold,
});
