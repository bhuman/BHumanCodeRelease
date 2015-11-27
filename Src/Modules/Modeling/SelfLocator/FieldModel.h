/**
* @file FieldModel.h
*
* This file declares a submodule that represents the robot's environment for self-localization
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
* @author Colin Graf
*/

#pragma once

#include "SelfLocatorBase.h"

/**
* @class FieldModel
*
* A class for mapping observations of a
* certain class to the closest positions in the field model.
*/
class FieldModel
{
public:
  /**
  * A field line relative to the robot.
  */
  class FieldLine
  {
  public:
    Vector2f start; /**< The starting point of the line. */
    Vector2f end; /**< The ending point of the line. */
    Vector2f dir; /**< The normalized direction of the line (from starting point). */
    float length; /**< The length of the line. */
    bool vertical; /**< Whether this is a vertical or horizontal line. */
  };

  std::vector<FieldLine> fieldLines; /**< Relevant field lines  */

private:
  Vector2f goalPosts[4];  /**< The positions of the goal posts. */
  Vector2f ownPenaltyMark;
  Vector2f opponentPenaltyMark;
  const SelfLocatorBase::Parameters& parameters;
  const CameraMatrix& cameraMatrix;
  std::vector< Vector2f > xCorners;
  std::vector< Vector2f > lCorners;
  std::vector< Vector2f > tCorners;
  float unknownGoalAcceptanceThreshold;
  float knownGoalAcceptanceThreshold;

public:
  /**
   * Constructor, initialized the field model
  */
  FieldModel(const FieldDimensions& fieldDimensions, const SelfLocatorBase::Parameters& parameters, const CameraMatrix& cameraMatrix);

  bool getAssociatedPenaltyMark(const Pose2f& robotPose, const Vector2f& penaltyMarkPercept, Vector2f& associatedPenaltyMark) const;
  
  bool getAssociatedUnknownGoalPost(const Pose2f& robotPose, const Vector2f& goalPercept, Vector2f& associatedPost) const;

  bool getAssociatedKnownGoalPost(const Pose2f& robotPose, const Vector2f& goalPercept, bool isLeft, Vector2f& associatedPost) const;

  bool getAssociatedCorner(const Pose2f& robotPose, const LinePercept::Intersection& intersection, Vector2f& associatedCorner) const;

  int getIndexOfAssociatedLine(const Pose2f& robotPose, const Vector2f& start, const Vector2f& end) const;

private:
  float getSqrDistanceToLine(const Vector2f& base, const Vector2f& dir, float length, const Vector2f& point) const;

  float getSqrDistanceToLine(const Vector2f& base, const Vector2f& dir, const Vector2f& point) const;

  bool intersectLineWithLine(const Vector2f& lineBase1, const Vector2f& lineDir1, const Vector2f& lineBase2,
                             const Vector2f& lineDir2, Vector2f& intersection) const;

  bool goalPostIsValid(const Vector2f& observedPosition, const Vector2f& modelPosition,
                       const Pose2f& robotPose, float goalAcceptanceThreshold) const;
};
