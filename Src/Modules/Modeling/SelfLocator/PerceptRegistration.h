/**
 * @file PerceptRegistration.h
 *
 * Declaration of a class that uses recent field feature observations
 * and associates them with their real world pendants.
 *
 * @author <A href="mailto:tlaue@uni-bremen.de">Tim Laue</A>
 */

#pragma once

#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Modeling/OwnSideModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Perception/FieldFeatures/GoalFrame.h"
#include "Representations/Perception/FieldFeatures/MidCircle.h"
#include "Representations/Perception/FieldFeatures/MidCorner.h"
#include "Representations/Perception/FieldFeatures/OuterCorner.h"
#include "Representations/Perception/FieldFeatures/PenaltyArea.h"
#include "Representations/Perception/FieldPercepts/CirclePercept.h"
#include "Representations/Perception/FieldPercepts/FieldLineIntersections.h"
#include "Representations/Perception/FieldPercepts/FieldLines.h"
#include "Representations/Perception/FieldPercepts/PenaltyMarkPercept.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Tools/RingBuffer.h"
#include "Tools/Debugging/DebugDrawings.h"

struct RegisteredLine
{
  Vector2f pStart;
  Vector2f pEnd;
  Vector2f pCenter;
  Vector2f pDir;
  Matrix2f covCenter;
  Vector2f wStart;
  Vector2f wEnd;
  bool vertical;
};

struct RegisteredLandmark
{
  Vector2f p;
  Vector2f w;
  Matrix2f cov;
};

struct RegisteredPose
{
  Pose2f p;
  Pose2f pose;
};

/**
 * @struct RegisteredPercepts
 * A set of <percept, model> pairs
 */
struct RegisteredPercepts
{
  void draw() const
  {
    DEBUG_DRAWING("representation:RegisteredPercepts:lines", "drawingOnField")
    {
      for(const auto& l : lines)
      {
        LINE("representation:RegisteredPercepts:lines", l.wStart.x(), l.wStart.y(), l.wEnd.x(), l.wEnd.y(),
             100, Drawings::solidPen, ColorRGBA(255, 128, 0, 128));
      }
    }
    DEBUG_DRAWING("representation:RegisteredPercepts:landmarks", "drawingOnField")
    {
      for(const auto& l : landmarks)
      {
        CIRCLE("representation:RegisteredPercepts:landmarks", l.w.x(), l.w.y(), 300,
               100, Drawings::solidPen, ColorRGBA(255, 128, 0, 128), Drawings::noBrush, ColorRGBA::black);
      }
    }
  }

  std::vector<RegisteredLine>     lines;
  std::vector<RegisteredLandmark> landmarks;
  std::vector<RegisteredPose>     poses;
  int totalNumberOfPerceivedLines;
  int totalNumberOfPerceivedLandmarks;
  int totalNumberOfPerceivedPoses;
};

class PerceptRegistration
{
private:
  const CameraInfo& theCameraInfo;
  const CameraMatrix& theCameraMatrix;
  const CirclePercept& theCirclePercept;
  const FieldDimensions& theFieldDimensions;
  const FrameInfo& theFrameInfo;
  const GameInfo& theGameInfo;
  const OwnTeamInfo& theOwnTeamInfo;
  const GoalFrame& theGoalFrame;
  const FieldLineIntersections& theFieldLineIntersections;
  const FieldLines& theFieldLines;
  const MidCircle& theMidCircle;
  const MidCorner& theMidCorner;
  const OuterCorner& theOuterCorner;
  const PenaltyArea& thePenaltyArea;
  const PenaltyMarkPercept& thePenaltyMarkPercept;

  const float& lineAssociationCorridor;
  const float& longLineAssociationCorridor;
  const float& centerCircleAssociationDistance;
  const float& penaltyMarkAssociationDistance;
  const float& intersectionAssociationDistance;
  const float& minimumLineLengthForAssociationToLongHorizontalLine;
  const float& globalPoseAssociationMaxDistanceDeviation;
  const Angle& globalPoseAssociationMaxAngularDeviation;
  const float& covarianceScalarLongHorizontalLine;        /**< If a line is long and horizontal, its computed covariance is multiplied by this number. Choosing a number less than 1 thus means that the confidence in this measurement is higher. */
  const float& minLengthLongHorizontalLine;               /**< Minimum length (in millimeters) of a line to be considered as "long" */
  const float& yDifferenceLongHorizontalLine;             /**< Maximum difference between lineStart.y and lineEnd.y in pixels */

  /**
   * A field line
   */
  class FieldLine
  {
  public:
    Vector2f start; /**< The starting point of the line. */
    Vector2f end; /**< The ending point of the line. */
    Vector2f dir; /**< The normalized direction of the line (from starting point). */
    float length; /**< The length of the line. */
    bool isLong;  /**< The line is longer than the penalty area width */
    bool vertical; /**< Whether this is a vertical or horizontal line. */
  };

  Pose2f robotPose;

  std::vector<FieldLine> verticalFieldLines;   /**< Relevant field lines  */
  std::vector<FieldLine> horizontalFieldLines; /**< Relevant field lines  */
  FieldLine* centerLine;

  Vector2f goalPosts[4];  /**< The positions of the goal posts. */
  Vector2f ownPenaltyMark;
  Vector2f opponentPenaltyMark;
  float penaltyAreaWidth;

  std::vector< Vector2f > xIntersections;
  std::vector< Vector2f > lIntersections;
  std::vector< Vector2f > tIntersections;
  float goalAcceptanceThreshold;
  Pose3f inverseCameraMatrix;
  Vector2f currentRotationDeviation;

  Matrix2f goalPostCovariance;
  Matrix2f penaltyMarkCovariance;
  Matrix2f circlePerceptCovariance;
  std::vector<Matrix2f> lineCovariances;
  std::vector<Matrix2f> intersectionCovariances;
  unsigned int lastGoalPostCovarianceUpdate;
  unsigned int lastPenaltyMarkCovarianceUpdate;
  unsigned int lastCirclePerceptCovarianceUpdate;
  std::vector<unsigned int> lineCovarianceUpdates;
  std::vector<unsigned int> intersectionCovarianceUpdates;

  int registerLines(std::vector<RegisteredLine>& lines);

  int registerLandmarks(std::vector<RegisteredLandmark>& landmarks);

  int registerPoses(std::vector<RegisteredPose>& poses);

  int registerPose(std::vector<RegisteredPose>& poses, const FieldFeature& feature);

  bool pickPoseFromFieldFeature(const Pose2f& robotPose, const FieldFeature& fieldFeature, Pose2f& pickedPose) const;

  bool getAssociatedIntersection(const FieldLineIntersections::Intersection& intersection, Vector2f& associatedIntersection) const;

  bool getAssociatedPenaltyMark(const Vector2f& penaltyMarkPercept, Vector2f& associatedPenaltyMark) const;

  bool getAssociatedGoalPost(const Vector2f& goalPostPercept, Vector2f& associatedGoalPost) const;

  const FieldLine* getPointerToAssociatedLine(const Vector2f& start, const Vector2f& end) const;

  float getSqrDistanceToLine(const Vector2f& base, const Vector2f& dir, float length, const Vector2f& point) const;

  float getSqrDistanceToLine(const Vector2f& base, const Vector2f& dir, const Vector2f& point) const;

  bool intersectLineWithLine(const Vector2f& lineBase1, const Vector2f& lineDir1,
                             const Vector2f& lineBase2, const Vector2f& lineDir2, Vector2f& intersection) const;

  bool lineCouldBeOnCenterCircle(const Vector2f& lineStart, const Vector2f& direction) const;

  bool iAmBeforeKickoffAndTheLineIsProbablyTheCenterLine(const Vector2f& lineStart, const Vector2f& lineEnd,
                                                         const Vector2f& direction, const Vector2f& orthogonal,
                                                         float length) const;

  bool iAmBeforeKickoffInTheCenterOfMyHalfLookingForward() const;

  bool lineIsLongAndHorizontalInImage(const RegisteredLine& line) const;


  void draw(const RegisteredPercepts& registeredPercepts, const Pose2f& theRobotPose);

public:
  PerceptRegistration(const CameraInfo& cameraInfo,
                      const CameraMatrix& cameraMatrix,
                      const CirclePercept& circlePercept,
                      const FieldDimensions& fieldDimensions,
                      const FrameInfo& frameInfo,
                      const GameInfo& gameInfo,
                      const OwnTeamInfo& ownTeamInfo,
                      const GoalFrame& goalFrame,
                      const FieldLineIntersections& fieldLineIntersections,
                      const FieldLines& fieldLines,
                      const MidCircle& midCircle,
                      const MidCorner& midCorner,
                      const OuterCorner& outerCorner,
                      const PenaltyArea& penaltyArea,
                      const PenaltyMarkPercept& penaltyMarkPercept,
                      bool         goalFrameIsPerceivedAsLines,
                      const float& lineAssociationCorridor,
                      const float& longLineAssociationCorridor,
                      const float& centerCircleAssociationDistance,
                      const float& penaltyMarkAssociationDistance,
                      const float& intersectionAssociationDistance,
                      const float& minimumLineLengthForAssociationToLongHorizontalLine,
                      const float& globalPoseAssociationMaxDistanceDeviation,
                      const Angle& globalPoseAssociationMaxAngularDeviation,
                      const float& covarianceScalarLongHorizontalLine,
                      const float& minLengthLongHorizontalLine,
                      const float& yDifferenceLongHorizontalLine);

  /** Computes the representation */
  void update(const Pose2f& theRobotPose, RegisteredPercepts& registeredPercepts,
              const Pose3f& inverseCameraMatrix, const Vector2f& currentRotationDeviation,
              bool drawRegisteredElements = false);
};
