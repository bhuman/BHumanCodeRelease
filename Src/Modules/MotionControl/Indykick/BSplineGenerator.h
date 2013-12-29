/**
 * @file BSplineGenerator.h
 * Declarations of the class to generate a cubic b-spline in bezier representation.
 * @author Felix Wenk
 */

#pragma once

#include "Tools/Math/Matrix.h"
#include "FrontContour.h"
#include "BKickBase.h"
#include "BezierBase.h"
#include "Tools/Streams/AutoStreamable.h"

class TorsoMatrix;
class RobotModel;
class RobotDimensions;

class BSplineGenerator
{
public:
  STREAMABLE(Parameters,
  {,
    (float) kickHeight, /**< Height of the kick foot relative to the ball radius (1 => kick the ball at its middle). 0 <= kickHeight <= 2. */
    (float) footRotationFactor, /**< Determines how much of the direction angle is achieved by rotating the kick foot. */
    (BKickPhase, PhaseId) firstBSplinePhase,
    (float) strikeoutHeight, /**< Height of the strike out and swing out points relative to the support foot in [mm]. */
    (float) heightOffset, /**< Offset in [mm] to add to the z coordinates to counter unobservable 'leaning over' of the robot. */
    (float) circleOffset1, /**< Offset in circle-of-the-ball-radii to move the first ball contact phase reference point in front of the ball. */
    (float) circleOffset2, /**< Offset in circle-of-the-ball-radii to move the second ball contact phase reference point behind the ball. */
    (float) strikeoutBallDistanceLimit, /**< Maximum distance between the ball and the strike out reference point. */
    (float) swingoutBallDistanceLimit, /**< Maximum distance between the ball and the swing out reference point. */
  });

  BSplineGenerator(const TorsoMatrix& theTorsoMatrix, const RobotModel& theRobotModel,
                   const RobotDimensions& theRobotDimensions);

  void init(const FrontContour& footFrontContour, const float kickDuration,
            const float defaultKickSpeed);
  void setDuration(const unsigned newDuration);
  void generateEndpoints(const BKickPhase::PhaseId phaseId,
                         const IndykickRequest& request,
                         const KickFootPose& standKickFootPose);
  void generateStartpoints(const BKickPhase::PhaseId phaseId,
                           const KickFootPose& initialKickFootPose);
  BezierCurve getPositions(const BKickPhase::PhaseId phaseId, KickFootPose::Reference& referenceCoordinateSystem) const;
  AngleBezierCurve getAngles(const BKickPhase::PhaseId phaseId, KickFootPose::Reference& referenceCoordinateSystem) const;
  bool modifiesDuration() const;
  int getDuration(const BKickPhase::PhaseId phaseId) const;
  float getDelta(const BKickPhase::PhaseId phaseId) const;
  BezierCurveReference getPositionsReference(const BKickPhase::PhaseId phaseId, KickFootPose::Reference& referenceCoordinateSystem) const;
  AngleBezierCurveReference getAnglesReference(const BKickPhase::PhaseId phaseId, KickFootPose::Reference& referenceCoordinateSystem) const;
  /** The coordinate system in which the bezier curve reference points are saved in. */
  KickFootPose::Reference referenceCoordinateSystem;

  float calculateLengthOfSegment(const int segment);

  /**
   * Draws the deBoor points into the 3d debug drawing
   * "module:IndykickEngine:deboor".
   * The deBoor points are in
   * the coordinate system of the kick leg pelvis.
   */
  void drawDeBoorPoints();
  void drawSpline();
  void drawSegment(int j);
  void drawMaximumOfSegment(const int segment);
  void drawPointOnSplineWithTangent(const int segment, const float localKnot);
  void drawReferencePoints();
  void performDeBoorAlgorithm(Vector3<> intermediates[4][4], const int segment, const float uStart, const float localKnot);
  void drawStrikeOutBall(const bool left) const;

  Parameters p;

  Vector3<> ballPositionInSupportFoot; /**< Ball position in support foot coordinates. Public for drawing. */
  Vector3<> kickDirectionInSupportFoot; /**< Kick direction in support foot coordinates. */

  Vector3<> contactTangent; /**< The tangent on the kick foot contour where the foot hits the ball. For drawing. */
  Vector3<> contactPoint; /**< The point on the kick foot which hits the ball. For drawing. */

  /*
   * Attributes to calculate the strike out and swing out points. Only saved here for drawing.
   * All vectors are in support foot coordinates.
   */
  float strikeoutCircleRadius;
  Vector2<> ballPlane;
  Vector2<> ballPlane1;
  Vector2<> ballPlane2;
  Vector2<> pelvisPlane;
  Vector2<> strikeOut;
  Vector2<> swingOut;

private:
  void generateSpline(const KickFootPose& standKickFootPose);
  void generateReferencePoints(const KickFootPose& standKickFootPose);
  void generateStartAndEndReferencePoint(const KickFootPose& kickFoot, const Pose3D& supportFootInKickHip);
  void calculateStrikeoutPointInSupportFoot();
  void calculateStrikeoutPoint(const Pose3D& supportFootInKickPelvis);
  void generateBallContactReferencePointsInSupportFoot();
  void generateBallContactReferencePoints(const Pose3D& supportFootInKickHip);
  void generateDeltasAlt();
  void updateBallContactPhaseDelta();
  void generateDeltasWithLengths();
  void calculateDeBoorMatrixFromDeltas();
  void generateDeBoorPointsAlt();
  void generateBezierPoints();
  bool refineSpeedDS();
  float calculateMaximumOfSegment(const int segment, Vector3<>& maximum);
  bool calculateMaximumOfSubsegment(Vector3<> intermediates[4][4], const int segment, const float segmentUStart,
                                     float uMin, float uMax, float valueMin, float valueMax, float& uMaximum, Vector3<>& maximum);
  void evaluateSegment(const int segment, const float localKnot, Vector3<>& point);

  /** Helper function to map all deltas before 0 and after (n-1) to zero. */
  float delta(int i) const;
  Vector3<>& deBoor(const int i, const bool rotation);

  IndykickRequest request; /**< The kick request for which to generate the spline curve. */
  const float ballRadius;
  float circleOfTheBallRadius;
  float kickDuration;
  float kickSpeed;
  FrontContour footFrontContour;
  int durations[BKickPhase::numOfPhaseIds];

  enum
  {
    numReferencePoints = 6,
    numBezierPoints = 3 * numReferencePoints - 2,
    numDeltas = numReferencePoints - 1 /**< Number of deltas that aren't clipped to 0. */
  };

  Vector3<> referencePointsPosition[numReferencePoints];
  Vector3<> bezierPointsPosition[numBezierPoints];
  Vector3<> deBoorPointsPosition[numReferencePoints - 2]; /* Four de Boor points are fixed due to boundary and end conditions. */
  Vector3<> referencePointsRotation[numReferencePoints];
  Vector3<> bezierPointsRotation[numBezierPoints];
  Vector3<> deBoorPointsRotation[numReferencePoints - 2];
  float deltas[numDeltas];
  /*
   * Coefficients for the de Boor (and reference) points in the
   * system of equations to calculate the de Boor points.
   */
  float alphas[numReferencePoints];
  float betas[numReferencePoints];
  float gammas[numReferencePoints];
  float omegas[numReferencePoints];
  Matrix4x4f deBoorMatrix;

  const TorsoMatrix& theTorsoMatrix;
  const RobotModel& theRobotModel;
  const RobotDimensions& theRobotDimensions;
};
