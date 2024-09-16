/**
 * @file WalkOutOfBallDirectionProvider.cpp
 *
 * This file implements a module that makes the robot leave the path of the ball to prevent an
 * unintentional interception of the ball.
 *
 * @author Sina Schreiber
 */

#include "Debugging/DebugDrawings.h"
#include "Debugging/Annotation.h"
#include "WalkOutOfBallDirectionProvider.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/Modeling/BallPhysics.h"

MAKE_MODULE(WalkOutOfBallDirectionProvider);

void WalkOutOfBallDirectionProvider::update(WalkOutOfBallDirection& theWalkOutOfBallDirection)
{
  DEBUG_DRAWING("motion:RobotPose", "drawingOnField") // Set the origin to the robot's current position
  {
    ORIGIN("motion:RobotPose", theRobotPose.translation.x(), theRobotPose.translation.y(), theRobotPose.rotation);
  }
  DECLARE_DEBUG_DRAWING("module:WalkOutOfBallLineProvider:walkOutPosition", "drawingOnField");
  theWalkOutOfBallDirection.createPhase = [this](const MotionRequest& motionRequest, const MotionPhase& lastPhase, const MotionPhase& currentPhase)
  {
    Vector2f ballVelocity = motionRequest.ballEstimate.velocity;
    const bool isLeftPhase = theWalkGenerator.isNextLeftPhase(lastPhase, motionRequest.ballEstimate.position);
    const Pose3f supportInTorso3D = theTorsoMatrix * (isLeftPhase ? theRobotModel.soleRight : theRobotModel.soleLeft);
    const Pose2f supportInTorso(supportInTorso3D.rotation.getZAngle(), supportInTorso3D.translation.head<2>());
    const Pose2f hipOffset(0.f, 0.f, isLeftPhase ? -theRobotDimensions.yHipOffset : theRobotDimensions.yHipOffset);
    const Pose2f scsCognition = hipOffset * supportInTorso.inverse() * theOdometryDataPreview.inverse() * motionRequest.odometryData;
    const Vector2f positionOfGoal(theFieldDimensions.xPosOpponentGoal, 0.f);
    const Vector2f relativePositionOfGoal = scsCognition * (theRobotPose.inverse() * positionOfGoal);
    // line that tracks robot relative the path of the ball
    const Geometry::Line ballLine = Geometry::Line(scsCognition * motionRequest.ballEstimate.position, ballVelocity.rotated(scsCognition.rotation));

    const Angle ballToRobot = (-ballLine.base).angle();
    const Angle ballToGoal = (relativePositionOfGoal - ballLine.base).angle();
    const Angle diffBallToGoalAndBallToGoal = (ballToGoal - ballToRobot);

    // if the robot is behind the ball, while the ball is rolling to the opponent goal
    if(std::abs(Angle::normalize(diffBallToGoalAndBallToGoal)) > 90_deg)
      return std::unique_ptr<MotionPhase>();

    // the distance between the robot and the ball path relative to the robot
    const float distanceToBallLine = Geometry::getDistanceToLine(ballLine, Vector2f(0.f, 0.f));

    // orthogonal point to check whether the robot should walk left or right
    const Vector2f orthPoint = Geometry::getOrthogonalProjectionOfPointOnLine(ballLine, Vector2f(0.f, 0.f));

    const Angle orthPointDirection = Angle::normalize(orthPoint.angle() + 180_deg);

    Vector2f walkOutVector = Vector2f::polar(400.f, orthPointDirection) + orthPoint;
    std::vector<Vector2f> translationPolygon;
    std::vector<Vector2f> translationPolygonNoCenter;
    theWalkGenerator.getTranslationPolygon(isLeftPhase, 0.f, lastPhase, motionRequest.walkSpeed, translationPolygon, translationPolygonNoCenter, false, false);

    if(!Geometry::isPointInsideConvexPolygon(translationPolygon.data(), static_cast<int>(translationPolygon.size()), walkOutVector))
    {
      Vector2f intersectionPoint;
      VERIFY(Geometry::getIntersectionOfLineAndConvexPolygon(translationPolygon, Geometry::Line(Vector2f(0.f, 0.f), walkOutVector), intersectionPoint, false));
      walkOutVector = intersectionPoint;
    }

    if(isLeftPhase == (walkOutVector.y() < 0.f))
      walkOutVector.y() = 0.f;

    /*
     * checks if the distance between the robot position and the ball line is smaller or equal the given threshold.
     * If this is the case the robot blocks the ball path
     */
    // calculate the walk step size given by the next motion phase (currentPhase)
    Pose2f currentStepSize = theWalkGenerator.getLastStepChange(currentPhase);
    // line that tracks robot relative the path of the ball after calculation the current step size
    const Geometry::Line ballLineWithCurrentStepSize = Geometry::Line(currentStepSize.inverse() * scsCognition * motionRequest.ballEstimate.position, ballVelocity.rotated(scsCognition.rotation - currentStepSize.rotation));
    // the distance between the robot and the ball path relative to the robot after calculating the current step size
    const float distanceToBallLineWithCurrentStepSize = Geometry::getDistanceToLine(ballLineWithCurrentStepSize, Vector2f(0.f, 0.f));

    if((distanceToBallLineWithCurrentStepSize <= robotWidthThreshold + theBallSpecification.radius
        || distanceToBallLine <= robotWidthThreshold + theBallSpecification.radius))
    {
      if(theFrameInfo.getTimeSince(annotationTimestamp) > annotationTime)
      {
        ANNOTATION("module:WalkOutOfBallDirectionProvider", "is active");
        annotationTimestamp = theFrameInfo.time;
      }
      // transform from motion coordinate system into cognition coordinate system for the drawing on the field
      auto debugVector = scsCognition.inverse() * walkOutVector;
      CROSS("module:WalkOutOfBallLineProvider:walkOutPosition", debugVector.x(), debugVector.y(), 100, 20, Drawings::solidPen, ColorRGBA::red);
      //if the robot unintentionally blocks the ball path the robot should make a sidestep
      return theWalkGenerator.createPhase(walkOutVector, lastPhase, 0.f);
    }

    // if the robot should not leave the position no motion set is set.
    return std::unique_ptr<MotionPhase>();
  };
}
