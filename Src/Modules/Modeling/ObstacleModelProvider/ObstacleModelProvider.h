/**
 * @file ObstacleModelProvider.h
 *
 * Declaration of a module that combines arm contacts,
 * foot bumper contacts and players perceptsinto one obstacle model.
 *
 * @author Florian Maaß
 */
#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/FieldBoundary.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Perception/PlayersPercept.h"
#include "Representations/Sensing/ArmContactModel.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/FootContactModel.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Tools/Debugging/Annotation.h"

MODULE(ObstacleModelProvider,
{,
 REQUIRES(ArmContactModel),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(FallDownState),
  REQUIRES(FieldBoundary),
  REQUIRES(FieldDimensions),
  REQUIRES(FootContactModel),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(GoalPercept),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(MotionInfo),
  REQUIRES(Odometer),
  REQUIRES(OwnTeamInfo),
  REQUIRES(PlayersPercept),
  REQUIRES(RobotInfo),
  REQUIRES(RobotModel),
  REQUIRES(RobotPose),
  REQUIRES(TeammateData),
  REQUIRES(TorsoMatrix),
  PROVIDES(ObstacleModel),
  LOADS_PARAMETERS(
  {,
    (unsigned) deleteAfter,                    /**< Delete obstacles that are not seen for deleteAfter amount of milliseconds */
    (Vector2f) pRobotRotationDeviation,        /**< Deviation of the rotation of the robot's torso */
    (Vector2f) pRobotRotationDeviationInStand, /**< Deviation of the rotation of the robot's torso while standing */
    (float) mergeDistanceSqr,                  /**< Distance to merge obstacles */
    (float) goalMergeDistanceSqr,              /**< Distance to merge goalposts */
    (float) minimalDistancetoGoalpostSqr,      /**< Other obstacles in front of goalposts should not be merged into a goalpost */
    (unsigned) minPercepts,                    /**< Minimal amount of percepts to become an obstacle */
    (bool) debug,                              /**< Flag for some debug stuff */
    (float) maxGoalAngle,                      /**< Used for eliminating noisy goal percepts */
    (float) maxGoalToGoalDistSqr,              /**< Used for eliminating noisy goal percepts */
    (float) maxVelocity,                       /**< Unit: mm per ms. Obstacle with higher velocity will be removed */
    (int) colorThreshold,                      /**< Only switch color if this threshold is reached */
    (int) uprightThreshold,                    /**< Only switch upright/fallen if this threshold is reached */
    (float) pNp,                               /**< Process noise for position vector */
    (float) pNv,                               /**< Process noise for velocity vector (moving objects) */
    (Vector2f) odoDeviation,                   /**< Percentage inaccuracy of the odometry */
    (int) playersPerceptGoalPerceptdiff,       /**< Image x coordinate wich must be greater between a real robot and a goal post, otherwise the players percept is probably a goal post */
    (unsigned) mergeOverlapTimeDiff,           /**< Merge overlapping models if they are measured. Avoid merging of oscillating obstacles */
    (bool) useGoalPercepts,                    /**< Enable the use of the GoalPercept */
    (unsigned) notSeenThreshold,               /**< How many times an obstacle should be seen. If that threshold is reached an obstacle will be removed */
    (unsigned) maxDistance,                    /**< Maximal distance to an obstacle */
    (bool) useArmContactModel,
    (bool) useFootContactModel,
    (bool) ignoreJerseyLowerCam, /*HACK, please remove after RoboCup*/
    (bool) ignoreJerseyUpperCam, /*HACK, please remove after RoboCup*/
  }),
});

class InternalObstacle : public Obstacle
{
  friend class ObstacleModelProvider;
private:
  InternalObstacle(const Matrix2f& covariance,
                   const Vector2f& center,
                   const Vector2f& left,
                   const Vector2f& right,
                   const unsigned lastmeasurement,
                   const unsigned seenCount,
                   const Type type);
  InternalObstacle(const Type type);
  bool isBehind(const InternalObstacle& other) const; /**< is this obstacle behind another one */
  static void closerFurther(InternalObstacle* closer, InternalObstacle* further) /**< swap if the further obstacle is closer than the 'closer' one*/
  {
    if((further->center - closer->center).squaredNorm() < .001f)
      ANNOTATION("InternalObstacle:closerFurther", "fail");
    if(further->center.squaredNorm() < closer->center.squaredNorm())
    {
      InternalObstacle* tmp = closer;
      closer = further;
      further = tmp;
    }
  }
  bool isBetween(const float leftAngle, const float rightAngle) const /**< is the left and the right vector between the two given parameters left and right angle (radian)*/
  {
    return leftAngle > left.angle() && right.angle() > rightAngle;
  }
  int color = 0;                          /**< negative value is for teammates */
  int upright = 0;                        /**< negative value is for fallen robots */
  unsigned lastMeasurement = 0;           /**< Frame of last measurement */
  unsigned seenCount = 0;                 /**< Somewhat validity (between minPercepts and maxPercepts in ObstacleModelProvider) */
  unsigned notSeenButShouldSeenCount = 0; /**< how many times the obstacle was not seen but could be measured */
};

class ObstacleModelProvider : public ObstacleModelProviderBase
{
private:
  // functions to provide the model
  void update(ObstacleModel& obstacleModel);
  void addArmContacts(); /**< add obstacles measured by arm contact */
  void addFootContacts(); /**< add obstacles measured by foot contact */
  void addPlayerPercepts(); /**< add players percepts */
  void dynamic(); /**< apply dynamic step from extended kalman filter on all obstacles */
  void deleteObstacles(); /**< delete wrong goal posts and invalid obstacles */
  void addGoalPercepts(); /**< add goal percepts */
  void considerTeammates(); /**< adjust 'color' of obstacles if we think we found our teammates */
  void propagateObstacles(ObstacleModel& ObstacleModel) const; /**< send valid obstacles to the representation */
  void considerType(InternalObstacle& obstacle, const InternalObstacle& measurement); /**< helper function to adjust the type of an obstacle by measurement */
  void tryToMerge(const InternalObstacle& obstacle); /**< find matching obstacle to one measurement */
  Matrix2f getCovOfPointInWorld(const Vector2f& pointInWorld2) const;
  void mergeOverlapping(); /**< overlapping obstacles will be merged to one obstacle */
  void shouldBeSeen(); /**< decrease seencount of obstacles that should be seen */
  bool fieldBoundaryFurtherAsObstacle(InternalObstacle& obstacle, const Vector2f centerInImage); /**< used by shouldBeSeen() */
  bool shadowedRobots(InternalObstacle* closer, const size_t i, const float cameraAngleLeft, const float cameraAngleRight); /**< used by shouldBeSeen() */
  bool isInImage(const InternalObstacle& obstacle, Vector2f centerInImage) const; /**< is an obstacle in the image */

  //extended kalman filter (due to rotation update of odometry)
  void dynamic(InternalObstacle& obstacle);
  void measurement(InternalObstacle& obstacle, const InternalObstacle& measurement);

  //two models in, one out
  //void fusion4D(InternalObstacle& one, const InternalObstacle& other);

  //some const
  const float MAX_DIST_ON_FIELD2 = 10400.f * 10400.f + 7400.f * 7400.f; /**< field diagonal */
  //to gain const one has to use finished()
  const Matrix2f feetCov = (Matrix2f() << 900.f, 0.f, 0.f, 6400.f).finished(); /**< 30mm standard deviation and 80mm standard deviation */
  const Matrix2f armCov = (Matrix2f() << 2500.f, 0.f, 0.f, 2500.f).finished(); /**< 50mm standard deviation */

  //to not spam the annotation
  bool leftArmContact = false, rightArmContact = false, leftFootContact = false, rightFootContact = false;

  std::vector<InternalObstacle> iWantToBeAnObstacle; /**< List of obstacles. */
  std::vector<bool> merged; /**< This is to merge obstacles once for every "percept" per frame */

  size_t sizeOfObstacles = 31;
};
