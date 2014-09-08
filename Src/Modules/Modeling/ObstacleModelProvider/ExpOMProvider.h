/**
 * @file ExpOMProvider.h
 *
 * Declaration of a module that combines arm contacts, foot bumper contacts, obstacle percepts
 * (from the module RobotPerceptor) and ultra sonic obstacles into one obstacle model.
 *
 * @author Florin
 */
#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Perception/RobotPercept.h"
#include "Representations/Modeling/ExpObstacleModel.h"
#include "Representations/Sensing/ArmContactModel.h"
#include "Representations/Sensing/FootContactModel.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Perception/ImageCoordinateSystem.h"

MODULE(ExpOMProvider,
{,
  REQUIRES(RobotPercept),
  REQUIRES(ArmContactModel),
  REQUIRES(FootContactModel),
  REQUIRES(USObstacleModel),
  REQUIRES(RobotModel),
  REQUIRES(TorsoMatrix),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(FrameInfo),
  REQUIRES(Odometer),
  REQUIRES(GoalPercept),
  REQUIRES(RobotInfo),
  REQUIRES(FallDownState),
  REQUIRES(GameInfo),
  REQUIRES(MotionInfo),
  REQUIRES(ImageCoordinateSystem),
  PROVIDES_WITH_MODIFY_AND_DRAW(ExpObstacleModel),
  LOADS_PARAMETERS(
  {,
    (unsigned) (10) deleteValue, /**< Delete obstacles after N seconds of last measurement */
    (Vector2<>) (0.02f, 0.06f) pRobotRotationDeviation, /**< Deviation of the rotation of the robot's torso */
    (Vector2<>) (0.02f, 0.04f) pRobotRotationDeviationInStand, /**< Deviation of the rotation of the robot's torso while standing */
    (float) (1.f) mergeDistanceSqr, /**< Distance to merge obstacles */
    (float) (2.f) goalMergeDistanceSqr, /**< Distance to merge goalposts */
    (float) (2.f) minimalDistancetoGoalpostSqr, /**< Other obstacles in front of goalposts should not be merged into a goalpost */
    (unsigned) (4) minPercepts, /**< Minimal amount of percepts to become an obstacle */
    (bool) (false) debug, /**< Flag for some debug stuff */
    (float) (.2f) maxGoalAngle, /**< Used for eliminating noisy goal percepts */
    (float) (1000.f) maxGoalToGoalDistSqr, /**< Used for eliminating noisy goal percepts */
    (float) (0.1f) varianceVel, /**< Unit: mm per ms. Variance of velocity for probably moving robots */
    (float) (0.f) unknownVarianceVelocity, /**< Unit: mm per ms. Variance of velocity for unknown objects */
    (float) (0.01f) minVelocity, /**< Unit: mm per ms. Lower velocity will be set to 0 */
    (float) (10.f) maxVelocity, /**< Unit: mm per ms. Obstacle with higher velocity will be removed */
    (float) (80.f) robotDepth, /**< Unit: mm. How far is the center of a robot behind the position reported by perceptor */
    (int) (4) colorThreshold, /**< Only switch color if this threshold is reached */
    (int) (4) uprightThreshold, /**< Only switch upright/fallen if this threshold is reached */
    (float) (3.f) lowerDistOtherGoalSqr,
    (float) (100.f) pNp2,
    (float) (1.f) pNv2,
  }),
});

class InternalExpObstacle : public ExpObstacleModel::ExpObstacle
{
  friend class ExpOMProvider;
private:
  InternalExpObstacle(const Matrix2x2<> cov, const Vector2<> pCenter, const Type t, const unsigned time, const unsigned sC, const float varianceVel);
  InternalExpObstacle(const Matrix2x2<> cov, const Vector2<> pCenter, const unsigned time, const unsigned sC, const float varianceVel);

  int color = 0;
  int upright = 0;
};

class ExpOMProvider : public ExpOMProviderBase
{
private:
  void update(ExpObstacleModel& expObstacleModel);
  void addArmContacts();
  void addFootContacts();
  void addUSObstacles();
  void addRobotPercepts();
  void dynamic();
  void mergeObstacles();
  void deleteObstacles();
  void addGoalPercepts();
  void propagateObstacles(ExpObstacleModel& expObstacleModel);
  void considerType(const InternalExpObstacle& z, InternalExpObstacle& old);
  void tryToMerge(const InternalExpObstacle& obstacle);
  Matrix2x2<> getCovOfPointInWorld(const Vector2<>& pointInWorld2) const;
  void mergeOverlapping();
  //float mahalanobisDistance(const InternalExpObstacle& one, const InternalExpObstacle& two) const;
  //EKF
  void dynamic(InternalExpObstacle& obstacle);
  void measurement(InternalExpObstacle& obstacle, const InternalExpObstacle& z);

  std::vector<InternalExpObstacle> iWantToBeAnObstacle;

  unsigned lastFrame = 0;
  float minDistance = 9999.f, maxDistance = 0.f, minAngle = 9.f, maxAngle = 0.f; //debugOutput

  std::vector<bool> merged; /**< This is to merge obstacles once for every "percept" per frame */
  float deltaTime = 0;
  unsigned maxPercepts = 0;
  const Matrix2x2<> feetCov = Matrix2x2<>(900.f, 0.f, 6400.f, 0.f); /**< 30mm standard deviation and 80mm standard deviation */
  const Matrix2x2<> armCov = Matrix2x2<>(2500.f, 0.f, 2500.f, 0.f); /**< 50mm standard deviation */
};
