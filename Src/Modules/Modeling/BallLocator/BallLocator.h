/**
* @file BallLocator.h
* Declares a class that estimates the position of the ball using a couple of Kalman filters.
* @author Colin Graf
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Sensing/RobotModel.h"
#include "Tools/Math/Vector.h"
#include "Tools/Math/Matrix.h"

MODULE(BallLocator)
  REQUIRES(BallPercept)
  REQUIRES(OdometryData)
  REQUIRES(FrameInfo)
  REQUIRES(FieldDimensions)
  REQUIRES(CameraMatrix)
  REQUIRES(CameraInfo)
  REQUIRES(ImageCoordinateSystem)
  REQUIRES(TorsoMatrix)
  REQUIRES(RobotModel)
  REQUIRES(RobotDimensions)
  USES(BallModel)
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(BallModel)
  DEFINES_PARAMETER(Vector4f, processDeviation, Vector4f(0.1f, 0.1f, 1.f, 1.f)) /**< The process noise. (petite) */
  DEFINES_PARAMETER(Vector2<>, robotRotationDeviation, Vector2<>(0.02f, 0.08f)) /**< Deviation of the rotation of the robot's torso */
  DEFINES_PARAMETER(Pose2D, odometryDeviation, Pose2D(0.5f, 0.5f, 0.5f)) /**< The percentage inaccuracy of the odometry */
  DEFINES_PARAMETER(float, initialStateWeight, 0.1f) /**< The weight of newly created states (between >0 and <1) */
  DEFINES_PARAMETER(Vector2<>, footOffset, Vector2<>(65.f, 0.f)) /**< Offset from foot ankle point to a "center" of the foot used for approximating the foot shape with a circle */
  DEFINES_PARAMETER(float, footRadius, 50.f) /**< The raidus of the approximated foot shape */
  DEFINES_PARAMETER(float, footMass, 0.1f) /**< An assumed mass for each foot (in kg) */
  DEFINES_PARAMETER(float, ballMass, 0.05f) /**< The mass of the ball (in kg) */
  DEFINES_PARAMETER(Vector2<>, kickDeviation, Vector2<>(1.f, 1.f)) /**< The percentage inaccuracy of passed velocities */
  DEFINES_PARAMETER(unsigned, ballNotSeenTimeout, 200)
  DEFINES_PARAMETER(unsigned, ballNotUpdatedTimeout, 8000)
END_MODULE

/**
 * @class BallLocator
 * A BallLocator using some Kalman filters.
 */
class BallLocator : public BallLocatorBase
{
public:
  /**
  * Default constructor.
  */
  BallLocator();

private:
  struct State
  {
    ENUM(Type,
      moving,
      stationary
    );

    Type type;
    float gain;
    float weight;
    float height;
    int age;

    Vector4f movingX;
    Matrix4x4f movingCov;

    Vector2f stationaryX;
    Matrix2x2f stationaryCov;
  };

  float deltaTime; /**< Time difference in seconds to previous image */

  State states[12];
  unsigned int stateCount;
  State* bestState;

  bool hasLastBallPercept;
  unsigned int lastBallPerceptTimeStamp;
  Vector2f lastBallPercept;
  Matrix2x2f lastBallPerceptCov;

  OdometryData lastOdometryData;
  unsigned int lastFrameTime;

  Vector2<> lastLeftFootCenter;
  Vector2<> lastRightFootCenter;
  Vector2<> lastBallPosition;

  unsigned timeNotSeen; /**< the time the ball was not seen altough it should have been visible (in ms) */
  unsigned firstDisappearance; /**< Time stamp of the last frame after which the ball disappearance was detected */
  bool ballWasBeenSeenInLastLowerCameraImage;
  bool ballWasSeenInThisFrame;

  /**
  * Initialize something.
  */
  void init();

  /**
  * Deletes all filters
  */
  void reset();

  /**
  * Provides ball model representation
  */
  void update(BallModel& ballModel);

  void motionUpdate(BallModel& ballModel);
  void handleCollisionWithFeet(const Vector2<>& leftFootCenter, const Vector2<>& rightFootCenter);
  void sensorUpdate(const Vector2f& measurement, const Matrix2x2f& measurementCov);
  void normalizeWeights(State*& bestState, State*& worstStationaryState, State*& worstMovingState);
  void createNewStates(const Vector2f& ballPercept, const Matrix2x2f& ballPerceptCov, State* worstStationaryState, State* worstMovingState);
  void generateModel(BallModel& ballModel);

  Matrix2x2f getCovOfPixelInWorld(const Vector2<>& correctedPointInImage, float pointZInWorld) const;
  float getUnscaledProbabilityAt(const Vector2f& mean, const Matrix2x2f& cov, const Vector2f& pos) const;
  float getProbabilityAtMean(const Matrix2x2f& cov) const;
  bool getSmallestLineWithCircleIntersectionFactor(const Vector2<>& lineBase, const Vector2<>& lineDir, const Vector2<>& circleBase, float circleRadius, float& factor) const;
  bool getLineWithLineIntersectionFactors(const Vector2<>& lineBase1, const Vector2<>& lineDir1, const Vector2<>& lineBase2, const Vector2<>& lineDir2, float& factor1, float& factor2) const;
};
