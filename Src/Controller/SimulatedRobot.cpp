/**
 * @file Controller/SimulatedRobot.cpp
 * Implementation of class SimulatedRobot for SimRobotQt.
 * @author Colin Graf
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#include "SimulatedRobot.h"
#include "Controller/RoboCupCtrl.h"
#include "Platform/BHAssert.h"
#include "Platform/Time.h"
#include "Representations/Infrastructure/GroundTruthWorldState.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Tools/Math/Eigen.h"
#include <SimRobotCore2.h>
#include <SimRobotCore2D.h>
#include <QString>

SimRobot::Object* SimulatedRobot::ball = nullptr;

SimulatedRobot::SimulatedRobot(SimRobot::Object* robot) :
  robot(robot)
{
  ASSERT(robot);

  // get the robot's team color and number
  firstTeam = isFirstTeam(robot);
  robotNumber = getNumber(robot);

  const int compoundType = RoboCupCtrl::controller->is2D ? static_cast<int>(SimRobotCore2D::compound) : static_cast<int>(SimRobotCore2::compound);

  // fill arrays with pointers to other robots
  firstTeamRobots.clear();
  secondTeamRobots.clear();
  // Parse "robots" group:
  SimRobot::Object* group = RoboCupCtrl::application->resolveObject("RoboCup.robots", compoundType);
  for(unsigned currentRobot = 0, count = RoboCupCtrl::application->getObjectChildCount(*group); currentRobot < count; ++currentRobot)
  {
    SimRobot::Object* robot = RoboCupCtrl::application->getObjectChild(*group, currentRobot);
    const int number = getNumber(robot);
    if(number != robotNumber)
    {
      if(number <= 6)
        firstTeamRobots.push_back(robot);
      else
        secondTeamRobots.push_back(robot);
    }
  }
  // Parse "extras" group:
  group = RoboCupCtrl::application->resolveObject("RoboCup.extras", compoundType);
  for(unsigned currentRobot = 0, count = RoboCupCtrl::application->getObjectChildCount(*group); currentRobot < count; ++currentRobot)
  {
    SimRobot::Object* robot = RoboCupCtrl::application->getObjectChild(*group, currentRobot);
    const int number = getNumber(robot);
    if(number != robotNumber)
    {
      if(number <= 6)
        firstTeamRobots.push_back(robot);
      else
        secondTeamRobots.push_back(robot);
    }
  }
}

void SimulatedRobot::setBall(SimRobot::Object* ball)
{
  SimulatedRobot::ball = ball;
}

void SimulatedRobot::getWorldState(GroundTruthWorldState& worldState) const
{
  // Initialize world state
  worldState.firstTeamPlayers.clear();
  worldState.secondTeamPlayers.clear();
  worldState.balls.clear();

  // Get the standard ball position from the scene
  if(ball)
  {
    GroundTruthWorldState::GroundTruthBall gtBall;
    gtBall.position = getPosition3D(ball);
    if(RoboCupCtrl::controller->is2D)
      gtBall.position.z() = 50.f;
    if(firstTeam)
      gtBall.position.head<2>() *= -1.f;
    const unsigned currentTime = Time::getCurrentSystemTime();
    if(lastBallTime && lastBallTime != currentTime)
      gtBall.velocity = 1000.f * (gtBall.position - lastBallPosition) / static_cast<float>(currentTime - lastBallTime);
    else
      gtBall.velocity = Vector3f::Zero();
    lastBallPosition = gtBall.position;
    lastBallTime = currentTime;
    worldState.balls.push_back(gtBall);
  }

  // Determine the robot's own pose and number
  getRobotPose(worldState.ownPose);

  // Add all other robots that are in this scene
  for(unsigned int i = 0; i < firstTeamRobots.size(); ++i)
  {
    GroundTruthWorldState::GroundTruthPlayer newGTPlayer;
    newGTPlayer.number = getNumber(firstTeamRobots[i]);
    newGTPlayer.upright = getPose2f(firstTeamRobots[i], newGTPlayer.pose);
    if(firstTeam)
      newGTPlayer.pose = Pose2f(pi) + newGTPlayer.pose;
    worldState.firstTeamPlayers.push_back(newGTPlayer);
  }
  for(unsigned int i = 0; i < secondTeamRobots.size(); ++i)
  {
    GroundTruthWorldState::GroundTruthPlayer newGTPlayer;
    newGTPlayer.number = getNumber(secondTeamRobots[i]) - 6;
    newGTPlayer.upright = getPose2f(secondTeamRobots[i], newGTPlayer.pose);
    if(firstTeam)
      newGTPlayer.pose = Pose2f(pi) + newGTPlayer.pose;
    worldState.secondTeamPlayers.push_back(newGTPlayer);
  }
}

void SimulatedRobot::getOdometryData(const Pose2f& robotPose, OdometryData& odometryData) const
{
  static_cast<Pose2f&>(odometryData) = firstTeam ? (Pose2f(pi) + robotPose) : robotPose;
}

void SimulatedRobot::getAbsoluteBallPosition(Vector2f& ballPosition)
{
  if(ball)
    ballPosition = getPosition(ball);
}

void SimulatedRobot::moveBall(const Vector3f& pos, bool resetDynamics)
{
  if(!ball)
    return;
  Vector3f position = pos * 0.001f;
  RoboCupCtrl::controller->is2D ? static_cast<SimRobotCore2D::Body*>(ball)->move(position.data()) : static_cast<SimRobotCore2::Body*>(ball)->move(position.data());
  if(resetDynamics)
    RoboCupCtrl::controller->is2D ? static_cast<SimRobotCore2D::Body*>(ball)->resetDynamics() : static_cast<SimRobotCore2::Body*>(ball)->resetDynamics();
}

Vector2f SimulatedRobot::getPosition(const SimRobot::Object* obj)
{
  const float* position = RoboCupCtrl::controller->is2D ?
                          static_cast<const SimRobotCore2D::Body*>(obj)->getPosition() :
                          static_cast<const SimRobotCore2::Body*>(obj)->getPosition();
  return Vector2f(position[0], position[1]) * 1000.f;
}

Vector3f SimulatedRobot::getPosition3D(const SimRobot::Object* obj)
{
  const float* position = RoboCupCtrl::controller->is2D ?
                          static_cast<const SimRobotCore2D::Body*>(obj)->getPosition() :
                          static_cast<const SimRobotCore2::Body*>(obj)->getPosition();
  return Vector3f(position[0], position[1], RoboCupCtrl::controller->is2D ? 0.f : position[2]) * 1000.f;
}

void SimulatedRobot::applyBallFriction(float friction)
{
  if(!RoboCupCtrl::controller->is2D)
    return;

  Vector2f ballVelocity;
  static_cast<SimRobotCore2D::Body*>(ball)->getVelocity(ballVelocity.data());
  const float ballSpeed = ballVelocity.norm();
  const float newBallSpeed = ballSpeed + friction * RoboCupCtrl::controller->simStepLength / 1000.f;
  if(newBallSpeed <= 0.f)
    ballVelocity.setZero();
  else
    ballVelocity *= newBallSpeed / ballSpeed;
  static_cast<SimRobotCore2D::Body*>(ball)->setVelocity(ballVelocity.data());
}

bool SimulatedRobot::isFirstTeam(const SimRobot::Object* obj)
{
  return getNumber(obj) <= 6;
}

int SimulatedRobot::getNumber(const SimRobot::Object* obj)
{
  QString robotNumberString = obj->getFullName();
  int pos = robotNumberString.lastIndexOf('.');
  robotNumberString.remove(0, pos + 6);
  return robotNumberString.toInt();
}
