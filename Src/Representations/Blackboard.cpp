/**
* @file Blackboard.cpp
* Implementation of a class representing the blackboard.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include "Blackboard.h"
#include <cstring>
#include <cstdlib>

Blackboard::Blackboard() :
// Initialize all representations by themselves:
// Infrastructure
  theButtonInterface(theButtonInterface),
  theJointData(theJointData),
  theJointRequest(theJointRequest),
  theSensorData(theSensorData),
  theKeyStates(theKeyStates),
  theLEDRequest(theLEDRequest),
  theImage(theImage),
  theCameraInfo(theCameraInfo),
  theFrameInfo(theFrameInfo),
  theCognitionFrameInfo(theCognitionFrameInfo),
  theRobotInfo(theRobotInfo),
  theOwnTeamInfo(theOwnTeamInfo),
  theOpponentTeamInfo(theOpponentTeamInfo),
  theGameInfo(theGameInfo),
  theTeamMateData(theTeamMateData),
  theMotionRobotHealth(theMotionRobotHealth),
  theRobotHealth(theRobotHealth),
  theTeamDataSenderOutput(theTeamDataSenderOutput),
  theUSRequest(theUSRequest),
  theThumbnail(theThumbnail),

// Configuration
  theCameraSettings(theCameraSettings),
  theFieldDimensions(theFieldDimensions),
  theRobotDimensions(theRobotDimensions),
  theJointCalibration(theJointCalibration),
  theSensorCalibration(theSensorCalibration),
  theCameraCalibration(theCameraCalibration),
  theMassCalibration(theMassCalibration),
  theHardnessSettings(theHardnessSettings),
  theDamageConfiguration(theDamageConfiguration),
  theHeadLimits(theHeadLimits),

// Perception
  theCameraMatrix(theCameraMatrix),
  theRobotCameraMatrix(theRobotCameraMatrix),
  theImageCoordinateSystem(theImageCoordinateSystem),
  theBallSpots(theBallSpots),
  theLineSpots(theLineSpots),
  thePossibleObstacleSpots(thePossibleObstacleSpots),
  theBallPercept(theBallPercept),
  theLinePercept(theLinePercept),
  theRegionPercept(theRegionPercept),
  theGoalPercept(theGoalPercept),
  theGroundContactState(theGroundContactState),
  theBodyContour(theBodyContour),
  theColorReference(theColorReference),
  theFieldBoundary(theFieldBoundary),
  theObstacleSpots(theObstacleSpots),

// Modeling
  theArmContactModel(theArmContactModel),
  theFallDownState(theFallDownState),
  theBallModel(theBallModel),
  theCombinedWorldModel(theCombinedWorldModel),
  theGroundTruthBallModel(theGroundTruthBallModel),
  theObstacleModel(theObstacleModel),
  theUSObstacleModel(theUSObstacleModel),
  theRobotPose(theRobotPose),
  theFilteredRobotPose(theFilteredRobotPose),
  theFootContactModel(theFootContactModel),
  theGroundTruthRobotPose(theGroundTruthRobotPose),
  theRobotsModel(theRobotsModel),
  theGroundTruthRobotsModel(theGroundTruthRobotsModel),
  theFreePartOfOpponentGoalModel(theFreePartOfOpponentGoalModel),
  theFieldCoverage(theFieldCoverage),
  theGlobalFieldCoverage(theGlobalFieldCoverage),
  theSideConfidence(theSideConfidence),
  theOdometer(theOdometer),
  theOwnSideModel(theOwnSideModel),
  theObstacleWheel(theObstacleWheel),
  theObstacleClusters(theObstacleClusters),

// BehaviorControl
  theActivationGraph(theActivationGraph),
  theBehaviorControlOutput(theBehaviorControlOutput),
  theBehaviorLEDRequest(theBehaviorLEDRequest),
  thePath(thePath),

// Sensing
  theFilteredJointData(theFilteredJointData),
  theFilteredSensorData(theFilteredSensorData),
  theInertiaSensorData(theInertiaSensorData),
  theOrientationData(theOrientationData),
  theGroundTruthOrientationData(theGroundTruthOrientationData),
  theTorsoMatrix(theTorsoMatrix),
  theRobotModel(theRobotModel),
  theJointDynamics(theJointDynamics),
  theFutureJointDynamics(theFutureJointDynamics),
  theRobotBalance(theRobotBalance),
  theFsrData(theFsrData),
  theFsrZmp(theFsrZmp),

// MotionControl
  theArmMotionEngineOutput(theArmMotionEngineOutput),
  theArmMotionRequest(theArmMotionRequest),
  theOdometryData(theOdometryData),
  theGroundTruthOdometryData(theGroundTruthOdometryData),
  theGetUpEngineOutput(theGetUpEngineOutput),
  theMotionRequest(theMotionRequest),
  theHeadAngleRequest(theHeadAngleRequest),
  theHeadMotionRequest(theHeadMotionRequest),
  theHeadJointRequest(theHeadJointRequest),
  theMotionSelection(theMotionSelection),
  theSpecialActionsOutput(theSpecialActionsOutput),
  theWalkingEngineOutput(theWalkingEngineOutput),
  theWalkingEngineStandOutput(theWalkingEngineStandOutput),
  theBikeEngineOutput(theBikeEngineOutput),
  theMotionInfo(theMotionInfo),
  theBallTakingOutput(theBallTakingOutput),
  theIndykickEngineOutput(theIndykickEngineOutput)
{
}

void Blackboard::operator=(const Blackboard& other)
{
  memcpy(this, &other, sizeof(Blackboard));
}

void* Blackboard::operator new(std::size_t size)
{
  return calloc(1, size);
}

void Blackboard::operator delete(void* p)
{
  return free(p);
}

void Blackboard::distract()
{
}

PROCESS_WIDE_STORAGE(Blackboard) Blackboard::theInstance = 0;
