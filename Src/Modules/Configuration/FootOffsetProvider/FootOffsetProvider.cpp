/*
 * @file FootOffsetProvider.cpp
 * Calibrates the positions of the soles of the feet. With this the robots know the real position of their feet relative to their model.
 * This model is based on the joint angles and the IMU. If one or both are wrong, at least the robot knows the correct feet position relative.
 * @author Philip Reichenberg
 */

#include "FootOffsetProvider.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include <cmath>

MAKE_MODULE(FootOffsetProvider, sensing);

FootOffsetProvider::FootOffsetProvider()
{
  trajectoryIndex = -1;
  timestamp = 0;
  state = none;
  startJoints.angles = theJointAngles.angles;
  currentTrajectoryLeft = currentTrajectoryRight = Vector2f::Zero();
  //movement command in all 4 directions.
  //forward and backward are the same for both feet.
  //the inner offset for both feet are calculated based on the other offsets.
  trajectory.emplace_back(Vector2f(footSpeed, 0));
  trajectory.emplace_back(Vector2f(-footSpeed, 0));
  trajectory.emplace_back(Vector2f(0, footSpeed));
  trajectory.emplace_back(Vector2f(0, -footSpeed));

  InMapFile stream("footOffset.cfg");
  ASSERT(stream.exists());
  stream >> const_cast<FootOffset&>(theFootOffset);
}

void FootOffsetProvider::update(FootOffset& footOffset)
{
  footOffset = newFootOffsets;
}

void FootOffsetProvider::update(JointRequest& jointRequest)
{
  DECLARE_DEBUG_DRAWING3D("module:FootOffsetProvider:comInFloor", "robot");
  bool start = false;
  bool next = false;
  DEBUG_RESPONSE("module:FootOffsetProvider:start")
    start = true;
  DEBUG_RESPONSE_ONCE("module:FootOffsetProvider:next")
    next = true;

  if(!start)
    return;
  //COM in foot plane calculation must be the same as used in the walking
  //Where is my COM in the left foot plane?
  Pose3f comInTorsoLeft = theTorsoMatrix.rotation * theRobotModel.centerOfMass;
  Pose3f soleInTorsoMatrixLeft = theTorsoMatrix.rotation * theRobotModel.soleLeft;
  Pose3f comInFloorLeft = comInTorsoLeft * Vector3f(0, 0, soleInTorsoMatrixLeft.translation.z() - comInTorsoLeft.translation.z());
  Pose3f comInFloorInRobotLeft = theTorsoMatrix.rotation.inverse() * comInFloorLeft;
  CROSS3D("module:FootOffsetProvider:comInFloor", comInFloorInRobotLeft.translation.x(), comInFloorInRobotLeft.translation.y(), comInFloorInRobotLeft.translation.z(), 3, 3, ColorRGBA::orange);
  Vector2f diffLeft = theRobotModel.soleLeft.translation.head<2>() - comInFloorInRobotLeft.translation.head<2>();

  //Where is my COM in the right foot plane?
  Pose3f comInTorsoRight = theTorsoMatrix.rotation * theRobotModel.centerOfMass;
  Pose3f soleInTorsoMatrixRight = theTorsoMatrix.rotation * theRobotModel.soleRight;
  Pose3f comInFloorRight = comInTorsoRight * Vector3f(0, 0, soleInTorsoMatrixRight.translation.z() - comInTorsoRight.translation.z());
  Pose3f comInFloorInRobotRight = theTorsoMatrix.rotation.inverse() * comInFloorRight;
  CROSS3D("module:FootOffsetProvider:comInFloor", comInFloorInRobotRight.translation.x(), comInFloorInRobotRight.translation.y(), comInFloorInRobotRight.translation.z(), 3, 5, ColorRGBA::red);
  Vector2f diffRight = theRobotModel.soleRight.translation.head<2>() - comInFloorInRobotRight.translation.head<2>();

  switch(state)
  {
    // default position
    case none:
    {
      if(timestamp == 0)
      {
        timestamp = theFrameInfo.time;
        static_cast<void>(InverseKinematic::calcLegJoints(Vector3f(0, footOffsetY, standHeight), Vector3f(0, -footOffsetY, standHeight), Vector2f::Zero(), targetJoints, theRobotDimensions));
        for(std::size_t i = 0; i < Joints::firstLeftLegJoint; i++)
        {
          targetJoints.angles[i] = 0;
          targetJoints.stiffnessData.stiffnesses[i] = nonLegStiffness;
        }
        for(std::size_t i = Joints::firstLegJoint; i < Joints::numOfJoints; i++)
        {
          targetJoints.stiffnessData.stiffnesses[i] = legStiffness;
        }
        targetJoints.angles[Joints::lShoulderPitch] = 90_deg;
        targetJoints.angles[Joints::rShoulderPitch] = 90_deg;
        targetJoints.angles[Joints::lShoulderRoll] = 90_deg;
        targetJoints.angles[Joints::rShoulderRoll] = -90_deg;
      }
      state = returnStand;
      break;
    }
    // wait for user input
    case wait:
    {
      if(next)
      {
        trajectoryIndex++;
        trajectoryIndex = trajectoryIndex % static_cast<int>(trajectory.size());
        currentTrajectoryLeft = Vector2f(0, footOffsetY);
        currentTrajectoryRight = Vector2f(0, -footOffsetY);
        state = moving;
      }
      break;
    }
    // move feet
    case moving:
    {
      // movement direction
      Vector2f addMovement = trajectory[trajectoryIndex] * Constants::motionCycleTime;
      if(std::abs(theInertialData.gyro.y()) > slowThreshold) // gyro is high, therefore slow down
      {
        addMovement /= slowFootSpeedFactor;
        addMovement /= slowFootSpeedFactor;
      }
      // calulcate feet tranlation
      currentTrajectoryLeft += addMovement;
      currentTrajectoryRight += addMovement;
      Vector3f traLeft = Vector3f(currentTrajectoryLeft.x(), currentTrajectoryLeft.y(), standHeight);
      Vector3f traRight = Vector3f(currentTrajectoryRight.x(), currentTrajectoryRight.y(), standHeight);
      static_cast<void>(InverseKinematic::calcLegJoints(traLeft, traRight, Vector2f::Zero(), jointRequest, theRobotDimensions)); // calculate joint request

      // gyro measurement is too high, therefor assume a fall
      if(std::abs(theInertialData.gyro.x()) > gyroThresholdX || std::abs(theInertialData.gyro.y()) > gyroThresholdY)
      {
        state = waitForFall;
        returnStandTimestamp = theFrameInfo.time;
        difLeftCopy = diffLeft;
        difRightCopy = diffRight;
      }
      break;
    }

    // wait if the robot is actually falling
    case waitForFall:
    {
      // if gyro is not high anymore, go back to moving state
      if(std::abs(theInertialData.gyro.x()) > gyroThresholdX || std::abs(theInertialData.gyro.y()) > gyroThresholdY)
      {
        // gyro was long enough high -> robot was falling, so go back to default stand
        if(theFrameInfo.getTimeSince(returnStandTimestamp) > timeUntilReturnStand)
        {
          //calc offset for foot;
          state = returnStand;
          timestamp = theFrameInfo.time;
          startJoints = jointRequest;
          static_cast<void>(InverseKinematic::calcLegJoints(Vector3f(0, footOffsetY, standHeight), Vector3f(0, -footOffsetY, standHeight), Vector2f::Zero(), targetJoints, theRobotDimensions));
          if(trajectoryIndex == 0)
            newFootOffsets.backward = std::abs(difRightCopy.x());
          else if(trajectoryIndex == 1)
            newFootOffsets.forward = std::abs(difRightCopy.x());
          else if(trajectoryIndex == 2)
            newFootOffsets.rightFoot.right = std::abs(difRightCopy.y());
          else if(trajectoryIndex == 3)
            newFootOffsets.leftFoot.left = std::abs(difLeftCopy.y());

          // trajectoryIndex was 3, therefor we know that a measured all 4 sides -> calibrate the foot offsets
          //because the robot is supported by both feet most of the time for the calibration process,
          //the inner sole edges are calculated based on the measured values and the documentation values
          if(trajectoryIndex == 3)
          {
            float originWidth = theRobotDimensions.soleToOuterEdgeLength + theRobotDimensions.soleToInnerEdgeLength;
            float originLength = theRobotDimensions.soleToFrontEdgeLength + theRobotDimensions.soleToBackEdgeLength;
            float newLength = newFootOffsets.forward + newFootOffsets.backward;
            float ratio = newLength / originLength;
            float newWidth = originWidth * ratio;
            newFootOffsets.leftFoot.right = newWidth - newFootOffsets.leftFoot.left;
            newFootOffsets.rightFoot.left = newWidth - newFootOffsets.rightFoot.right;
          }
          OUTPUT_TEXT("-----------");
          OUTPUT_TEXT(trajectoryIndex);
          OUTPUT_TEXT("left: " << difLeftCopy.x() << " " << difLeftCopy.y());
          OUTPUT_TEXT("right: " << difRightCopy.x() << " " << difRightCopy.y());
        }
      }
      else
        state = moving;
      break;
    }

    // return to default stand position
    case returnStand:
    {
      float ratio = std::min(theFrameInfo.getTimeSince(timestamp) / interpolationTime, 1.f);
      MotionUtilities::interpolate(startJoints, targetJoints, ratio, jointRequest, theJointAngles);
      if(ratio >= 1.f)
        state = wait;
      break;
    }
  }
}
