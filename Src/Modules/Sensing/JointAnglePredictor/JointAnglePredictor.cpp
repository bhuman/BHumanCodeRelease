/**
 * @file JointAnglePredictor.cpp
 *
 * This file implements a module that provides JointAngle predictions.
 *
 * @author Jan Fiedler
 */

#include "JointAnglePredictor.h"
#include "Debugging/Annotation.h"
#include "Platform/SystemCall.h"

#include <filesystem>

MAKE_MODULE(JointAnglePredictor);

void JointAnglePredictor::update(JointAnglePred& theJointAnglePred)
{
  // Set model name in the first frame.
  if(theJointAnglePred.modelName.empty())
    theJointAnglePred.modelName = modelName;

  // At least in 2019 theJointRequest.timestamp was accidentally  == 0.
  // The JointRequest must be from the previous frame. This is not ensured by USES currently.
  ASSERT(theJointRequest.timestamp == 0
         || theJointRequest.timestamp < theJointSensorData.timestamp
         || SystemCall::getMode() == SystemCall::logFileReplay
         || SystemCall::getMode() == SystemCall::remoteRobot);

  DEBUG_RESPONSE_ONCE("module:JointAnglePredictor:compile")
  {
    compile(true);
    theJointAnglePred.modelName = modelName;
  }

  // Add the data from this frame.
  jointSensorAngles.push_front(theJointSensorData.angles);
  // Add Request from last frame. == USES(JointRequest)
  jointRequestAngles.push_front(theJointRequest.angles);

  // Fill all joints with ignore values.
  theJointAnglePred.angles.fill(SensorData::ignore);
  // TODO: It would probably be more correct if the output would be valid 1 or 2 motion cycles earlier.
  // The model output is only valid during walking and is otherwise undefined.
  theJointAnglePred.isValid = theMotionInfo.isMotion(MotionPhase::walk);

  // Do noting until data is present and only calculate the predictions if the results are valid.
  if(!jointSensorAngles.full() || !jointRequestAngles.full() || !theJointAnglePred.isValid)
  {
    theJointAnglePred.isValid = false;
    return;
  }

  // Add input to model.
  float* input = network.input(0).data();
  // Add values from old to new and first requests and then the sensor values.
  for(int i = historyLength - 1; i >= 0; i--)
  {
    // Skip lHipYawPitch and use hipYawPitch==rHipYawPitch.
    for(std::size_t joint = Joints::firstLegJoint + 1; joint < Joints::numOfJoints; joint++)
      *input++ = jointRequestAngles[i][joint];
    for(std::size_t joint = Joints::firstLegJoint + 1; joint < Joints::numOfJoints; joint++)
      *input++ = jointSensorAngles[i][joint];
  }

  // Run network.
  STOPWATCH("module:JointAnglePredictor:apply")
    network.apply();

  const float* output = network.output(0).data();
  // Override joints that are provided. Skip lHipYawPitch and use hipYawPitch==rHipYawPitch.
  for(std::size_t joint = Joints::firstLegJoint + 1; joint < Joints::numOfJoints; joint++)
    theJointAnglePred.angles[joint] = *output++;
  theJointAnglePred.angles[Joints::lHipYawPitch] = theJointAnglePred.angles[Joints::rHipYawPitch];
}

void JointAnglePredictor::compile(bool output)
{
  if(output)
  {
    if(!std::filesystem::exists(modelPath + modelName))
    {
      OUTPUT_ERROR("File " << modelPath << modelName << " does not exist");
      return;
    }
  }
  else
    ASSERT(std::filesystem::exists(modelPath + modelName));

  network.compile(Model(modelPath + modelName));
  ASSERT(network.valid());

  // Input shape: (historyLength, 22)
  ASSERT(network.numOfInputs() == 1);
  ASSERT(network.input(0).rank() == 2); // (Batch, Time, Features)
  ASSERT(network.input(0).dims(0) == historyLength);
  ASSERT(network.input(0).dims(1) == 22); // == Request + Sensor

  // Output shape: (1, 11)
  ASSERT(network.numOfOutputs() == 1);
  ASSERT(network.output(0).rank() == 2); // (Batch, Time, Features)
  ASSERT(network.output(0).dims(0) == 1);
  ASSERT(network.output(0).dims(1) == 11); // == Sensor

  if(output)
    ANNOTATION("JointAnglePredictor", "Change model to " + modelName);
}
