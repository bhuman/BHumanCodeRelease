/**
 * @file JointAnglePredictor.cpp
 *
 * This file implements a module that provides JointAngle predictions.
 *
 * @author Jan Fiedler
 */

#include "JointAnglePredictor.h"
#include "Debugging/Annotation.h"
#include "Platform/Time.h"
#include "Streaming/Global.h"
#include <CompiledNN/CompiledNN.h>

MAKE_MODULE(JointAnglePredictor);

void JointAnglePredictor::update(JointAnglePred& theJointAnglePred)
{
  if(theJointAnglePred.modelName.empty())
  {
    theJointAnglePred.modelName = modelName;
    compile(true);
  }

  // Do noting until the data is present.
  if(!jointSensorAngles.full() || !jointRequestAngles.full())
  {
    jointSensorAngles.push_front(theJointSensorData.angles);
    jointRequestAngles.push_front(theJointRequest.angles);
    return;
  }

  DEBUG_RESPONSE_ONCE("module:JointAnglePredictor:compile")
    compile(true);

  // Add tha data from this frame.
  jointSensorAngles.push_front(theJointSensorData.angles);
  jointRequestAngles.push_front(theJointRequest.angles);

  if(!jointSensorAngles.full())
    return;

  // Add input to model.
  float* input = network.input(0).data();
  // Add values from old to new and first requests and then the sensor values.
  for(int i = frames - 1; i >= 0; i--)
  {
    for(const Joints::Joint& joint : jointOrder)
      *input++ = jointRequestAngles[i][joint];
    for(const Joints::Joint& joint : jointOrder)
      *input++ = jointSensorAngles[i][joint];
  }

  // Run network.
  STOPWATCH("module:JointAnglePredictor:apply")
    network.apply();

  const float* output = network.output(0).data();
  FOREACH_ENUM(JointAnglePred::Joint, joint)
    theJointAnglePred.angles[joint] = *output++;
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

  // Input shape: (3, 22)
  ASSERT(network.numOfInputs() == 1);
  ASSERT(network.input(0).rank() == 2); // (Batch, Time, Features)
  ASSERT(network.input(0).dims(0) == frames);
  ASSERT(network.input(0).dims(1) == 22); // == Request + Sensor

  // Output shape: (1, 11)
  ASSERT(network.numOfOutputs() == 1);
  ASSERT(network.output(0).rank() == 2); // (Batch, Time, Features)
  ASSERT(network.output(0).dims(0) == 1);
  ASSERT(network.output(0).dims(1) == 11); // == Sensor

  if(output)
    ANNOTATION("JointAnglePredictor", "Change model to " + modelName);
}
