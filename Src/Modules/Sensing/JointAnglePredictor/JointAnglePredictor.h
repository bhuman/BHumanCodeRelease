/**
 * @file JointAnglePredictor.h
 *
 * This file declares a module that provides JointAngle predictions.
 *
 * @author Jan Fiedler
 */

#pragma once

#include "Framework/Module.h"
#include "MathBase/RingBuffer.h"
#include "Platform/File.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Sensing/JointAnglePred.h"

//#include <CompiledNN/CompiledNN.h>
#include <CompiledNN2ONNX/CompiledNN.h>

using namespace NeuralNetworkONNX;

MODULE(JointAnglePredictor,
{,
  USES(JointRequest), // It must be ensured that the data comes from the last frame.
  REQUIRES(JointSensorData),
  USES(MotionInfo),
  PROVIDES(JointAnglePred),
  LOADS_PARAMETERS(
  {,
    (std::string) modelName,  /**< The file name (relative to "NeuralNets/JointAngle") from which to load the model.*/
    (unsigned) historyLength, /**< The amount of frames of past frames the model uses as input. */
  }),
});

class JointAnglePredictor : public JointAnglePredictorBase
{
public:
  JointAnglePredictor() : network(&Global::getAsmjitRuntime()) { compile(false); }

private:
  // Save input data from old to new.
  RingBuffer<ENUM_INDEXED_ARRAY(Angle, Joints::Joint)> jointSensorAngles{ historyLength }; // ..., -2, -1, -0
  RingBuffer<ENUM_INDEXED_ARRAY(Angle, Joints::Joint)> jointRequestAngles{ historyLength }; // ...,-3, -2, -1

  // Model.
  const std::string modelPath = std::string(File::getBHDir()) + "/Config/NeuralNets/JointAngle/";
  CompiledNN network; /**< The compiled neural network. */

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theJointAnglePred The representation updated.
   */
  void update(JointAnglePred& theJointAnglePred) override;
  /**
   * Compile the model.
   * @param output Whether to output information.
   */
  void compile(bool output);
};
