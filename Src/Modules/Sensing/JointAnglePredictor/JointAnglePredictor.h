/**
 * @file JointAnglePredictor.h
 *
 * This file declares a module that provides JointAngle predictions.
 *
 * @author Jan Fiedler
 */

#pragma once

#include "Framework/Module.h"
#include "MathBase/RingBufferWithSum.h"
#include "MathBase/RingBuffer.h"
#include "Platform/File.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Sensing/JointAnglePred.h"

//#include <CompiledNN/CompiledNN.h>
#include <CompiledNN2ONNX/CompiledNN.h>
#include <filesystem>

using namespace NeuralNetworkONNX;

MODULE(JointAnglePredictor,
{,
  REQUIRES(FrameInfo),
  USES(JointRequest),
  REQUIRES(JointSensorData),
  PROVIDES(JointAnglePred),
  LOADS_PARAMETERS(
  {,
    (std::string) modelName,  /**< The file name (relative to "NeuralNets/JointAngle") from which to load the model.*/
  }),
});

class JointAnglePredictor : public JointAnglePredictorBase
{
public:
  JointAnglePredictor() : network(&Global::getAsmjitRuntime()) { compile(false); }

private:
  static constexpr unsigned frames = 3;
  RingBuffer<ENUM_INDEXED_ARRAY(Angle, Joints::Joint), frames> jointSensorAngles; // -2, -1, -0
  RingBuffer<ENUM_INDEXED_ARRAY(Angle, Joints::Joint), frames> jointRequestAngles; // -3, -2, -1
  // TODO: This order is from pybh and should be corrected later.
  std::array<Joints::Joint, 11> jointOrder =
  {
    Joints::Joint::rAnkleRoll, Joints::Joint::rAnklePitch, Joints::Joint::rKneePitch, Joints::Joint::rHipPitch, Joints::Joint::rHipRoll,
    Joints::Joint::lKneePitch, Joints::Joint::lAnkleRoll, Joints::Joint::lAnklePitch, Joints::Joint::lHipPitch, Joints::Joint::lHipRoll,
    Joints::Joint::lHipYawPitch,
  };

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
   * @ param output Whether to output information.
   */
  void compile(bool output);
};
