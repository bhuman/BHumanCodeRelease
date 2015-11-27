#pragma once

#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/Sensing/JointDataPrediction.h"
#include "Representations/Sensing/JointVelocities.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Modeling/PT2.h"
#include "Tools/Module/Module.h"
#include "Tools/RingBuffer.h"
#include <deque>
#include <vector>


MODULE(JointDataPredictor,
{,
  REQUIRES(JointAngles),
  REQUIRES(JointVelocities),
  REQUIRES(FrameInfo),
  REQUIRES(RobotDimensions),
  REQUIRES(MassCalibration),
  USES(JointRequest), //NOTE: This has to be the JointRequest from last frame, this is the case as long as MotionCombinator requires the prediction
  PROVIDES(JointDataPrediction),
  LOADS_PARAMETERS(
  {,
    (PT2) headYaw,
    (PT2) headPitch,
    (PT2) lShoulderPitch,
    (PT2) lShoulderRoll,
    (PT2) lElbowYaw,
    (PT2) lElbowRoll,
    (PT2) lWristYaw,
    (PT2) lHand,
    (PT2) rShoulderPitch,
    (PT2) rShoulderRoll,
    (PT2) rElbowYaw,
    (PT2) rElbowRoll,
    (PT2) rWristYaw,
    (PT2) rHand,
    (PT2) lHipYawPitch,
    (PT2) lHipRoll,
    (PT2) lHipPitch,
    (PT2) lKneePitch,
    (PT2) lAnklePitch,
    (PT2) lAnkleRoll,
    (PT2) rHipYawPitch,
    (PT2) rHipRoll,
    (PT2) rHipPitch,
    (PT2) rKneePitch,
    (PT2) rAnklePitch,
    (PT2) rAnkleRoll,
    (int) requestSize,
    (float) (0.005f) iterationDt, /*<The dt used to iterate the model. The real dt should be divisible by this */
    (float) (0.1f) predictVariance,/**<Variances for the kalmanfilter used to filter the positions */
    (float) (0.9f) updateVariance,
    (double) (0.0001) minimumVelocity, /**Velocities below this value will be set to zero to avoid numerical instabilities */
    //debugging parameters:
    (int) debugCompareSize,

  }),
});

/**
 * Predicts the current state of the motor using a model of the motor and the
 * last joint requests.
 */
class JointDataPredictor: public JointDataPredictorBase
{
private:
  std::vector<PT2> models; /**<One model for each motor. Index 0 = lHipYawPitch */
  /**The last n joint requests.*/
  std::deque<JointRequest> requests;

  /**The last n predictions.
   * Used for evaluation and testing only*/
  std::deque<JointDataPrediction> predictions;
  /**used inside iterateModels() to calculate the com acceleration at the end */
  RingBuffer<float, 3> lastPositions[Joints::numOfJoints];

  /**JointData from last frame. Used to calculate joint velocity*/
  JointAngles lastJointData;
  
  /**True if filterPostions() has never been called*/
  bool firstFilterRun;

public:
  JointDataPredictor();

  void update(JointDataPrediction& prediction);

private:
  /**initialize the models using the current joint data and joint velocities*/
  void initializeModels();
  /**iterate the joint models using the last 3 requests and update the prediction.
  *  Also writes the last 3 positions of each joint to lastPositions.*/
  void iterateModels(JointDataPrediction& prediction);
  /**updates the com based on lastPositions*/
  void updateCom(JointDataPrediction& prediction) const;
  /**Declares all the debugging stuff, plots, debug responses, etc.*/
};
