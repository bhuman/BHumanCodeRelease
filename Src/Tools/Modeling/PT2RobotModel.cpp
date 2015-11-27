#include "PT2RobotModel.h"
#include "Platform/BHAssert.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(Parameters,
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
});

PT2RobotModel::PT2RobotModel(const std::string& configFile)
{
  ASSERT(configFile.size() > 0);
  InMapFile stream(configFile);
  ASSERT(stream.exists());
  Parameters p;
  stream >> p;

  models.push_back(p.headYaw);
  models.push_back(p.headPitch);
  models.push_back(p.lShoulderPitch);
  models.push_back(p.lShoulderRoll);
  models.push_back(p.lElbowYaw);
  models.push_back(p.lElbowRoll);
  models.push_back(p.lWristYaw);
  models.push_back(p.lHand);
  models.push_back(p.rShoulderPitch);
  models.push_back(p.rShoulderRoll);
  models.push_back(p.rElbowYaw);
  models.push_back(p.rElbowRoll);
  models.push_back(p.rWristYaw);
  models.push_back(p.rHand);
  models.push_back(p.lHipYawPitch);
  models.push_back(p.lHipRoll);
  models.push_back(p.lHipPitch);
  models.push_back(p.lKneePitch);
  models.push_back(p.lAnklePitch);
  models.push_back(p.lAnkleRoll);
  models.push_back(p.rHipYawPitch);
  models.push_back(p.rHipRoll);
  models.push_back(p.rHipPitch);
  models.push_back(p.rKneePitch);
  models.push_back(p.rAnklePitch);
  models.push_back(p.rAnkleRoll);
}

void PT2RobotModel::init(const JointDataPrediction& jointDataPrediction)
{
  for(int i = Joints::headYaw; i < Joints::numOfJoints; ++i)
  {
    ASSERT(jointDataPrediction.angles[i] != JointAngles::off &&
           jointDataPrediction.angles[i] != JointAngles::ignore);
    models[i].initialize(jointDataPrediction.angles[i], jointDataPrediction.velocities[i]);
    joints.angles[i] = jointDataPrediction.angles[i];
  }
  initialized = true;
}

void PT2RobotModel::update(const JointAngles& request, const float dt)
{
  ASSERT(models.size() == Joints::numOfJoints);
  ASSERT(initialized);
  for(int i = 0; i < Joints::numOfJoints; ++i)
  {
    ASSERT(request.angles[i] != JointAngles::off &&
           request.angles[i] != JointAngles::ignore);
    if(models[i].isValid())
    {
      models[i].step(request.angles[i], dt);
      joints.angles[i] = models[i].getPosition();
    }
    else
    {
      //if the model is not valid simply assume that the requested position
      //will be reached
      joints.angles[i] = request.angles[i];
    }
  }
}
