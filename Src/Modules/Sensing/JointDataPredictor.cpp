
#include "JointDataPredictor.h"
#include "Representations/Sensing/RobotModel.h"
#include "Platform/BHAssert.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Debugging/DebugDrawings.h"

using namespace std;

MAKE_MODULE(JointDataPredictor, sensing)

JointDataPredictor::JointDataPredictor()
{
  models.push_back(headYaw);
  models.push_back(headPitch);
  models.push_back(lShoulderPitch);
  models.push_back(lShoulderRoll);
  models.push_back(lElbowYaw);
  models.push_back(lElbowRoll);
  models.push_back(lWristYaw);
  models.push_back(lHand);
  models.push_back(rShoulderPitch);
  models.push_back(rShoulderRoll);
  models.push_back(rElbowYaw);
  models.push_back(rElbowRoll);
  models.push_back(rWristYaw);
  models.push_back(rHand);
  models.push_back(lHipYawPitch);
  models.push_back(lHipRoll);
  models.push_back(lHipPitch);
  models.push_back(lKneePitch);
  models.push_back(lAnklePitch);
  models.push_back(lAnkleRoll);
  models.push_back(rHipYawPitch);
  models.push_back(rHipRoll);
  models.push_back(rHipPitch);
  models.push_back(rKneePitch);
  models.push_back(rAnklePitch);
  models.push_back(rAnkleRoll);

  firstFilterRun = true;
}

void JointDataPredictor::update(JointDataPrediction &prediction)
{
  prediction.valid = false;
  requests.push_back(theJointRequest);
    //we can do nothing if there are not enough buffered joint requests
  if(requests.size() == (size_t)requestSize)
  {
    initializeModels();
    iterateModels(prediction);
    updateCom(prediction);//depends on iterateModels(), dont call before
    requests.pop_front();
    prediction.valid = true;
    prediction.predictedRobotModel.setJointData(prediction, theRobotDimensions, theMassCalibration);

  }
  else if(requests.size() > (size_t)requestSize)
  {//this happens if requestSize is decreased via SimRobot
    requests.clear();
  }
  lastJointData = theJointAngles;
}

void JointDataPredictor::initializeModels()
{
  ASSERT(models.size() == Joints::numOfJoints);
  for(int i = 0; i < Joints::numOfJoints; ++i)
  {
    models[i].initialize(theJointAngles.angles[i], theJointVelocities.velocities[i]);
  }
}

void JointDataPredictor::iterateModels(JointDataPrediction& prediction)
{
  ASSERT(requests.size() == (size_t)requestSize);
  const int steps = int(round(theFrameInfo.cycleTime / iterationDt));
  for(int i = 0; i < Joints::numOfJoints; ++i)
  {

    if(models[i].isValid() && theJointRequest.angles[i] != JointAngles::off)
    {
      for(size_t j = 0; j < requests.size(); ++j)//0 is the oldest request
      {
        for(int s = 0; s < steps; ++s)
        {
          models[i].step(requests[j].angles[i], iterationDt);
          lastPositions[i].push_front(models[i].getPosition());
        }
      }
      prediction.angles[i] = models[i].getPosition();
      prediction.velocities[i] = models[i].getVelocity();
      prediction.accelerations[i] = models[i].getAcceleration();
    }
    else
    {
      if(theJointRequest.angles[i] == JointAngles::off)
      {//if the request is off: use the measured data as prediction
        prediction.angles[i] = theJointAngles.angles[i];
      }
      else
      {//if the model is invalid simply assume that the joint has reached its destination
        prediction.angles[i] = theJointRequest.angles[i];
      }
      prediction.velocities[i] = 0;
      prediction.accelerations[i] = 0;      
      lastPositions[i].push_front(prediction.angles[i]);
      lastPositions[i].push_front(prediction.angles[i]);
      lastPositions[i].push_front(prediction.angles[i]);
    }
  }
}

void JointDataPredictor::updateCom(JointDataPrediction& prediction) const
{
  //throughout this function index 0 means 'newest'
  RobotModel models[3];
  ASSERT(lastPositions[0].full());
  for(int j = 0; j < 3; ++j)
  {
    //copy j'th joint positions to a temporary JointData and use the RobotModel
    //to calc the center of mass
    JointAngles joints;
    for(int i = 0; i < Joints::numOfJoints; ++i)
    {
      joints.angles[i] = lastPositions[i][j];
    }
    models[j].setJointData(joints, theRobotDimensions, theMassCalibration);
  }
  //calc com velocities and acc
  /*If the robot did not move the coms will be very close together. When subtracting
    nearly equal numbers we get a lot of floating point error. Thats why we use double
    and clip the velocities if they are too small.*/
  Vector3d diff0 = (models[0].centerOfMass - models[1].centerOfMass).cast<double>();
  if(abs(diff0.x()) < minimumVelocity)
  {
    diff0.x() = 0.0;
  }
  if(abs(diff0.y()) < minimumVelocity)
  {
    diff0.y() = 0.0;
  }

  if(abs(diff0.z()) < minimumVelocity)
  {
    diff0.z() = 0.0;
  }

  Vector3d diff1 = (models[1].centerOfMass - models[2].centerOfMass).cast<double>();
  if(abs(diff1.x()) < minimumVelocity)
  {
    diff1.x() = 0.0;
  }
  if(abs(diff1.y()) < minimumVelocity)
  {
    diff1.y() = 0.0;
  }

  if(abs(diff1.z()) < minimumVelocity)
  {
    diff1.z() = 0.0;
  }
  const Vector3d vel1 = diff1 / iterationDt;
  const Vector3d vel0 = diff0 / iterationDt;
  prediction.comVelocity = vel0.cast<float>();
  prediction.comAcceleration = ((vel0 - vel1) / iterationDt).cast<float>();
  prediction.com = models[0].centerOfMass;

  PLOT("module:JointDataPredictor:com0VelX", vel0.x());
  PLOT("module:JointDataPredictor:com0VelY", vel0.y());
  PLOT("module:JointDataPredictor:com1VelX", vel1.x());
  PLOT("module:JointDataPredictor:com1VelY", vel0.y());
}

