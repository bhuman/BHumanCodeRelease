/**
 * @file JointCalibratorAuto.cpp
 *
 * @author Philip Reichenberg
 */

#include "JointCalibratorAuto.h"

MAKE_MODULE(JointCalibratorAuto, infrastructure);

void JointCalibratorAuto::update(JointCalibration& jointCalibration)
{
  if(calibrationState == State::off)
    calibration = jointCalibration;
  if(calibrationState == State::finished)
    jointCalibration = calibration;
}

void JointCalibratorAuto::update(JointRequest& jointRequest)
{
  //Start when requested
  DEBUG_RESPONSE_ONCE("module:JointCalibratorAuto:start")
    if(calibrationState == State::off)
      calibrationState = State::waiting;
  int whatToDo = 0;
  DEBUG_RESPONSE_ONCE("module:JointCalibratorAuto:calibrate")
    whatToDo = 1;
  DEBUG_RESPONSE_ONCE("module:JointCalibratorAuto:takeAsReference")
    whatToDo = 2;
  switch(calibrationState)
  {
    case State::waiting:
    {
      Vector2a torsoDif = theInertialData.angle.head<2>() - calibrationPoses[index].torsoAngle;
      if(std::abs(torsoDif.x()) < maxTorsoDif && std::abs(torsoDif.y()) < maxTorsoDif   //robot is tilted into a correct position
         && theFrameInfo.getTimeSince(switchTransition) > switchTransitionWaitTime) //robot is long enough in the tilted position
      {
        calibrationState = State::switching;
        switchTransition = theFrameInfo.time;
        calculateNextTarget(); // init start and target
        break;
      }
      else
      {
        if(!(std::abs(torsoDif.x()) < maxTorsoDif && std::abs(torsoDif.y()) < maxTorsoDif))  //robot is not tilted in the correct position
          switchTransition = theFrameInfo.time;
        if(theFrameInfo.getTimeSince(waitingResponse) > responseWaitTime)  //print current and requested torso angle
        {
          waitingResponse = theFrameInfo.time;
          OUTPUT_TEXT("--- Please tilt the robot ---");
          OUTPUT_TEXT("Goal Torsoangle: " << calibrationPoses[index].torsoAngle.x() << " - " << calibrationPoses[index].torsoAngle.y());
          OUTPUT_TEXT("Current Torsoangle: " << theInertialData.angle.x() << " - " << theInertialData.angle.y());
        }
      }
      break;
    }
    case State::switching:
    {
      float ratio = std::min(theFrameInfo.getTimeSince(switchTransition) / switchInterpolation, 1.f);
      MotionUtilities::interpolate(startJoints, targetJoints, ratio, jointRequest, theJointAngles);
      if(ratio == 1.f)
      {
        switchTransition = theFrameInfo.time;
        calibrationState = State::sampling;
        sample = std::vector<Sample>();
        sample.resize(calibrationPoses[index].measure.size());
        for(size_t i = 0; i < sample.size(); i++)
        {
          sample[i].joint = calibrationPoses[index].measure[i];
          sample[i].max = JointAngles::off;
          sample[i].min = JointAngles::off;
        }
        OUTPUT_TEXT("--- Sampling ---");
      }
      break;
    }
    case State::sampling:
    {
      if(theFrameInfo.getTimeSince(switchTransition) > samplingTime / 2)
        for(Joints::Joint joint : calibrationPoses[index].measure)
          jointRequest.stiffnessData.stiffnesses[joint] = 0;
      if(theFrameInfo.getTimeSince(switchTransition) < samplingTime && sample.size() != 0)
        for(size_t i = 0; i < sample.size(); i++)
        {
          Joints::Joint joint = sample[i].joint;
          Angle angle = theJointAngles.angles[joint];
          if(sample[i].max < angle || sample[i].max == JointAngles::off)
            sample[i].max = angle;
          if(sample[i].min > angle || sample[i].min == JointAngles::off)
            sample[i].min = angle;
        }
      else
      {
        for(size_t i = 0; i < sample.size(); i++)
        {
          measuredPositions[sample[i].joint].x() = std::min(measuredPositions[sample[i].joint].x(), sample[i].min);
          measuredPositions[sample[i].joint].y() = std::max(measuredPositions[sample[i].joint].y(), sample[i].max);
        }
        for(Joints::Joint joint : calibrationPoses[index].measure)
          jointRequest.stiffnessData.stiffnesses[joint] = stiffnessLegs;
        switchTransition = theFrameInfo.time;
        calibrationState = State::waiting;
        ++index;
        if(index >= static_cast<int>(calibrationPoses.size()))
        {
          calibrationState = State::finished;
          OUTPUT_TEXT("-------------- finished ----------------");
          OUTPUT_TEXT("dr module:JointCalibratorAuto:calibrate");
          OUTPUT_TEXT("dr module:JointCalibratorAuto:takeAsReference");
          OUTPUT_TEXT("save representation:JointCalibration");
          OUTPUT_TEXT("save parameters:JointCalibratorAuto");
          break;
        }
        OUTPUT_TEXT("-------------- NEXT ----------------");
        SystemCall::say("Next");
      }
      break;
    }
    case State::finished:
    {
      if(whatToDo == 1)
      {
        FOREACH_ENUM(Joints::Joint, joint)
        {
          if(measuredPositions[joint].x() != 0 || measuredPositions[joint].y() != 0)
          {
            Angle maxDif = theJointLimits.limits[joint].max - measuredPositions[joint].y();
            Angle minDif = theJointLimits.limits[joint].min - measuredPositions[joint].x();
            Angle measurement = std::abs(maxDif) < std::abs(minDif) ? measuredPositions[joint].y() : measuredPositions[joint].x();
            Angle ref = calibrationReference[joint];
            calibration.offsets[joint] -= ref - measurement;
          }
        }
      }
      if(whatToDo == 2)
        FOREACH_ENUM(Joints::Joint, joint)
        {
          if(measuredPositions[joint].x() != 0 || measuredPositions[joint].y() != 0)
          {
            Angle maxDif = theJointLimits.limits[joint].max - measuredPositions[joint].y();
            Angle minDif = theJointLimits.limits[joint].min - measuredPositions[joint].x();
            Angle measurement = std::abs(maxDif) < std::abs(minDif) ? measuredPositions[joint].y() : measuredPositions[joint].x();
            calibrationReference[joint] = measurement;
          }
          else
            calibrationReference[joint] = calibrationReference[joint] = 0;
        }
      break;
    }
  }
}

void JointCalibratorAuto::calculateNextTarget()
{
  startJoints.angles = targetJoints.angles = theJointAngles.angles; //init
  for(PhasePairs phase : calibrationPoses[index].jointPositions) //set changes to reach new position
    targetJoints.angles[phase.joint] = phase.position;
  FOREACH_ENUM(Joints::Joint, joint) //set default stiffness
  {
    if(joint < Joints::firstArmJoint)
      targetJoints.stiffnessData.stiffnesses[joint] = stiffnessHead;
    else if(joint < Joints::firstLegJoint)
      targetJoints.stiffnessData.stiffnesses[joint] = stiffnessArms;
    else
      targetJoints.stiffnessData.stiffnesses[joint] = stiffnessLegs;
  }
  for(Joints::Joint joint : calibrationPoses[index].measure) //set high stiffness for joints that are measured
    targetJoints.stiffnessData.stiffnesses[joint] = stiffnessMeasuredJoints;
}

JointCalibratorAuto::JointCalibratorAuto()
{
  waitingResponse = 0;
  switchTransition = 0;
  calibrationState = State::off;
  index = 0;

  calibration = JointCalibration();
  FOREACH_ENUM(Joints::Joint, joint)
    measuredPositions[joint].x() = measuredPositions[joint].y() = 0;
}
