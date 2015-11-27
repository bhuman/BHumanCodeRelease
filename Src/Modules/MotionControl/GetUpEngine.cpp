/*
 * @file GetUpEngine.cpp
 * @author <A href="mailto:judy@tzi.de">Judith Müller</A>
 */

#include "GetUpEngine.h"
#include "Tools/Debugging/Modify.h"
#include "Representations/Infrastructure/FrameInfo.h"

using namespace std;

void GetUpEngine::update(GetUpEngineOutput& output)
{
  output.odometryOffset = Pose2f();
  if(theMotionSelection.ratios[MotionRequest::getUp] > 0.f)
  {
    if(!wasActive)
    {
      wasActive = true;
      output.tryCounter = 0;
      output.isLeavingPossible = false; //no leaving anymore
      soundTimeStamp = theFrameInfo.time;
      state = decideAction;
      //save the joint angles from last frame
      output.stiffnessData.resetToDefault();
      for(int i = 0; i < Joints::numOfJoints; ++i)
        output.angles[i] = theJointAngles.angles[i];
    }

    if(state == decideAction || state == breakUp || state == schwalbe)
      pickMotion(output); //this decides what is to do

    if(state == working) //only true if a stand up motion is initialized
    {
      setNextJoints(output);
      if(lineCounter >= maxCounter)
      {
        if(abs(theInertialData.angle.y()) > 30_deg)
        {
          setStiffness(output, 0);
          if(output.tryCounter >= maxNumOfUnsuccessfulTries)
            state = schwalbe;   //if no more trys left let behavior cry for help.
          else
          {
            if(SystemCall::getMode() != SystemCall::simulatedRobot)
              output.tryCounter++;

            state = breakUp;
            breakUpTimeStamp = theFrameInfo.time;
          }
        }
        else //Success
        {
         // if(theGroundContactState.contact) // this could go into deadlock
         // {
            if(!output.isLeavingPossible) //set odometry only once!
              output.odometryOffset = internalOdometryOffset;
            else
              output.odometryOffset = Pose2f();

            output.isLeavingPossible = true;
        //  }
        }
      }
    }
    else if(state == recover) //if a recovering motion is intialized
    {
      setNextJoints(output);
      if(lineCounter >= maxCounter)
      {
        state = decideAction;
        breakUpTimeStamp = theFrameInfo.time;
      }
    }
    else if(state == pickUp) //if a stand motion is intialized
    {
      setNextJoints(output);
      if(lineCounter >= maxCounter)
        output.isLeavingPossible = true;
    }
    // else
    /*output is the same as in the last frame*/
  }
  else
  {
    wasActive = false; //the engine did not do anything
    output.tryCounter = 0;
    lastNotActiveTimeStamp = theFrameInfo.time;
    lastMotionInfo = theMotionInfo; // store the last MotionInfo
  }
}

void GetUpEngine::interpolate(const JointAngles& from, const JointRequest& to, float& ratio, JointRequest& target)
{
  for(int i = 0; i < Joints::numOfJoints; ++i)
  {
    float f = from.angles[i];
    float t = to.angles[i];

    if(t == JointAngles::ignore && f == JointAngles::ignore)
      continue;

    if(t == JointAngles::ignore)
      t = target.angles[i];
    if(f == JointAngles::ignore)
      f = target.angles[i];

    if(t == JointAngles::off || t == JointAngles::ignore)
      t = theJointAngles.angles[i];
    if(f == JointAngles::off || f == JointAngles::ignore)
      f = theJointAngles.angles[i];

    target.angles[i] = ratio * (t - f) + f;
  }
}

void GetUpEngine::addBalance(JointRequest& jointRequest)
{
  if(theInertialData.gyro.y() != 0 && balance && motionID > -1)
  {
    float cycletime = theFrameInfo.cycleTime;
    float gyroDiff((theInertialData.gyro.y() - gLast) / cycletime);
    gLast = theInertialData.gyro.y();
    float calcVelocity(kp * gLast - kd * gyroDiff - ki * gBuffer);
    jointRequest.angles[Joints::rHipPitch] += calcVelocity * cycletime;
    jointRequest.angles[Joints::lHipPitch] += calcVelocity * cycletime;
    jointRequest.angles[Joints::lAnklePitch] += calcVelocity * cycletime;
    jointRequest.angles[Joints::rAnklePitch] += calcVelocity * cycletime;
    gBuffer += gLast;
  }
}

void GetUpEngine::pickMotion(GetUpEngineOutput& output)
{
  initVariables();
  const float& bodyAngleY = theInertialData.angle.y();
  const float& bodyAngleX = theInertialData.angle.x();
  switch(state)
  {
    case decideAction:
    {
      if((theFallDownState.direction == FallDownState::left || theFallDownState.direction == FallDownState::right) && theFrameInfo.getTimeSince(lastNotActiveTimeStamp) > 300)
      {
        //init recover motion
        state = recover;
        setStiffness(output, 70);
        setCurrentMotion(Mof::recoverFromSide);
      }
      else if(bodyAngleY > 65_deg && theFrameInfo.getTimeSince(lastNotActiveTimeStamp) > 300)
      {
        //init stand up front
        state = working;
        setStiffness(output, 90);
        setCurrentMotion(Mof::front);
      }
      //on back side experimental use of body angles
      else if(bodyAngleY < -65_deg && theFrameInfo.getTimeSince(lastNotActiveTimeStamp) > 300)
      {
        //init stand up back motion
        state = working;
        setStiffness(output, 90);
        setCurrentMotion(Mof::back);
      }
      //not fallen at all?
      else if(abs(bodyAngleY) < 30_deg && abs(bodyAngleX) < 50_deg && theFrameInfo.getTimeSince(lastNotActiveTimeStamp) > 500) //near upright
      {
        //init stand
        state = pickUp;
        setStiffness(output, 75);
        setCurrentMotion(Mof::stand);
      }
      else
      {
        //do nothing until the fall down state is decided
        state = decideAction;
        setCurrentMotion(Mof::numOfMotions);
      }
      break;
    }
    case breakUp:
    {
      if(theFrameInfo.getTimeSince(breakUpTimeStamp) < 2000)
      {
        // do nothing in order to wait if there is a distress (other robots etc.)
        state = breakUp;
        setCurrentMotion(Mof::numOfMotions);
      }
      else
      {
        //init recover motion
        state = recover;
        setStiffness(output, 90);
        setCurrentMotion(Mof::recovering);
      }
      break;
    }
    case schwalbe:
    {
      if(abs(theInertialData.angle.y()) > 30_deg) //not near upright
      {
        //do nothing then
        state = schwalbe;
        setCurrentMotion(Mof::numOfMotions);
        if(theFrameInfo.getTimeSince(soundTimeStamp) > 5000)
        {
          SystemCall::playSound("helpMe.wav");
          soundTimeStamp = theFrameInfo.time;
        }
      }
      else
      {
        //if near upright init stand
        state = pickUp;
        setStiffness(output, 75);
        setCurrentMotion(Mof::stand);
      }
      break;
    }
    default: //should never happen
    {
      setCurrentMotion(Mof::numOfMotions);
      state = decideAction;
      break;
    }
  }
}

void GetUpEngine::initVariables()
{
  //init global variables
  lineCounter = 0; //start at the beginning each time the engine gets active again after being not active
  lineStartTimeStamp = theFrameInfo.time; //save the start time
  startJoints = theJointAngles; //save the current joint angles
  gLast = 0.f; //reset
  gBuffer = 0.f;
  internalOdometryOffset = Pose2f();
  balance = false;
  //reset all last balancing angles
  lastUnbalanced.angles = theJointAngles.angles;
}

void GetUpEngine::checkCriticalParts(GetUpEngineOutput& output)
{
  if(mofs[motionID].lines[lineCounter].critical)
  {
    //if the body is not almost upright recover or cry for help
    if(abs(theInertialData.angle.y()) > 30_deg)
    {
      setStiffness(output, 0);
      if(output.tryCounter >= maxNumOfUnsuccessfulTries) //if no more trys left let behavior cry for help.
      {
        state = schwalbe;
      }
      else
      {
        if(SystemCall::getMode() != SystemCall::simulatedRobot)
          output.tryCounter++;

        state = breakUp;
        breakUpTimeStamp = theFrameInfo.time;
      }
    }
  }
}

void GetUpEngine::setNextJoints(GetUpEngineOutput& output)
{
  //do stuff only if we are not at the end of the movement
  if(lineCounter < maxCounter && motionID > -1)
  {
    float time = mofs[motionID].lines[lineCounter].duration;
    ASSERT(time > 0);
    ratio = (float)theFrameInfo.getTimeSince(lineStartTimeStamp) / time;
    //check if we are done yet with the current line
    if(ratio > 1.f)
    {
      //if yes update
      lineCounter++;
      lineStartTimeStamp = theFrameInfo.time;
      startJoints = lastUnbalanced;
      ratio = 0.f;
    }
    //are we still not at the end?
    if(lineCounter < maxCounter)
    {
      balance = (lineCounter >= mofs[motionID].balanceStartLine && mofs[motionID].balanceStartLine > -1);
      //set head joints
      for(int i = 0; i < 2; ++i)
        targetJoints.angles[i] = Angle::fromDegrees(mofs[motionID].lines[lineCounter].head[i]);
      //set arm joints
      for(int i = 0; i < 6; ++i)
      {
        targetJoints.angles[Joints::lShoulderPitch + i] = Angle::fromDegrees(mofs[motionID].lines[lineCounter].leftArm[i]);
        targetJoints.angles[Joints::rShoulderPitch + i] = Angle::fromDegrees(mofs[motionID].lines[lineCounter].rightArm[i]);
      }
      //set leg joints
      for(int i = 0; i < 6; ++i)
      {
        targetJoints.angles[Joints::lHipYawPitch + i] = Angle::fromDegrees(mofs[motionID].lines[lineCounter].leftLeg[i]);
        targetJoints.angles[Joints::rHipYawPitch + i] = Angle::fromDegrees(mofs[motionID].lines[lineCounter].rightLeg[i]);
      }
      checkCriticalParts(output);
    }
  }
  interpolate(startJoints, targetJoints, ratio, output);
  lastUnbalanced = output;
  addBalance(output);
}

void GetUpEngine::setStiffness(GetUpEngineOutput& output, int stiffness)
{
  //maybe we need different modes to save the ankles and the head
  for(int i = 0; i < Joints::numOfJoints; ++i)
    output.stiffnessData.stiffnesses[i] = stiffness;
}

void GetUpEngine::setCurrentMotion(Mof::Motion current)
{
  motionID = -1;
  for(int i = 0; i < Mof::numOfMotions; ++i)
    if(mofs[i].name == current)
    {
      motionID = i;
      break;
    }
  if(motionID < 0)
  {
    maxCounter = -1;
    internalOdometryOffset = Pose2f();
    return;
  }
  maxCounter = static_cast<int>(mofs[motionID].lines.size());
  internalOdometryOffset = mofs[motionID].odometryOffset;
}

MAKE_MODULE(GetUpEngine, motionControl)
