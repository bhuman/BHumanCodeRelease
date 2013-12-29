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
  output.odometryOffset = Pose2D();
  if(theMotionSelection.ratios[MotionRequest::getUp] > 0.f)
  {
    if(!wasActive)
    {
      wasActive = true;
      tryCounter = 0;
      output.isLeavingPossible = false; //no leaving anymore
      soundTimeStamp = theFrameInfo.time;
      state = decideAction;
      //save the joint angles from last frame
      output.jointHardness.resetToDefault();
      for(int i = 0; i < JointData::numOfJoints; ++i)
        output.angles[i] = theFilteredJointData.angles[i];
    }

    if(state == decideAction || state == breakUp || state == schwalbe)
      pickMotion(output); //this decides what is to do

    if(state == working) //only true if a stand up motion is initialized
    {
      setNextJoints(output);
      if(lineCounter >= maxCounter)
      {
        if(abs(theFilteredSensorData.data[SensorData::angleY]) > fromDegrees(30))
        {
          setHardness(output, 0);
          if(tryCounter >= maxNumOfUnsuccessfulTries)
            state = schwalbe;   //if no more trys left let behavior cry for help.
          else
          {
            tryCounter++;
            state = breakUp;
            breakUpTimeStamp = theFrameInfo.time;
          }
        }
        else //Success
        {
          if(theGroundContactState.contact)
          {
            if(!output.isLeavingPossible) //set odometry only once!
              output.odometryOffset = internalOdometryOffset;
            else
              output.odometryOffset = Pose2D();

            output.isLeavingPossible = true;
          }
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
    lastNotActiveTimeStamp = theFrameInfo.time;
  }
}

void GetUpEngine::interpolate(const JointData& from, const JointRequest& to, float& ratio, JointRequest& target)
{
  for(int i = 0; i < JointData::numOfJoints; ++i)
  {
    float f = from.angles[i];
    float t = to.angles[i];

    if(t == JointData::ignore && f == JointData::ignore)
      continue;

    if(t == JointData::ignore)
      t = target.angles[i];
    if(f == JointData::ignore)
      f = target.angles[i];

    if(t == JointData::off || t == JointData::ignore)
      t = theFilteredJointData.angles[i];
    if(f == JointData::off || f == JointData::ignore)
      f = theFilteredJointData.angles[i];

    target.angles[i] = ratio * (t - f) + f;
  }
}

void GetUpEngine::addBalance(JointRequest& jointRequest)
{
  if(theFilteredSensorData.data[SensorData::gyroY] != 0 && balance && motionID > -1)
  {
    float cycletime = theFrameInfo.cycleTime;
    float gyroDiff((theFilteredSensorData.data[SensorData::gyroY] - gLast) / cycletime);
    gLast = theFilteredSensorData.data[SensorData::gyroY];
    float calcVelocity(kp * gLast - kd * gyroDiff - ki * gBuffer);
    jointRequest.angles[JointData::RHipPitch] += calcVelocity * cycletime;
    jointRequest.angles[JointData::LHipPitch] += calcVelocity * cycletime;
    jointRequest.angles[JointData::LAnklePitch] +=  calcVelocity * cycletime;
    jointRequest.angles[JointData::RAnklePitch] += calcVelocity * cycletime;
    gBuffer += gLast;
  }
}

void GetUpEngine::pickMotion(GetUpEngineOutput& output)
{
  initVariables();
  switch(state)
  {
  case decideAction:
  {
    //on front side
    if(theFallDownState.direction == FallDownState::front &&
       (theFallDownState.state == FallDownState::onGround || theFallDownState.state == FallDownState::undefined) && theFrameInfo.getTimeSince(lastNotActiveTimeStamp) > 300)
    {
      //init stand up front
      state = working;
      setHardness(output, 90);
      setCurrentMotion(Mof::front);
    }
    //on back side
    else if(theFallDownState.direction == FallDownState::back &&
            (theFallDownState.state == FallDownState::onGround || theFallDownState.state == FallDownState::undefined) && theFrameInfo.getTimeSince(lastNotActiveTimeStamp) > 300)
    {
      //init stand up back motion
      state = working;
      setHardness(output, 90); //maybe 90 is enough?
      setCurrentMotion(Mof::back);
    }
    //not fallen at all?
    else if(abs(theFilteredSensorData.data[SensorData::angleY]) < fromDegrees(30) && theFrameInfo.getTimeSince(lastNotActiveTimeStamp) > 500) //near upright
    {
      //init stand
      state = pickUp;
      setHardness(output, 50);
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
      setHardness(output, 90);
      setCurrentMotion(Mof::recovering);
    }
    break;
  }
  case schwalbe:
  {
    if(abs(theFilteredSensorData.data[SensorData::angleY]) > fromDegrees(30)) //not near upright
    {
      //do nothing then
      state = schwalbe;
      setCurrentMotion(Mof::numOfMotions);
      if(theFrameInfo.getTimeSince(soundTimeStamp) > 10000)
      {
        SystemCall::playSound("helpMe.wav");
        soundTimeStamp = theFrameInfo.time;
      }
    }
    else
    {
      //if near upright init stand
      state = pickUp;
      setHardness(output, 50);
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
  startJoints = theFilteredJointData; //save the current joint angles
  gLast = 0.f; //reset
  gBuffer = 0.f;
  internalOdometryOffset = Pose2D();
  balance = false;
  //reset all last balancing angles
  for(int i = 0; i < 22; ++i)
    lastUnbalanced.angles[i] = theFilteredJointData.angles[i];
}

void GetUpEngine::checkCriticalParts(GetUpEngineOutput& output)
{
  if(mofs[motionID].lines[lineCounter].critical)
  {
    //if the body is not almost upright recover or cry for help
    if(abs(theFilteredSensorData.data[SensorData::angleY]) > fromDegrees(30))
    {
      setHardness(output, 0);
      if(tryCounter >= maxNumOfUnsuccessfulTries) //if no more trys left let behavior cry for help.
      {
        state = schwalbe;
      }
      else
      {
        tryCounter++;
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
        targetJoints.angles[i] = fromDegrees(mofs[motionID].lines[lineCounter].head[i]);
      //set arm joints
      for(int i = 0; i < 4; ++i)
      {
        targetJoints.angles[JointData::LShoulderPitch + i] = fromDegrees(mofs[motionID].lines[lineCounter].leftArm[i]);
        targetJoints.angles[JointData::RShoulderPitch + i] = fromDegrees(mofs[motionID].lines[lineCounter].rightArm[i]);
      }
      //set leg joints
      for(int i = 0; i < 6; ++i)
      {
        targetJoints.angles[JointData::LHipYawPitch + i] = fromDegrees(mofs[motionID].lines[lineCounter].leftLeg[i]);
        targetJoints.angles[JointData::RHipYawPitch + i] = fromDegrees(mofs[motionID].lines[lineCounter].rightLeg[i]);
      }
      checkCriticalParts(output);

    }
  }
  interpolate(startJoints, targetJoints, ratio, output);
  lastUnbalanced = output;
  addBalance(output);
}

void GetUpEngine::setHardness(GetUpEngineOutput& output, int hardness)
{
  //maybe we need different modes to save the ankles and the head
  for(int i = 0; i < JointData::numOfJoints; ++i)
    output.jointHardness.hardness[i] = hardness;
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
    internalOdometryOffset = Pose2D(0.f, 0.f, 0.f);
    return;
  }
  maxCounter = (int) mofs[motionID].lines.size();
  internalOdometryOffset = mofs[motionID].odometryOffset;
}

MAKE_MODULE(GetUpEngine, Motion Control)

