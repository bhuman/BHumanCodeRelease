/*
 * @file GetUpEngine.cpp
 * @author <A href="mailto:judy@tzi.de">Judith Müller</A>
 * @author Marvin Franke
 * @author Philip Reichenberg
 */

#include <cmath>
#include <functional>
#include <string>

#include "GetUpEngine.h"
#include "Platform/File.h"
#include "Platform/SystemCall.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Tools/Debugging/Modify.h"
#include "Tools/Motion/MotionUtilities.h"
#include "Tools/Debugging/DebugDrawings.h"

using namespace std;

void GetUpEngine::update(GetUpEngineOutput& output)
{
#ifndef NDEBUG
  GetUpMotion motionToMof = GetUpMotion::stand;
  MODIFY_ONCE("module:GetUpEngine:generateMofOf", motionToMof);
  if(motionToMof != GetUpMotion::stand)
    generateMofOfMotion(motionToMof);

  GetUpMotion mofToMotion = GetUpMotion::stand;
  MODIFY_ONCE("module:GetUpEngine:generateMotionFromDummyMof", mofToMotion);
  if(mofToMotion != GetUpMotion::stand)
    generateMotionOfMof(mofToMotion);

  GetUpMotion motionCmpBase = GetUpMotion::stand;
  GetUpMotion motionCmpOther = GetUpMotion::stand;
  MODIFY("module:GetUpEngine:compareMotionBase", motionCmpBase);
  MODIFY_ONCE("module:GetUpEngine:compareMotionOther", motionCmpOther);
  if(motionCmpOther != GetUpMotion::stand)
    compareMotions(motionCmpBase, motionCmpOther);
  MODIFY("module:GetUpEngine:comparison", comparison);
#endif

  output.failBack = false;
  output.failFront = false;
  output.odometryOffset = Pose2f();

  if(theLegMotionSelection.ratios[MotionRequest::getUp] > 0.f)
  {
    if(!wasActive)
    {
      wasActive = true;
      output.tryCounter = 0;
      output.isLeavingPossible = false; //no leaving anymore
      soundTimeStamp = theFrameInfo.time;
      state = decideAction;
      //start with mirrored motion if only the mirrored stand up is possible
      mirror = theDamageConfigurationBody.brokenStandUp == DamageConfigurationBody::onlyMirrored;
      //save the joint angles from last frame
      //    output.stiffnessData.resetToDefault();
      for(int i = 0; i < Joints::numOfJoints; ++i)
      {
        output.angles[i] = theJointAngles.angles[i];
        output.stiffnessData.stiffnesses[i] = 50; //some crappy default
        targetJoints.angles[i] = theJointAngles.angles[i];
      }
      if(theDamageConfigurationBody.brokenStandUp == DamageConfigurationBody::allBroken)
        state = schwalbe;
    }

    if(state == decideAction || state == breakUp || state == schwalbe)
      pickMotion(output); //this decides what is to do

    if(state == working)
    {
      setNextJoints(output);
      if(lineCounter >= maxCounter)
      {
        if(verifyUprightTorso(output))
        {
          if(!output.isLeavingPossible) //set odometry only once!
            if(mirror)
              output.odometryOffset = Pose2f(-internalOdometryOffset.rotation, internalOdometryOffset.translation.x(), -internalOdometryOffset.translation.y());
            else
              output.odometryOffset = internalOdometryOffset;
          else
            output.odometryOffset = Pose2f();

          if(mofs[motionID].continueTo != motionID)
          {
            initVariables();
            output.criticalTriggered = false;
            state = working;
            setCurrentMotion(mofs[motionID].continueTo);
            setBaseLimbStiffness();
          }
          else
            output.isLeavingPossible = true;
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
    lastMotionInfo = theMotionInfo; // store the last MotionInfo
  }
  output.name = motionID;
  output.lineCounter = lineCounter;
}

void GetUpEngine::update(GetUpEngineOutputLog& output)
{
  output.tryCounter = theGetUpEngineOutput.tryCounter;
  output.lineCounter = theGetUpEngineOutput.lineCounter;
  output.name = theGetUpEngineOutput.name;
  output.criticalTriggered = theGetUpEngineOutput.criticalTriggered;
  output.optionalLine = theGetUpEngineOutput.optionalLine;
  output.theBalanceFloatY = theGetUpEngineOutput.theBalanceFloatY;
  output.theBalanceFloatX = theGetUpEngineOutput.theBalanceFloatX;
}

float GetUpEngine::addBalanceY(JointRequest& jointRequest)
{
  if(lineCounter < maxCounter && theInertialData.gyro.y() != 0 && mofs[motionID].lines[lineCounter].balanceWithHipAndAnkleY && std::abs(theInertialData.angle.y()) < 45_deg)
  {
    //the more the robot is tilted, the stronger we want to balance
    float ratio = std::min(std::abs(theInertialData.gyro.y()) / 10.f, 1.f);
    constexpr float cycletime = Constants::motionCycleTime;
    float gyroDiff((theInertialData.gyro.y() - gLastY) / cycletime);
    gLastY = theInertialData.gyro.y();
    //calculation with a pid controller
    float calcVelocity(kp * gLastY - kd * gyroDiff - ki * gBufferY);
    float balanceFloat = calcVelocity * cycletime * ratio;
    balanceFloat = balanceFloat * 10.f;
    //Increase balance factor if conditions are right
    if(mofs[motionID].lines[lineCounter].increaseBalanceY)
    {
      if((theInertialData.angle.y().toDegrees() - balanceHipAngleY) > 0.f && theInertialData.gyro.y() > 0.f)
      {
        balanceFloat = balanceFloat * (1.f + (theInertialData.angle.y().toDegrees() - balanceHipAngleY) / 3.f);
        if(balanceFloat < 0.f)
        {
          balanceFloat = std::fabs(balanceFloat);
        }
      }
    }
    //cap the balance value to prevent too high value
    if(balanceFloat < 0.f)
    {
      balanceFloat = std::max(balanceFloat, -0.1047f);
    }
    else
    {
      balanceFloat = std::min(balanceFloat, 0.1047f);
    }
    jointRequest.angles[Joints::rHipPitch] += balanceFloat;
    jointRequest.angles[Joints::lHipPitch] += balanceFloat;
    jointRequest.angles[Joints::lAnklePitch] += balanceFloat;
    jointRequest.angles[Joints::rAnklePitch] += balanceFloat;
    gBufferY += gLastY;

    float returnFloat = 0.f;
    if(balanceFloat != 0)
    {
      returnFloat = (float)(balanceFloat / 3.141) * 180.f;
    }
    return returnFloat;
  }
  //reset the values while the balancer is off
  else
  {
    if(currentBalanceLine < lineCounter)
    {
      currentBalanceLine = lineCounter;
      balanceHipAngleY = theInertialData.angle.y().toDegrees();
    }
    gLastY = 0.f;
    gBufferY = 0.f;
    return 0.f;
  }
}

float GetUpEngine::addBalanceX(JointRequest& jointRequest)
{
  if(lineCounter < maxCounter && theInertialData.gyro.x() != 0 && mofs[motionID].lines[lineCounter].balanceWithHipAndAnkleX && std::abs(theInertialData.angle.x()) < 45_deg)
  {
    float ratio = std::min(std::abs(theInertialData.angle.x()) / 5.f, 1.f);
    constexpr float cycletime = Constants::motionCycleTime;
    float gyroDiff((theInertialData.gyro.x() - gLastX) / cycletime);
    gLastX = theInertialData.gyro.x();

    //calculation with a pid controller
    float calcVelocity(kp * gLastX - kd * gyroDiff - ki * gBufferX);
    float balanceFloat = calcVelocity * cycletime * ratio;
    //cap the balance value to prevent too high value
    if(balanceFloat < 0)
    {
      balanceFloat = std::max(balanceFloat, -0.05f);
    }
    else
    {
      balanceFloat = std::min(balanceFloat, 0.05f);
    }
    jointRequest.angles[Joints::rHipRoll] += balanceFloat;
    jointRequest.angles[Joints::lHipRoll] -= balanceFloat;
    jointRequest.angles[Joints::lAnkleRoll] += balanceFloat;
    jointRequest.angles[Joints::rAnkleRoll] -= balanceFloat;
    gBufferX += gLastX;

    float returnFloat = 0.f;
    if(balanceFloat != 0)
    {
      returnFloat = (float)(balanceFloat / 3.141) * 180.f;
    }
    return returnFloat;
  }
  //reset the values while the balancer is off
  else
  {
    gLastX = 0.f;
    gBufferX = 0.f;
    return 0.f;
  }
}

void GetUpEngine::pickMotion(GetUpEngineOutput& output)
{
  initVariables();
  output.criticalTriggered = false;
  const float& bodyAngleY = theInertialData.angle.y();
  const float& bodyAngleX = theInertialData.angle.x();
  bool leftArmOverHead = theJointAngles.angles[Joints::lShoulderPitch] < -40_deg;
  bool rightArmOverHead = theJointAngles.angles[Joints::rShoulderPitch] < -40_deg;

  switch(state)
  {
    case decideAction:
    {
      //special handling for sitDown positions
      if(lastMotionInfo.motion == MotionRequest::specialAction
              && lastMotionInfo.specialActionRequest.specialAction == SpecialActionRequest::sitDown)
      {
        state = working;
        setCurrentMotion(GetUpMotion::fromSitDown);
        setBaseLimbStiffness();
        lastMotionInfo.motion = MotionRequest::getUp; //set that we started a getUp motion in order to use normal break up motions
      }
      else if(bodyAngleY > 65_deg && theFrameInfo.getTimeSince(lastNotActiveTimeStamp) > 300)
      {
        //init stand up front
        if(output.tryCounter >= static_cast<int>(theDamageConfigurationBody.getUpFront.size()))
          state = schwalbe;
        else
        {
          state = working;
          setCurrentMotion(theDamageConfigurationBody.getUpFront[output.tryCounter]);
          setBaseLimbStiffness();
        }
      }
      //on back side
      else if(bodyAngleY < -65_deg && theFrameInfo.getTimeSince(lastNotActiveTimeStamp) > 300)
      {
        //init stand up back motion
        state = working;
        if(output.tryCounter >= static_cast<int>(theDamageConfigurationBody.getUpBack.size()))
          state = schwalbe;
        else
          setCurrentMotion(theDamageConfigurationBody.getUpBack[output.tryCounter]);
        setBaseLimbStiffness();
      }
      else if(std::abs(bodyAngleX) > 50_deg && theFrameInfo.getTimeSince(lastNotActiveTimeStamp) > 300)
      {
        //init recover motion
        if((rightArmOverHead || leftArmOverHead) && theFrameInfo.getTimeSince(recoverTimeStamp) > 3000)  // maybe check for special actions?
        {
          state = recover;
          recoverTimeStamp = theFrameInfo.time;
          setCurrentMotion(GetUpMotion::recoverFromSideAfterJump);
        }
        else if(theFrameInfo.getTimeSince(recoverTimeStamp) > 3000)
        {
          state = recover;
          recoverTimeStamp = theFrameInfo.time;
          setCurrentMotion(GetUpMotion::recoverFromSide);
        }
        // sometimes after a recover motion the robot doesn't recognize that he turned to his back or front
        else if(bodyAngleY < 0)
        {
          state = working;
          setCurrentMotion(theDamageConfigurationBody.getUpBack[output.tryCounter]);
        }
        else
        {
          state = working;
          setCurrentMotion(theDamageConfigurationBody.getUpFront[output.tryCounter]);
        }
        setBaseLimbStiffness();
      }
      //not fallen at all?
      else if(std::abs(bodyAngleY) < 35_deg && std::abs(bodyAngleX) < 50_deg && theFrameInfo.getTimeSince(lastNotActiveTimeStamp) > 500) //near upright
      {
        //init stand
        state = pickUp;
        setCurrentMotion(GetUpMotion::stand);
        setBaseLimbStiffness();
      }
      else
      {
        //do nothing until the fall down state is decided
        state = decideAction;
        setCurrentMotion(GetUpMotions::numOfGetUpMotions);
      }
      break;
    }
    case breakUp:
    {
      if(bodyAngleY > 0 && (theJointAngles.angles[Joints::lElbowRoll] < -40_deg || theJointAngles.angles[Joints::rElbowRoll] > 40_deg))
      {
        state = recover;
        recoverTimeStamp = theFrameInfo.time;
        setCurrentMotion(GetUpMotion::recoverBadArms);
        setBaseLimbStiffness();
        lastMotionInfo.motion = MotionRequest::getUp;
      }

      else if((theFrameInfo.getTimeSince(breakUpTimeStamp) < 500 && output.tryCounter == 1) || theFrameInfo.getTimeSince(breakUpTimeStamp) < 1000)
      {
        // do nothing in order to wait if there is a distress (other robots etc.)
        setCurrentMotion(GetUpMotions::numOfGetUpMotions);
      }
      else
      {
        //init recover motion
        state = recover;
        if(std::abs(bodyAngleY) < 55_deg && std::abs(bodyAngleX) < 35_deg) //TODO check if it is possible to interpolate to sitDown position
        {
          setCurrentMotion(GetUpMotion::recoverAfterBadBreakUp);
          state = working;
          output.tryCounter -= 1;
        }
        else if(output.tryCounter <= 1)
          setCurrentMotion(GetUpMotion::recoverFast);
        else
          setCurrentMotion(GetUpMotion::recoverAndWait);
        setBaseLimbStiffness();
      }
      break;
    }
    case schwalbe:
    {
      if(std::abs(theInertialData.angle.y()) > 35_deg) //not near upright
      {
        //do nothing then
        state = schwalbe;
        setCurrentMotion(GetUpMotions::numOfGetUpMotions);
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
        setCurrentMotion(GetUpMotion::stand);
        setBaseLimbStiffness();
      }
      break;
    }
    default: //should never happen
    {
      setCurrentMotion(GetUpMotions::numOfGetUpMotions);
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
  internalOdometryOffset = Pose2f();
  balance = false;
  balanceArm = false;
  balanceArmAndLeg = false;
  isInOptionalLine = false;
  //reset all last balancing angles
  lastUnbalanced.angles = theJointAngles.angles;
  upperArmBorder = 10.f;
  upperLegBorder = 2.f;
  lowerArmBorder = -10.f;
  lowerLegBorder = -2.f;
  gLastY = 0.f;
  gBufferY = 0.f;
  gLastX = 0.f;
  gBufferX = 0.f;
}

void GetUpEngine::setNextJoints(GetUpEngineOutput& output)
{
  //do stuff only if we are not at the end of the movement
  if(lineCounter < maxCounter)
  {
    const float& time = mofs[motionID].lines[lineCounter].duration;
    ASSERT(time > 0);
    ratio = static_cast<float>(theFrameInfo.getTimeSince(lineStartTimeStamp) / time);

    if(ratio > 1.f && checkConditions(mofs[motionID].lines[lineCounter].waitConditions, mofs[motionID].lines[lineCounter].waitConditionAnds))
      ratio = static_cast<float>(theFrameInfo.getTimeSince(lineStartTimeStamp) / (time + mofs[motionID].lines[lineCounter].waitDuration));
    //check if we are done yet with the current line
    if(ratio > 1.f)
    {
      //Calculate the difference of the goal angles and the reached angles
      for(int i = 0; i < 6; ++i)
      {
        float leftArmLast = mofs[motionID].lines[lineCounter].leftArm[i];
        float rightArmLast = mofs[motionID].lines[lineCounter].rightArm[i];
        float leftLegLast = mofs[motionID].lines[lineCounter].leftLeg[i];
        float rightLegLast = mofs[motionID].lines[lineCounter].rightLeg[i];

        float leftArmDif = leftArmLast - theJointAngles.angles[Joints::lShoulderPitch + i].toDegrees();
        float rightArmDif = rightArmLast - theJointAngles.angles[Joints::rShoulderPitch + i].toDegrees();
        float leftLegDif = leftLegLast - theJointAngles.angles[Joints::lHipYawPitch + i].toDegrees();
        float rightLegDif = rightLegLast - theJointAngles.angles[Joints::rHipYawPitch + i].toDegrees();

        lArmDif[i] = leftArmDif;
        rArmDif[i] = rightArmDif;
        lLegDif[i] = leftLegDif;
        rLegDif[i] = rightLegDif;

        if(i == 5)
        {
          lArmDif[i] = 0.f;
          rArmDif[i] = 0.f;
        }
      }

      lineStartTimeStamp = theFrameInfo.time;
      startJoints = lastUnbalanced;
      ratio = 0.f;
      //update stiffness
      ++lineCounter;

      //skip motionLines if there is a condition which is not met.
      skipForOptionalLine();

      output.optionalLine = isInOptionalLine;
      if(lineCounter < maxCounter)
      {
        for(unsigned i = 0; i < mofs[motionID].lines[lineCounter].singleMotorStiffnessChange.size(); ++i)
          targetJoints.stiffnessData.stiffnesses[mofs[motionID].lines[lineCounter].singleMotorStiffnessChange[i].joint] = mofs[motionID].lines[lineCounter].singleMotorStiffnessChange[i].s;
      }
    }
    //are we still not at the end?
    if(lineCounter < maxCounter)
    {
      //Check if a balancer needs to be activated
      if(!balanceArm && lineCounter >= mofs[motionID].balanceArmStartLine && mofs[motionID].balanceArmStartLine > -1 && !balance)
      {
        theBalancer.init(mirror, balancingParamsForArms);
        balanceArm = true;
      }
      if(!balanceArmAndLeg && lineCounter >= mofs[motionID].balanceArmAndLegStartLine && mofs[motionID].balanceArmAndLegStartLine > -1 && !balance)
      {
        theBalancer.init(mirror, balancingParamsForArms);
        balanceArmAndLeg = true;
      }
      if(!balance && lineCounter >= mofs[motionID].balanceStartLine && mofs[motionID].balanceStartLine > -1)
      {
        theBalancer.init(mirror, balancingParams);
        balance = true;
      }
      //set head joints
      for(int i = 0; i < 2; ++i)
        targetJoints.angles[i] = convertToAngleSpecialCases(mofs[motionID].lines[lineCounter].head[i]);
      //set arm and leg joints
      //the calculated difference values are added on top
      for(int i = 0; i < 6; ++i)
      {
        //these values are needed for the correctAngleMotion method to determin, if we should add a value on top on the requested joints.
        float leftArmNext = mofs[motionID].lines[lineCounter].leftArm[i];
        float rightArmNext = mofs[motionID].lines[lineCounter].rightArm[i];
        float leftLegNext = mofs[motionID].lines[lineCounter].leftLeg[i];
        float rightLegNext = mofs[motionID].lines[lineCounter].rightLeg[i];

        int lineCounterDif = lineCounter - 1;
        if(lineCounterDif < 0 || lineCounterDif == maxCounter)
        {
          lineCounterDif = lineCounter;
        }

        float leftArmLast = mofs[motionID].lines[lineCounterDif].leftArm[i];
        float rightArmLast = mofs[motionID].lines[lineCounterDif].rightArm[i];
        float leftLegLast = mofs[motionID].lines[lineCounterDif].leftLeg[i];
        float rightLegLast = mofs[motionID].lines[lineCounterDif].rightLeg[i];

        float leftArmDif = lArmDif[i];
        float rightArmDif = rArmDif[i];
        float leftLegDif = lLegDif[i];
        float rightLegDif = rLegDif[i];

        //Add the full difference on top if we are at the start of the motionline.
        //Add a fraction when we are already doing the motionLine.
        if(ratio <= 1.f)
        {
          leftArmDif = leftArmDif * (1.f - ratio);
          rightArmDif = rightArmDif * (1.f - ratio);
          leftLegDif = leftLegDif * (1.f - ratio);
          rightLegDif = rightLegDif * (1.f - ratio);
        }
        else
        {
          leftArmDif = 0.f;
          rightArmDif = 0.f;
          leftLegDif = 0.f;
          rightLegDif = 0.f;
        }

        //correct the joint values based on the last reached joint values and then convert them into Angles.
        targetJoints.angles[Joints::lShoulderPitch + i] = convertToAngleSpecialCases(correctAngleMotion(leftArmNext, leftArmLast, leftArmDif, true));
        targetJoints.angles[Joints::rShoulderPitch + i] = convertToAngleSpecialCases(correctAngleMotion(rightArmNext, rightArmLast, rightArmDif, true));
        targetJoints.angles[Joints::lHipYawPitch + i] = convertToAngleSpecialCases(correctAngleMotion(leftLegNext, leftLegLast, leftLegDif, false));
        targetJoints.angles[Joints::rHipYawPitch + i] = convertToAngleSpecialCases(correctAngleMotion(rightLegNext, rightLegLast, rightLegDif, false));
      }
      //mirror joints if necessary
      if(mirror)
      {
        JointRequest mirroredJoints;
        mirroredJoints.mirror(targetJoints);
        targetJoints = mirroredJoints;
      }
      bool criticalIsFalse = false;
      if(!mofs[motionID].lines[lineCounter].critical)
      {
        criticalIsFalse = true;
      }

      //check if are in a critical line
      if(criticalIsFalse || (mofs[motionID].lines[lineCounter].critical && verifyUprightTorso(output)))
      {
        //Interpolate the requested joint values
        MotionUtilities::interpolate(startJoints, targetJoints, ratio, output, theJointAngles);
        lastUnbalanced = output;

        //check if the balancers are active
        if(balanceArm && !balance)
        {
          JointRequest preBalance = output;
          theBalancer.balanceJointRequest(output, balancingParams);
          for(int i = 0; i < 6; ++i)
          {
            output.angles[Joints::lHipYawPitch + i] = preBalance.angles[Joints::lHipYawPitch + i];
            output.angles[Joints::rHipYawPitch + i] = preBalance.angles[Joints::rHipYawPitch + i];
          }
        }
        else if(balanceArmAndLeg && !balance)
        {
          JointRequest preBalance = output;
          theBalancer.balanceJointRequest(output, balancingParams);
          if(std::abs(theInertialData.angle.y()) > 35_deg)
          {
            for(int i = 0; i < 6; ++i)
            {
              output.angles[Joints::lHipYawPitch + i] = preBalance.angles[Joints::lHipYawPitch + i];
              output.angles[Joints::rHipYawPitch + i] = preBalance.angles[Joints::rHipYawPitch + i];
            }
          }
        }
        else if(balance)
          theBalancer.balanceJointRequest(output, balancingParams);

        float balanceFloat = addBalanceY(output);
        output.theBalanceFloatY = balanceFloat;

        balanceFloat = addBalanceX(output);
        output.theBalanceFloatX = balanceFloat;

        //The balancer is not allowed to move the arms to much up.
        if(output.angles[Joints::lShoulderRoll] > 20_deg)
        {
          output.angles[Joints::lShoulderRoll] = lastUnbalanced.angles[Joints::lShoulderRoll];
        }
        if(output.angles[Joints::rShoulderRoll] < -20_deg)
        {
          output.angles[Joints::rShoulderRoll] = lastUnbalanced.angles[Joints::rShoulderRoll];
        }
      }

      //we are in a critical Line and the torso is too tilted. Set the arm joints so we dont fall on our arms.
      else
      {
        output.angles[Joints::lShoulderPitch] = 20_deg;
        output.angles[Joints::lShoulderRoll] = 30_deg;
        output.angles[Joints::lElbowYaw] = -116_deg;
        output.angles[Joints::lElbowRoll] = -60;

        output.angles[Joints::rShoulderPitch] = 20_deg;
        output.angles[Joints::rShoulderRoll] = -30_deg;
        output.angles[Joints::rElbowYaw] = 116_deg;
        output.angles[Joints::rElbowRoll] = 60;

        if(theInertialData.angle.y() > 0_deg)
        {
          output.angles[Joints::headPitch] = -35_deg;
        }
        else
        {
          output.angles[Joints::headPitch] = 35_deg;
        }
      }
    }
  }
}

void GetUpEngine::setStiffnessAllJoints(GetUpEngineOutput& output, int stiffness)
{
  //maybe we need different modes to save the ankles and the head
  for(int i = 0; i < Joints::numOfJoints; ++i)
  {
    targetJoints.stiffnessData.stiffnesses[i] = stiffness;
    output.stiffnessData.stiffnesses[i] = stiffness;
  }
}

void GetUpEngine::setBaseLimbStiffness()
{
  for(int i = 0; i < 2; ++i)
    targetJoints.stiffnessData.stiffnesses[i] = mofs[motionID].baseLimbStiffness[0];
  //set arm joints
  for(int i = 0; i < 6; ++i)
  {
    targetJoints.stiffnessData.stiffnesses[Joints::lShoulderPitch + i] = mofs[motionID].baseLimbStiffness[1];
    targetJoints.stiffnessData.stiffnesses[Joints::rShoulderPitch + i] = mofs[motionID].baseLimbStiffness[2];
    targetJoints.stiffnessData.stiffnesses[Joints::lHipYawPitch + i] = mofs[motionID].baseLimbStiffness[3];
    targetJoints.stiffnessData.stiffnesses[Joints::rHipYawPitch + i] = mofs[motionID].baseLimbStiffness[4];
  }
}

void GetUpEngine::setCurrentMotion(GetUpMotion current)
{
  motionID = current;
  maxCounter = static_cast<int>(mofs[motionID].lines.size());
  internalOdometryOffset = mofs[motionID].odometryOffset;
}

bool GetUpEngine::verifyUprightTorso(GetUpEngineOutput& output)
{
  //if the body is not almost upright recover or cry for help
  if(std::abs(theInertialData.angle.y()) > 35_deg)
  {
    setStiffnessAllJoints(output, 10);
    if(SystemCall::getMode() != SystemCall::simulatedRobot)
      output.tryCounter++;

    if(theDamageConfigurationBody.brokenStandUp == DamageConfigurationBody::allFine)
      mirror = !mirror; //try it mirrored then

    if(theInertialData.angle.y() < 0)
      output.failBack = true;
    else
      output.failFront = true;

    state = breakUp;
    breakUpTimeStamp = theFrameInfo.time;
    output.criticalTriggered = true;
    return false;
  }

  return true;
}

void GetUpEngine::generateMofOfMotion(GetUpMotion motionName)
{
  char name[512];
  sprintf(name, "%s/Config/mof/getUpEngineDummy.mof", File::getBHDir());
  OutTextRawFile f(name, false);
  f << "\" IF YOU HAVE CHANGED THIS FILE DO NOT COMMIT IT!" << endl;
  f << "motion_id = getUpEngineDummy" << endl;
  f << "label start" << endl;

  // keep trac of stiffness changes, init with base stiffness
  int internalStiffness[Joints::numOfJoints];

  for(int i = 0; i < 2; ++i)
    internalStiffness[i] = mofs[motionName].baseLimbStiffness[0];
  //set arm joints
  for(int i = 0; i < 6; ++i)
  {
    internalStiffness[Joints::lShoulderPitch + i] = mofs[motionName].baseLimbStiffness[1];
    internalStiffness[Joints::rShoulderPitch + i] = mofs[motionName].baseLimbStiffness[2];
    internalStiffness[Joints::lHipYawPitch + i] = mofs[motionName].baseLimbStiffness[3];
    internalStiffness[Joints::rHipYawPitch + i] = mofs[motionName].baseLimbStiffness[4];
  }

  auto putStiffness = [&](OutTextRawFile& f)
  {
    f << endl << "stiffness";
    for(int i = 0; i < Joints::numOfJoints; ++i)
      f << " " << internalStiffness[i];
    f << " 0" << endl << endl;
  };

  putStiffness(f);

  auto putAngle = [&](OutTextRawFile& f, float angle)
  {
    if(angle == JointAngles::off)
      f << "- ";
    else if(angle == JointAngles::ignore)
      f << "* ";
    else
    {
      char formatted[10];
      sprintf(formatted, "%.2f ", angle);
      f << formatted;
    }
  };

  //add lines now
  int lastLine = (int) mofs[motionName].lines.size() - 1;
  bool isOptionalLine = false;
  for(int i = 0; i < (int) mofs[motionName].lines.size(); i++)
  {
    if(!mofs[motionName].lines[i].singleMotorStiffnessChange.empty())
    {
      for(unsigned j = 0; j < mofs[motionName].lines[i].singleMotorStiffnessChange.size(); ++j)
        internalStiffness[mofs[motionName].lines[i].singleMotorStiffnessChange[j].joint] = mofs[motionName].lines[i].singleMotorStiffnessChange[j].s;

      putStiffness(f);
    }

    if(isOptionalLine)
    {
      if(!mofs[motionName].lines[i].isPartOfPreviousOptionalLine)
      {
        isOptionalLine = false;
        f << endl << "\"@endOptionalLine" << endl << endl;
      }
    }
    if(mofs[motionName].lines[i].conditions.size() > 0)  // convert optionalLine and add it before the start line
    {
      isOptionalLine = true;
      f << endl << "\"@optionalLine ";

      f << (mofs[motionName].lines[i].isElseBlock ? "ElseBlock " : "IfBlock ");

      f << (mofs[motionName].lines[i].optionalLineAnds ? "And " : "Or ");

      for(int indexCondition = 0; indexCondition < (int)mofs[motionName].lines[i].conditions.size(); ++indexCondition)
      {
        if(mofs[motionName].lines[i].conditions[indexCondition].isNot)
        {
          f << "Not ";
        }
        else
        {
          f << "Is ";
        }

        f << TypeRegistry::getEnumName(mofs[motionName].lines[i].conditions[indexCondition].variable) << " ";

        f << mofs[motionName].lines[i].conditions[indexCondition].lowerFloat << " ";
        f << mofs[motionName].lines[i].conditions[indexCondition].higherFloat << " ";
      }

      f << "End " << endl << endl;
    }
    if(mofs[motionName].lines[i].critical)
      f << endl << "\"@critical (upright check at start of interpolation to next line)" << endl << endl;
    if(mofs[motionName].lines[i].balanceWithHipAndAnkleY)
    {
      f << endl << "\"@balanceWithHipAndAnkleY (balance with ankle and hip)" << endl << endl;
      if(mofs[motionName].lines[i].increaseBalanceY)
        f << endl << "\"@increaseBalanceY (increase balanceWithHipAndAnkleY)" << endl << endl;
    }
    if(mofs[motionName].lines[i].balanceWithHipAndAnkleX)
      f << endl << "\"@balanceWithHipAndAnkleX (balance with ankle and hip)" << endl << endl;
    if(mofs[motionName].lines[i].increaseBalanceX)
      f << endl << "\"@increaseBalanceX (increase balanceWithHipAndAnkleX)" << endl << endl;
    if(i == mofs[motionName].balanceStartLine - 1) //add it before the start line
      f << endl << "\"@balanceStartLine (turn on balance at start of interpolation to next line)" << endl << endl;
    if(i == mofs[motionName].balanceArmStartLine - 1) //add it before the start line
      f << endl << "\"@balanceArmStartLine (turn on balance for only Arms at start of interpolation to next line)" << endl << endl;
    if(i == mofs[motionName].balanceArmAndLegStartLine - 1) //add it before the start line
      f << endl << "\"@balanceArmAndLegStartLine (turn on balance for Legs if Torso < 35_deg and for Arms at start of interpolation to next line)" << endl << endl;
    //add label repeat before last line
    if(i == lastLine)
      f << "label repeat" << endl;

    for(float headAngle : mofs[motionName].lines[i].head)
      putAngle(f, headAngle);
    for(float leftArmAngle : mofs[motionName].lines[i].leftArm)
      putAngle(f, leftArmAngle);
    for(float rightArmAngle : mofs[motionName].lines[i].rightArm)
      putAngle(f, rightArmAngle);
    for(float leftLegAngle : mofs[motionName].lines[i].leftLeg)
      putAngle(f, leftLegAngle);
    for(float rightLegAngle : mofs[motionName].lines[i].rightLeg)
      putAngle(f, rightLegAngle);
    f << 1 << " " << mofs[motionName].lines[i].duration << endl;
  }
  //add odometry offset as comment
  char odometry[100];
  sprintf(odometry, "\n\"@odometryOffset %.2f %.2f %.2f\n", mofs[motionName].odometryOffset.translation.x(), mofs[motionName].odometryOffset.translation.y(), mofs[motionName].odometryOffset.rotation.toDegrees());
  f << odometry << endl
    << "\"@continueTo " << TypeRegistry::getEnumName(mofs[motionName].continueTo) << endl << endl
    << "transition getUpEngineDummy getUpEngineDummy repeat" << endl
    << "transition allMotions extern start" << endl;
}

void GetUpEngine::generateMotionOfMof(GetUpMotion motionName)
{
  char name[512];
  sprintf(name, "%s/Config/mof/getUpEngineDummy.mof", File::getBHDir());
  FILE* f = fopen(name, "r");
  if(!f)
    OUTPUT_ERROR("GetUpEngineDummy.mof was not found");
  else
  {
    bool foundBaseStiffness(false);

    Mof newMotion;
    newMotion.balanceStartLine = -1;
    newMotion.balanceArmStartLine = -1;
    newMotion.balanceArmAndLegStartLine = -1;
    newMotion.continueTo = motionName;
    int headStart(0),
        lArmStart(headStart + 2),
        rArmStart(lArmStart + 6),
        lLegStart(rArmStart + 6),
        rLegStart(lLegStart + 6);

    bool partOfPreviousOptionalLine = false;
    bool partOfPreviousOptionalLineHelp = false;//This line is true AFTER the line that started a
    //optionalLine and if it is part of the optionalLine Block
    bool optionalLineAndsVar = false;
    bool isElse = false;
    std::vector<Condition> condVec;

    float maxWaitingDuration = 0.f;
    bool dynamicWaitAnds = false;
    std::vector<Condition> waitConditions;

    std::vector<StiffnessPair> singleMotorStiffnessChange;
    bool nextLineCritical(false);

    bool isBalanceWithHipAndAnkleY = false;
    bool isBalanceWithHipAndAnkleX = false;
    bool isIncreaseBalanceY = false;
    bool isIncreaseBalanceX = false;

    auto getBaseLimb = [&](int k)
    {
      return (k < lArmStart) ? 0 : ((k < rArmStart) ? 1 : ((k < lLegStart) ? 2 : ((k < rLegStart) ? 3 : 4)));
    };

    auto getJoint = [&](int k, MofLine& motion_line)
    {
      return (k < lArmStart) ? &motion_line.head[k] : ((k < rArmStart) ? &motion_line.leftArm[k - lArmStart] : ((k < lLegStart) ? &motion_line.rightArm[k - rArmStart] : ((k < rLegStart) ? &motion_line.leftLeg[k - lLegStart] : &motion_line.rightLeg[k - rLegStart])));
    };

    auto parseConditions = [&](char sval[Joints::numOfJoints + 3][256], vector<Condition>& target)
    {
      for(int iddd = 3; iddd <= 25; iddd += 4)
      {
        if(!strcmp(sval[iddd], "End"))
          break;

        Condition con;
        con.isNot = !strcmp(sval[iddd], "Not");
        con.variable = static_cast<ConditionVar>(TypeRegistry::getEnumValue(typeid(ConditionVar).name(), sval[iddd + 1]));
        con.lowerFloat = static_cast<float>(atof(sval[iddd + 2]));
        con.higherFloat = static_cast<float>(atof(sval[iddd + 3]));

        target.push_back(con);
      }
    };

    char s[128000];
    size_t siz = fread(s, 1, 128000, f);
    fclose(f);
    if(siz > 0)
    {
      s[siz] = 0;
      char* t = &s[0];
      int line = 1;
      while(*t)
      {
        char* u = strchr(t, '\n');
        if(u >= t)
        {
          *u = 0;
          char sval[Joints::numOfJoints + 3][256]; // joints + interpolate + duration + bad argument
          int c = sscanf(t, "%s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s", // (numOfJoints + 3) * %s
                         sval[0], sval[1], sval[2], sval[3], sval[4], sval[5], sval[6], sval[7],
                         sval[8], sval[9], sval[10], sval[11], sval[12], sval[13], sval[14], sval[15],
                         sval[16], sval[17], sval[18], sval[19], sval[20], sval[21], sval[22], sval[23],
                         sval[24], sval[25], sval[26], sval[27], sval[28]); // sval[0]..sval[numOfJoints + 2]

          if(!strcmp(sval[0], "\"@critical"))  //annotation of critical line as comment (since specialAction does not support it)
            nextLineCritical = true;
          if(!strcmp(sval[0], "\"@balanceWithHipAndAnkleY"))  //annotation of critical line as comment (since specialAction does not support it)
          {
            isBalanceWithHipAndAnkleY = true;
          }
          if(!strcmp(sval[0], "\"@balanceWithHipAndAnkleX"))  //annotation of critical line as comment (since specialAction does not support it)
          {
            isBalanceWithHipAndAnkleX = true;
          }
          if(!strcmp(sval[0], "\"@increaseBalanceY"))  //annotation of critical line as comment (since specialAction does not support it)
          {
            isIncreaseBalanceY = true;
          }
          if(!strcmp(sval[0], "\"@increaseBalanceX"))  //annotation of critical line as comment (since specialAction does not support it)
          {
            isIncreaseBalanceX = true;
          }
          else if(!strcmp(sval[0], "\"@balanceStartLine"))  //annotation of balanceStartLine in comment
            newMotion.balanceStartLine = (int)newMotion.lines.size() + 1; //start with next line
          else if(!strcmp(sval[0], "\"@balanceArmStartLine"))  //annotation of balanceArmStartLine in comment
            newMotion.balanceArmStartLine = (int)newMotion.lines.size() + 1; //start with next line
          else if(!strcmp(sval[0], "\"@balanceArmAndLegStartLine"))  //annotation of balanceArmAndLegStartLine in comment
            newMotion.balanceArmAndLegStartLine = (int)newMotion.lines.size() + 1; //start with next line
          else if(!strcmp(sval[0], "\"@odometryOffset")) //annotation of critical line as comment
          {
            newMotion.odometryOffset.translation.x() = (float)atof(sval[1]);
            newMotion.odometryOffset.translation.y() = (float)atof(sval[2]);
            newMotion.odometryOffset.rotation = Angle::fromDegrees((float) atof(sval[3]));
          }
          else if(!strcmp(sval[0], "\"@continueTo")) //annotation of critical line as comment
          {
            newMotion.continueTo = static_cast<GetUpMotions::GetUpMotion>(TypeRegistry::getEnumValue(typeid(GetUpMotions::GetUpMotion).name(), string(sval[1])));
          }
          if(!strcmp(sval[0], "\"@endOptionalLine"))  //annotation of critical line as comment (since specialAction does not support it)
          {
            partOfPreviousOptionalLine = false;
            isElse = false;
            optionalLineAndsVar = false;
            condVec.clear();
          }
          else if(!strcmp(sval[0], "\"@optionalLine") && c > 0) //annotation of balanceArmAndLegStartLine in comment
          {
            partOfPreviousOptionalLine = false;
            partOfPreviousOptionalLineHelp = true;

            isElse = !strcmp(sval[1], "ElseBlock");
            optionalLineAndsVar = !strcmp(sval[2], "And");

            parseConditions(sval, condVec);
          }
          else if(!strcmp(sval[0], "\"@dynamicWait") && c > 0) //annotation of critical line as comment
          {
            waitConditions.clear();
            maxWaitingDuration = static_cast<float>(atof(sval[1]));
            dynamicWaitAnds = !strcmp(sval[2], "And");
            parseConditions(sval, waitConditions);
          }
          else if(c == -1 || sval[0][0] == '"' || !strncmp(sval[0], "//", 2) || !strcmp(sval[0], "motion_id") || !strcmp(sval[0], "transition")
                  || !strcmp(sval[0], "label"))
          {
            //skip comments and empty lines and all mof stuff not supported by doing nothing
          }
          else if(!strcmp(sval[0], "stiffness"))
          {
            if(c == Joints::numOfJoints + 2)
            {
              int val = 0;
              //for base stiffness use the first stiffness value of joint of limb of file
              if(!foundBaseStiffness)
              {
                int j = 1;
                int h = 0;
                while(j < Joints::numOfJoints + 1)
                {
                  if((!strcmp(sval[j], "-1") || sscanf(sval[j], "%i", &val) == 1) && val >= 0 && val <= 100 && strcmp(sval[j], "*"))
                  {
                    newMotion.baseLimbStiffness[h] = atoi(sval[j]);
                  }
                  else
                  {
                    char buffer[50];
                    sprintf(buffer, "%s(%i) : error: stiffness data format -> %s\n", name, line, sval[j]);
                    OUTPUT_ERROR(buffer);
                    return;
                  }
                  j = (h == 0) ? j + 2 : j + 6;
                  ++h;
                }
                foundBaseStiffness = true;
              }
              else
              {
                //save singleMotorStiffnessChange and add to next line
                for(int j = 1; j < Joints::numOfJoints + 1; ++j)
                  if((!strcmp(sval[j], "-1") || sscanf(sval[j], "%i", &val) == 1) && val >= 0 && val <= 100 && strcmp(sval[j], "*"))
                  {
                    int s = atoi(sval[j]);
                    if(newMotion.baseLimbStiffness[getBaseLimb(j - 1)] != s)
                      singleMotorStiffnessChange.emplace_back(static_cast<Joints::Joint>(j - 1), s);
                  }
                  else
                  {
                    char buffer[50];
                    sprintf(buffer, "%s(%i) : error: stiffness data format -> %s\n", name, line, sval[j]);
                    OUTPUT_ERROR(buffer);
                    return;
                  }
              }
            }
            else
            {
              char buffer[50];
              sprintf(buffer, "%s(%i) : error: stiffness format\n", name, line);
              OUTPUT_ERROR(buffer);
              return;
            }
          }
          else if(c == Joints::numOfJoints + 2)
          {
            int val = 0;
            MofLine motion_line;
            motion_line.critical = nextLineCritical;
            nextLineCritical = false;
            motion_line.singleMotorStiffnessChange = singleMotorStiffnessChange;
            singleMotorStiffnessChange.clear();
            motion_line.conditions = condVec;
            motion_line.optionalLineAnds = optionalLineAndsVar;
            motion_line.isPartOfPreviousOptionalLine = partOfPreviousOptionalLine;
            motion_line.isElseBlock = isElse;
            motion_line.balanceWithHipAndAnkleY = isBalanceWithHipAndAnkleY;
            motion_line.balanceWithHipAndAnkleX = isBalanceWithHipAndAnkleX;
            motion_line.waitDuration = maxWaitingDuration;
            motion_line.waitConditionAnds = dynamicWaitAnds;
            motion_line.waitConditions = waitConditions;
            if(!partOfPreviousOptionalLine && partOfPreviousOptionalLineHelp)
            {
              partOfPreviousOptionalLine = true;
              partOfPreviousOptionalLineHelp = false;
            }
            motion_line.increaseBalanceY = isIncreaseBalanceY;
            motion_line.increaseBalanceX = isIncreaseBalanceX;

            isBalanceWithHipAndAnkleY = false;
            isBalanceWithHipAndAnkleX = false;
            isIncreaseBalanceY = false;
            isIncreaseBalanceX = false;
            isElse = false;
            optionalLineAndsVar = false;
            condVec.clear();
            maxWaitingDuration = 0.f;
            dynamicWaitAnds = false;
            waitConditions.clear();

            for(int j = 0; j < Joints::numOfJoints; ++j)
            {
              if(!strcmp(sval[j], "-"))
              {
                *getJoint(j, motion_line) = JointAngles::off;
              }
              else if(!strcmp(sval[j], "*"))
              {
                *getJoint(j, motion_line) = JointAngles::ignore;
              }
              else if(sscanf(sval[j], "%i", &val) == 1 && val >= -210 && val <= 210)
              {
                *getJoint(j, motion_line) = (float)atof(sval[j]);
              }
              else
              {
                char buffer[50];
                sprintf(buffer, "%s(%i) : error: joint data format -> %s", name, line, sval[j]);
                OUTPUT_ERROR(buffer);
                return;
              }
            }
            if(sscanf(sval[Joints::numOfJoints], "%i", &val) == 1 && (val >= 0 || val <= 3))
            {
              //do nothing with interpolation flag
            }
            else
            {
              char buffer[50];
              sprintf(buffer, "%s(%i) : error: interpolation data format", name, line);
              OUTPUT_ERROR(buffer);
              return;
            }
            if(sscanf(sval[Joints::numOfJoints + 1], "%i", &val) == 1 && val > 0)
            {
              motion_line.duration = (float) atoi(sval[Joints::numOfJoints + 1]);
            }
            else
            {
              char buffer[50];
              sprintf(buffer, "%s(%i) : error: time data format", name, line);
              OUTPUT_ERROR(buffer);
              return;
            }
            newMotion.lines.emplace_back(motion_line);
          }
          else
          {
            char buffer[50];
            sprintf(buffer, "%s(%i) : error: illegal number of arguments", name, line);
            OUTPUT_ERROR(buffer);
            return;
          }
          ++line;
          t = u + 1;
        }
        else
          t += strlen(t);
      }
    }
    else
    {
      char buffer[50];
      sprintf(buffer, "error reading from %s. Aborting.", name);
      OUTPUT_ERROR(buffer);
      return;
    }
    mofs[motionName] = newMotion;
  }
}

Angle GetUpEngine::convertToAngleSpecialCases(float angle)
{
  return angle == JointAngles::off || angle == JointAngles::ignore ? static_cast<Angle>(angle) : Angle::fromDegrees(angle);
}

//The Treshold of 10 has no deeper meaning.
//It is just a "feeling" of a threshold. I did not test any other one
float GetUpEngine::correctAngleMotion(float nextAngle, float lastAngle, float dif, bool isArm)
{
  if(((nextAngle >= lastAngle && dif > 0.f) || (nextAngle <= lastAngle && dif < 0.f)) && lastAngle < 130.f && lastAngle > -130.f && nextAngle < 130.f && nextAngle > -130.f)
  {
    if(dif >= 0.f)
    {
      if(isArm)
      {
        return nextAngle + std::min(dif, upperArmBorder);
      }
      return nextAngle + std::min(dif, upperLegBorder);
    }
    else
    {
      if(isArm)
      {
        return nextAngle + std::max(dif, lowerArmBorder);
      }
      return nextAngle + std::max(dif, lowerLegBorder);
    }
  }
  return nextAngle;
}

void GetUpEngine::compareMotions(GetUpMotion motionIdBase, GetUpMotion motionIdCompare)
{
  Mof& motionBase = mofs[motionIdBase];
  Mof& motionCompare = mofs[motionIdCompare];

  comparison = GetUpComparison();

  auto angleDiff = [&](float angle1, float angle2)
  {
    if(angle1 == JointAngles::ignore || angle1 == JointAngles::off)
      angle1 = angle2;
    if(angle2 == JointAngles::ignore || angle2 == JointAngles::off)
      angle2 = angle1;
    return angle1 - angle2;
  };

  auto compareLinesAngles = [&](MofLine& lineBase, MofLine& lineCompare)
  {
    float deviation = 0;
    for(size_t i = 0; i < 2; i++)
    {
      deviation += pow(angleDiff(lineBase.head[i], lineCompare.head[i]), 2);
    }
    for(size_t i = 0; i < 6; i++)
    {
      deviation += pow(angleDiff(lineBase.leftArm[i], lineCompare.leftArm[i]), 2);
      deviation += pow(angleDiff(lineBase.rightArm[i], lineCompare.rightArm[i]), 2);
      deviation += pow(angleDiff(lineBase.leftLeg[i], lineCompare.leftLeg[i]), 2);
      deviation += pow(angleDiff(lineBase.rightLeg[i], lineCompare.rightLeg[i]), 2);
    }
    comparison.baseComparisonLines.back().lineComparisons.back().anglesDeviation = sqrt(deviation / 26);
  };

  auto compareLinesDuration = [&](MofLine& lineBase, MofLine& lineCompare)
  {
    comparison.baseComparisonLines.back().lineComparisons.back().durationDifference = lineBase.duration - lineCompare.duration;
  };

  auto compareLinesCritical = [&](MofLine& lineBase, MofLine& lineCompare)
  {
    comparison.baseComparisonLines.back().lineComparisons.back().criticalIdentical = lineBase.critical == lineCompare.critical;
  };

  auto compareLines = [&](MofLine& lineBase, MofLine& lineCompare)
  {
    comparison.baseComparisonLines.back().lineComparisons.push_back(GetUpComparisonLine());
    compareLinesAngles(lineBase, lineCompare);
    compareLinesDuration(lineBase, lineCompare);
    compareLinesCritical(lineBase, lineCompare);
    if(comparison.baseComparisonLines.back().lineComparisons.back().criticalIdentical && comparison.baseComparisonLines.back().lineComparisons.back().durationDifference == 0.0f && comparison.baseComparisonLines.back().lineComparisons.back().anglesDeviation == 0.0f)
    {
      OUTPUT_TEXT((int)(comparison.baseComparisonLines.back().startLineBase + comparison.baseComparisonLines.back().lineComparisons.size()) << " matches " << (int)(comparison.baseComparisonLines.back().startLineCompare + comparison.baseComparisonLines.back().lineComparisons.size()));
    }
  };

  for(size_t i = 0; i < motionBase.lines.size() + motionCompare.lines.size() - 1; i++)
  {
    size_t startLineBase = i < motionCompare.lines.size() ? 0 : i + 1 - motionCompare.lines.size();
    size_t startLineCompare = i < motionCompare.lines.size() ? motionCompare.lines.size() - 1 - i : 0;
    size_t comparisons = min(motionBase.lines.size() - startLineBase, motionCompare.lines.size() - startLineCompare);
    GetUpComparisonBaseLine baseline;
    baseline.startLineBase = static_cast<int>(startLineBase);
    baseline.startLineCompare = static_cast<int>(startLineCompare);
    comparison.baseComparisonLines.push_back(baseline);

    for(size_t j = 0; j < comparisons; j++)
    {
      compareLines(motionBase.lines[startLineBase + j], motionCompare.lines[startLineCompare + j]);
    }
  }
}

void GetUpEngine::skipForOptionalLine()
{
  bool doneSkipping = false;

  while(!doneSkipping && lineCounter < maxCounter)
  {
    //We are not in an optionalLine yet but found one
    if(!isInOptionalLine && mofs[motionID].lines[lineCounter].conditions.size() > 0)
    {
      doneSkipping = isInOptionalLine = checkConditions(mofs[motionID].lines[lineCounter].conditions, mofs[motionID].lines[lineCounter].optionalLineAnds);
    }
    //We were in an optionalLine and found the end
    else if(isInOptionalLine && !mofs[motionID].lines[lineCounter].isPartOfPreviousOptionalLine)
    {
      //We found an optionalLine-ElseBlock
      if(mofs[motionID].lines[lineCounter].isElseBlock)
      {
        doneSkipping = false;
        isInOptionalLine = false;
      }
      //We found an optionalLine-IfBlock
      else if(mofs[motionID].lines[lineCounter].conditions.size() > 0)
        doneSkipping = isInOptionalLine = checkConditions(mofs[motionID].lines[lineCounter].conditions, mofs[motionID].lines[lineCounter].optionalLineAnds);
      //We found the end of the optionalLine without a following else-Block or new optionalLine
      else
        doneSkipping = true;
    }
    //This case should never happen but is a failsave if a case at another place in the getUpEnginge is not correctly implemented
    else
      doneSkipping = isInOptionalLine || !mofs[motionID].lines[lineCounter].isPartOfPreviousOptionalLine;
    if(!doneSkipping)
    {
      ++lineCounter;
      while(lineCounter < maxCounter && mofs[motionID].lines[lineCounter].isPartOfPreviousOptionalLine)
      {
        ++lineCounter;
      }
    }
  }
}

bool GetUpEngine::checkConditions(vector<Condition> conditions, bool useAnd)
{
  int conditionSize = (int) conditions.size();
  // Needs to match the order of definition
  float variableValues[numOfConditionVars] =
  {
    theInertialData.angle.y().toDegrees(),
    theInertialData.angle.x().toDegrees(),
    theDamageConfigurationBody.optionalLineVersionFront,
    theDamageConfigurationBody.optionalLineVersionBack,
  };
  for(int i = 0; i < conditionSize; ++i)
  {
    bool check = variableValues[conditions[i].variable] >= conditions[i].lowerFloat && variableValues[conditions[i].variable] <= conditions[i].higherFloat;
    if(conditions[i].isNot)
      check = !check;
    if(!check && useAnd)
      return false;
    else if(check && !useAnd)
      return true;
  }
  // Conjunction true if nothing was false, disjunction false if nothing was true
  return useAnd;
}

MAKE_MODULE(GetUpEngine, motionControl)
