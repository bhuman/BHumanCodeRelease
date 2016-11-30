/*
 * @file GetUpEngine.cpp
 * @author <A href="mailto:judy@tzi.de">Judith Müller</A>
 */

#include "GetUpEngine.h"
#include "Platform/SystemCall.h"
#include "Tools/Debugging/Modify.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Platform/File.h"
#include "Tools/Debugging/Modify.h"
#include <string>
#include <functional>

using namespace std;

void GetUpEngine::update(GetUpEngineOutput& output)
{
#ifndef NDEBUG
  Mof::Motion motionToMof = Mof::Motion::stand;
  MODIFY_ENUM_ONCE("module:GetUpEngine:generateMofOf", motionToMof, Mof);
  if(motionToMof != Mof::Motion::stand)
    generateMofOfMotion(motionToMof);


  Mof::Motion mofToMotion = Mof::Motion::stand;
  MODIFY_ENUM_ONCE("module:GetUpEngine:generateMotionFromDummyMof", mofToMotion, Mof);
  if(mofToMotion != Mof::Motion::stand)
    generateMotionOfMof(mofToMotion);
#endif

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
            output.odometryOffset = internalOdometryOffset;
          else
            output.odometryOffset = Pose2f();

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

    target.stiffnessData.stiffnesses[i] = targetJoints.stiffnessData.stiffnesses[i];
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
        setCurrentMotion(Mof::recoverFromSide);
        setBaseLimbStiffness();
      }
      else if(bodyAngleY > 65_deg && theFrameInfo.getTimeSince(lastNotActiveTimeStamp) > 300)
      {
        //init stand up front
        state = working;
        if(p.forceOldGetUp || theDamageConfigurationBody.useOldStandUpFront)
          setCurrentMotion(Mof::front);
        else
          setCurrentMotion(Mof::frontFast);
        setBaseLimbStiffness();
      }
      //on back side
      else if(bodyAngleY < -65_deg && theFrameInfo.getTimeSince(lastNotActiveTimeStamp) > 300)
      {
        //init stand up back motion
        state = working;
        if(p.forceOldGetUp || theDamageConfigurationBody.useOldStandUpBack)
          setCurrentMotion(Mof::back);
        else
          setCurrentMotion(Mof::backFast);
        setBaseLimbStiffness();
      }
      //not fallen at all?
      else if(abs(bodyAngleY) < 35_deg && abs(bodyAngleX) < 50_deg && theFrameInfo.getTimeSince(lastNotActiveTimeStamp) > 500) //near upright
      {
        //init stand
        state = pickUp;
        setCurrentMotion(Mof::stand);
        setBaseLimbStiffness();
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
        setCurrentMotion(Mof::recovering);
        setBaseLimbStiffness();
      }
      break;
    }
    case schwalbe:
    {
      if(abs(theInertialData.angle.y()) > 35_deg) //not near upright
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
        setCurrentMotion(Mof::stand);
        setBaseLimbStiffness();
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

void GetUpEngine::setNextJoints(GetUpEngineOutput& output)
{
  //do stuff only if we are not at the end of the movement
  if(lineCounter < maxCounter && motionID > -1)
  {
    const float& time = p.mofs[motionID].lines[lineCounter].duration;
    ASSERT(time > 0);
    ratio = (float)theFrameInfo.getTimeSince(lineStartTimeStamp) / time;
    //check if we are done yet with the current line
    if(ratio > 1.f)
    {
      lineStartTimeStamp = theFrameInfo.time;
      startJoints = lastUnbalanced;
      ratio = 0.f;
      //update stiffness
      if(++lineCounter < maxCounter)
        for(unsigned i = 0; i < p.mofs[motionID].lines[lineCounter].singleMotorStiffnessChange.size(); ++i)
          targetJoints.stiffnessData.stiffnesses[p.mofs[motionID].lines[lineCounter].singleMotorStiffnessChange[i].joint] = p.mofs[motionID].lines[lineCounter].singleMotorStiffnessChange[i].s;
    }
    //are we still not at the end?
    if(lineCounter < maxCounter)
    {
      balance = (lineCounter >= p.mofs[motionID].balanceStartLine && p.mofs[motionID].balanceStartLine > -1);
      //set head joints
      for(int i = 0; i < 2; ++i)
        targetJoints.angles[i] = Angle::fromDegrees(p.mofs[motionID].lines[lineCounter].head[i]);
      //set arm joints
      for(int i = 0; i < 6; ++i)
      {
        targetJoints.angles[Joints::lShoulderPitch + i] = Angle::fromDegrees(p.mofs[motionID].lines[lineCounter].leftArm[i]);
        targetJoints.angles[Joints::rShoulderPitch + i] = Angle::fromDegrees(p.mofs[motionID].lines[lineCounter].rightArm[i]);
        targetJoints.angles[Joints::lHipYawPitch + i] = Angle::fromDegrees(p.mofs[motionID].lines[lineCounter].leftLeg[i]);
        targetJoints.angles[Joints::rHipYawPitch + i] = Angle::fromDegrees(p.mofs[motionID].lines[lineCounter].rightLeg[i]);
      }

      //mirror joints if necessary
      if(mirror)
      {
        JointRequest mirroredJoints;
        mirroredJoints.mirror(targetJoints);
        targetJoints = mirroredJoints;
      }

      if(p.mofs[motionID].lines[lineCounter].critical)
        verifyUprightTorso(output);
    }
  }
  interpolate(startJoints, targetJoints, ratio, output);
  lastUnbalanced = output;
  addBalance(output);
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
  if(motionID > -1)
  {
    for(int i = 0; i < 2; ++i)
      targetJoints.stiffnessData.stiffnesses[i] = p.mofs[motionID].baseLimbStiffness[0];
    //set arm joints
    for(int i = 0; i < 6; ++i)
    {
      targetJoints.stiffnessData.stiffnesses[Joints::lShoulderPitch + i] = p.mofs[motionID].baseLimbStiffness[1];
      targetJoints.stiffnessData.stiffnesses[Joints::rShoulderPitch + i] = p.mofs[motionID].baseLimbStiffness[2];
      targetJoints.stiffnessData.stiffnesses[Joints::lHipYawPitch + i] = p.mofs[motionID].baseLimbStiffness[3];
      targetJoints.stiffnessData.stiffnesses[Joints::rHipYawPitch + i] = p.mofs[motionID].baseLimbStiffness[4];
    }
  }
}

void GetUpEngine::setCurrentMotion(Mof::Motion current)
{
  motionID = -1;
  for(int i = 0; i < Mof::numOfMotions; ++i)
    if(p.mofs[i].name == current)
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
  maxCounter = static_cast<int>(p.mofs[motionID].lines.size());
  internalOdometryOffset = p.mofs[motionID].odometryOffset;
}

bool GetUpEngine::verifyUprightTorso(GetUpEngineOutput& output)
{
  //if the body is not almost upright recover or cry for help
  if(abs(theInertialData.angle.y()) > 35_deg)
  {
    setStiffnessAllJoints(output, 0);
    if(output.tryCounter >= p.maxNumOfUnsuccessfulTries) //if no more trys left let behavior cry for help.
    {
      state = schwalbe;
    }
    else
    {
      if(SystemCall::getMode() != SystemCall::simulatedRobot)
        output.tryCounter++;

      if(theDamageConfigurationBody.brokenStandUp == DamageConfigurationBody::allFine)
        mirror = !mirror; //try it mirrored then

      state = breakUp;
      breakUpTimeStamp = theFrameInfo.time;
    }
    return false;
  }

  return true;
}

void GetUpEngine::generateMofOfMotion(Mof::Motion motionName)
{
  int motion = -1;
  for(int i = 0; i < Mof::numOfMotions; ++i)
    if(p.mofs[i].name == motionName)
    {
      motion = i;
      break;
    }
  if(motion < 0)
    return;


  char name[512];
  sprintf(name, "%s/Config/mof/getUpEngineDummy.mof", File::getBHDir());
  FILE* f = fopen(name, "w"); //for simplicity discard the whole content
  if(!f)
    OUTPUT_ERROR("GetUpEngineDummy.mof was not found");
  else
  {
    fputs("\" IF YOU HAVE CHANGED THIS FILE DO NOT COMMIT IT!\n", f);
    fputs("motion_id = getUpEngineDummy\n", f);
    fputs("label start\n", f);

    // keep trac of stiffness changes, init with base stiffness
    int internalStiffness[Joints::numOfJoints];

    auto putStiffness = [&](FILE * f)
    {
      char stiffness[250] = "\nstiffness";
      for(int i = 0; i < Joints::numOfJoints; ++i)
      {
        strcat(stiffness, " ");
        strcat(stiffness, std::to_string(internalStiffness[i]).c_str());
      }
      strcat(stiffness, " 0\n\n");
      fputs(stiffness, f);
    };

    for(int i = 0; i < 2; ++i)
      internalStiffness[i] = p.mofs[motion].baseLimbStiffness[0];
    //set arm joints
    for(int i = 0; i < 6; ++i)
    {
      internalStiffness[Joints::lShoulderPitch + i] = p.mofs[motion].baseLimbStiffness[1];
      internalStiffness[Joints::rShoulderPitch + i] = p.mofs[motion].baseLimbStiffness[2];
      internalStiffness[Joints::lHipYawPitch + i] = p.mofs[motion].baseLimbStiffness[3];
      internalStiffness[Joints::rHipYawPitch + i] = p.mofs[motion].baseLimbStiffness[4];
    }

    putStiffness(f);

    //add lines now
    int lastLine = (int) p.mofs[motion].lines.size() - 1;
    for(int i = 0; i < (int) p.mofs[motion].lines.size(); i++)
    {
      if(!p.mofs[motion].lines[i].singleMotorStiffnessChange.empty())
      {
        for(unsigned j = 0; j < p.mofs[motion].lines[i].singleMotorStiffnessChange.size(); ++j)
          internalStiffness[p.mofs[motion].lines[i].singleMotorStiffnessChange[j].joint] = p.mofs[motion].lines[i].singleMotorStiffnessChange[j].s;

        putStiffness(f);
      }

      if(p.mofs[motion].lines[i].critical)
        fputs("\n\"@critical (upright check at start of interpolation to next line)\n\n", f);

      if(i == p.mofs[motion].balanceStartLine - 1) //add it before the start line
        fputs("\n\"@balanceStartLine (turn on balance at start of interpolation to next line)\n\n", f);

      //add label repeat before last line
      if(i == lastLine)
        fputs("label repeat\n", f);

      char line[250];
      sprintf(line, "%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f 1 %d\n",
              p.mofs[motion].lines[i].head[0], p.mofs[motion].lines[i].head[1],
              p.mofs[motion].lines[i].leftArm[0], p.mofs[motion].lines[i].leftArm[1], p.mofs[motion].lines[i].leftArm[2],
              p.mofs[motion].lines[i].leftArm[3], p.mofs[motion].lines[i].leftArm[4], p.mofs[motion].lines[i].leftArm[5],
              p.mofs[motion].lines[i].rightArm[0], p.mofs[motion].lines[i].rightArm[1], p.mofs[motion].lines[i].rightArm[2],
              p.mofs[motion].lines[i].rightArm[3], p.mofs[motion].lines[i].rightArm[4], p.mofs[motion].lines[i].rightArm[5],
              p.mofs[motion].lines[i].leftLeg[0], p.mofs[motion].lines[i].leftLeg[1], p.mofs[motion].lines[i].leftLeg[2],
              p.mofs[motion].lines[i].leftLeg[3], p.mofs[motion].lines[i].leftLeg[4], p.mofs[motion].lines[i].leftLeg[5],
              p.mofs[motion].lines[i].rightLeg[0], p.mofs[motion].lines[i].rightLeg[1], p.mofs[motion].lines[i].rightLeg[2],
              p.mofs[motion].lines[i].rightLeg[3], p.mofs[motion].lines[i].rightLeg[4], p.mofs[motion].lines[i].rightLeg[5],
              (int) p.mofs[motion].lines[i].duration);
      fputs(line, f);
    }
    //add odometry offset as comment
    char odometry[100];
    sprintf(odometry, "\n\"@odometryOffset %.2f %.2f %.2f\n", p.mofs[motion].odometryOffset.translation.x(), p.mofs[motion].odometryOffset.translation.y(), p.mofs[motion].odometryOffset.rotation.toDegrees());
    fputs(odometry, f);
    //add footer
    fputs("\ntransition getUpEngineDummy getUpEngineDummy repeat\n", f);
    fputs("transition allMotions extern start\n", f);
    fclose(f);
  }
}

void GetUpEngine::generateMotionOfMof(Mof::Motion motionName)
{
  int motion = -1;
  for(int i = 0; i < Mof::numOfMotions; ++i)
    if(p.mofs[i].name == motionName)
    {
      motion = i;
      break;
    }
  if(motion < 0)
    return;

  char name[512];
  sprintf(name, "%s/Config/mof/getUpEngineDummy.mof", File::getBHDir());
  FILE* f = fopen(name, "r");
  if(!f)
    OUTPUT_ERROR("GetUpEngineDummy.mof was not found");
  else
  {
    bool foundBaseStiffness(false);

    Mof newMotion;
    newMotion.name = static_cast<Mof::Motion>(motionName);
    newMotion.balanceStartLine = -1;

    int headStart(0),
        lArmStart(headStart + 2),
        rArmStart(lArmStart + 6),
        lLegStart(rArmStart + 6),
        rLegStart(lLegStart + 6);

    std::vector<StiffnessPair> singleMotorStiffnessChange;
    bool nextLineCritical(false);

    auto getBaseLimb = [&](int k)
    {
      return (k < lArmStart) ? 0 : ((k < rArmStart) ? 1 : ((k < lLegStart) ? 2 : ((k < rLegStart) ? 3 : 4)));
    };

    auto getJoint = [&](int k, MofLine & motion_line)
    {
      return (k < lArmStart) ? &motion_line.head[k] : ((k < rArmStart) ? &motion_line.leftArm[k - lArmStart] : ((k < lLegStart) ? &motion_line.rightArm[k - rArmStart] : ((k < rLegStart) ? &motion_line.leftLeg[k - lLegStart] : &motion_line.rightLeg[k - rLegStart])));
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
          else if(!strcmp(sval[0], "\"@balanceStartLine"))  //annotation of balanceStartLine in comment
            newMotion.balanceStartLine = (int)newMotion.lines.size() + 1; //start with next line
          else if(!strcmp(sval[0], "\"@odometryOffset"))
          {
            newMotion.odometryOffset.translation.x() = (float)atof(sval[1]);
            newMotion.odometryOffset.translation.y() = (float)atof(sval[2]);
            newMotion.odometryOffset.rotation = Angle::fromDegrees((float) atof(sval[3]));
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

            for(int j = 0; j < Joints::numOfJoints; ++j)
            {
              if(strcmp(sval[j], "*") && strcmp(sval[j], "-") && sscanf(sval[j], "%i", &val) == 1 && val >= -210 && val <= 210)
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
    p.mofs[motion] = newMotion;
    //and save it all
    OutMapFile out("getUpEngine.cfg");
    if(out.exists())
      out << p;
  }
}

MAKE_MODULE(GetUpEngine, motionControl)
