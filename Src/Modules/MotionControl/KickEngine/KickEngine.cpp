/*
 * @file KickEngine.cpp
 * This file implements a module that creates motions.
 * @author <A href="mailto:judy@tzi.de">Judith Müller</A>
 */

#include "KickEngine.h"
#include "KickEngineParameters.h"
#include "Platform/File.h"
#include "Representations/MotionControl/KickRequest.h"

#include <dirent.h>
#include <cstdio>
#include <cstring>
#include <cerrno>

MAKE_MODULE(KickEngine, motionControl)

KickEngine::KickEngine()
{
  params.reserve(10);

  char dirname[260];

  sprintf(dirname, "%s/Config/KickEngine/", File::getBHDir());
  DIR* dir = opendir(dirname);
  ASSERT(dir);
  struct dirent* file = readdir(dir);

  while(file != nullptr)
  {
    char name[260] = "";
    sprintf(name, "KickEngine/%s", file->d_name);

    if(strstr(name, ".kmc"))
    {
      InMapFile stream(name);
      ASSERT(stream.exists());

      KickEngineParameters parameters;
      stream >> parameters;

      sprintf(name, "%s", file->d_name);
      for(int i = 0; i < 260; i++)
      {
        if(name[i] == '.')
          name[i] = 0;
      }
      strcpy(parameters.name, name);

      if(KickRequest::getKickMotionFromName(parameters.name) < KickRequest::newKick)
        params.push_back(parameters);
      else
      {
        OUTPUT_TEXT("Warning: KickRequest is missing the id for " << parameters.name);
        fprintf(stderr, "Warning: KickRequest is missing the id for %s \n", parameters.name);
      }
    }
    file = readdir(dir);
  }
  closedir(dir);

  for(int i = 0; i < KickRequest::newKick; ++i)
  {
    int id = -1;
    for(unsigned int p = 0; p < params.size(); ++p)
    {
      if(KickRequest::getKickMotionFromName(&params[p].name[0]) == i)
      {
        id = i;
        break;
      }
    }
    if(id == -1)
    {
      OUTPUT_TEXT("Warning: The kick motion file for id " << TypeRegistry::getEnumName(static_cast<KickRequest::KickMotionID>(i)) << " is missing.");
      fprintf(stderr, "Warning: The kick motion file for id %s is missing. \n", TypeRegistry::getEnumName(static_cast<KickRequest::KickMotionID>(i)));
    }
  }

  //This is needed for adding new kicks
  KickEngineParameters newKickMotion;
  strcpy(newKickMotion.name, "newKick");
  params.push_back(newKickMotion);
};

void KickEngine::update(KickEngineOutput& kickEngineOutput)
{
  //Is the KickEngine activ?
  if(theLegMotionSelection.ratios[MotionRequest::kick] > 0.f)
  {
    //Did the KickEngine start went active or is about to shutdown?
    if(theLegMotionSelection.ratios[MotionRequest::kick] < 1.f && !compensated)
      compensate = true;

    if(theMotionRequest.kickRequest.kickMotionType != KickRequest::none)
      lastValidKickRequest = theMotionRequest.kickRequest;
    data.robotModel = theRobotModel;

    //Do we need to wait befor we can do a kick?
    if(data.sitOutTransitionDisturbance(compensate, compensated, theInertialData, kickEngineOutput, theJointRequest, theFrameInfo))
    {
      if(data.activateNewMotion(lastValidKickRequest, kickEngineOutput.isLeavingPossible) && lastValidKickRequest.kickMotionType != KickRequest::none)
      {
        data.initData(theFrameInfo, lastValidKickRequest, params, theJointAngles, theTorsoMatrix, kickEngineOutput, theRobotDimensions, theMassCalibration, theDamageConfigurationBody);
        data.currentKickRequest = lastValidKickRequest;
        data.setExecutedKickRequest(kickEngineOutput.executedKickRequest);

        data.internalIsLeavingPossible = false;
        kickEngineOutput.isLeavingPossible = false;

        data.calcOdometryOffset(kickEngineOutput, theRobotModel); //to init it
        kickEngineOutput.odometryOffset = Pose2f();

        for(int i = Joints::lShoulderPitch; i < Joints::numOfJoints; ++i)
          kickEngineOutput.stiffnessData.stiffnesses[i] = 100;

        boostState = 0;
        kickEngineOutput.isStable = true;
      }

      //Is our Kick not over?
      if(data.checkPhaseTime(theFrameInfo, theJointAngles, theTorsoMatrix))
      {
        data.calcPhaseState();
        data.calcPositions(theTorsoMatrix);
        timeSinceLastPhase = theFrameInfo.time;
      }
      //Our current kick is over
      else
      {
        kickEngineOutput.isLeavingPossible = true;
        data.internalIsLeavingPossible = true;
      }

      //Is the current kick id valid, then calculate the jointRequest once for the balanceCom()
      if(data.calcJoints(kickEngineOutput, theRobotDimensions, theDamageConfigurationBody))
      {
        data.balanceCOM(kickEngineOutput, theRobotDimensions, theMassCalibration);
        data.calcJoints(kickEngineOutput, theRobotDimensions, theDamageConfigurationBody);
        data.mirrorIfNecessary(kickEngineOutput);

        if(!theDamageConfigurationBody.dontBoost)
        {
          Pose3f real = theRobotModel.soleLeft.inverse() * theRobotModel.soleRight;
          if(kickEngineOutput.executedKickRequest.mirror)
            real.invert();
          if(boostState == 0 && real.translation.x() < -30)
            boostState = 1;
          if(boostState == 1 && real.translation.x() > -20)
            boostState = 2;
          if(boostState >= 2 && boostState <= 4)
          {
            boostState += boostState < 4 ? 1 : 0;
            data.BOOST(kickEngineOutput, boostState);
            if(real.translation.x() > 80)
              boostState = 6;
          }
        }

        data.calcOdometryOffset(kickEngineOutput, theRobotModel);
      }
      data.addGyroBalance(kickEngineOutput, theJointLimits, theInertialData, theLegMotionSelection.ratios[MotionRequest::kick]);
    }
  }
  else
    compensated = false;

  data.setEngineActivation(theLegMotionSelection.ratios[MotionRequest::kick]);
  data.ModifyData(theMotionRequest.kickRequest, kickEngineOutput, params);
}
