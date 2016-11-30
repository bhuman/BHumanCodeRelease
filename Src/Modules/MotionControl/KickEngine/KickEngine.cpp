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

      if(KickRequest::getKickMotionFromName(parameters.name) < KickRequest::none)
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

  for(int i = 0; i < KickRequest::numOfKickMotionIDs - 2; ++i)
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
      OUTPUT_TEXT("Warning: The kick motion file for id " << KickRequest::getName((KickRequest::KickMotionID) i) << " is missing.");
      fprintf(stderr, "Warning: The kick motion file for id %s is missing. \n", KickRequest::getName((KickRequest::KickMotionID) i));
    }
  }

  //This is needed for adding new kicks
  KickEngineParameters newKickMotion;
  strcpy(newKickMotion.name, "newKick");
  params.push_back(newKickMotion);
};

void KickEngine::update(KickEngineOutput& kickEngineOutput)
{
  if(theLegMotionSelection.ratios[MotionRequest::kick] > 0.f)
  {
    data.setCycleTime(theFrameInfo.cycleTime);

    if(theLegMotionSelection.ratios[MotionRequest::kick] < 1.f && !compensated)
      compensate = true;

    data.setRobotModel(theRobotModel);

    if(data.sitOutTransitionDisturbance(compensate, compensated, theInertialData, kickEngineOutput, theJointRequest, theFrameInfo))
    {
      if(data.activateNewMotion(theMotionRequest.kickRequest, kickEngineOutput.isLeavingPossible) && theMotionRequest.motion == MotionRequest::kick)
      {
        data.initData(theFrameInfo, theMotionRequest, params, theJointAngles, theTorsoMatrix);
        data.setCurrentKickRequest(theMotionRequest);
        data.setExecutedKickRequest(kickEngineOutput.executedKickRequest);

        data.internalIsLeavingPossible = false;
        kickEngineOutput.isLeavingPossible = false;

        kickEngineOutput.odometryOffset = Pose2f();

        for(int i = Joints::lShoulderPitch; i < Joints::numOfJoints; ++i)
          kickEngineOutput.stiffnessData.stiffnesses[i] = 100;

        kickEngineOutput.isStable = true;
      }//this gotta go to config file and be more common

      if(data.checkPhaseTime(theFrameInfo, theJointAngles, theTorsoMatrix))
      {
        data.calcPhaseState();
        data.calcPositions();
        data.setStaticReference();
        timeSinceLastPhase = theFrameInfo.time;
      }
      else
      {
        kickEngineOutput.isLeavingPossible = true;
        data.internalIsLeavingPossible = true;
      }

      //  if(data.isMotionAlmostOver()) //last three phases are unstable
      //    kickEngineOutput.isStable = false;

      if(data.calcJoints(kickEngineOutput, theRobotDimensions, theHeadJointRequest))
      {
        data.balanceCOM(kickEngineOutput, theRobotDimensions, theMassCalibration);
        data.calcJoints(kickEngineOutput, theRobotDimensions, theHeadJointRequest);
        data.mirrorIfNecessary(kickEngineOutput);
      }
      data.addGyroBalance(kickEngineOutput, theJointCalibration, theInertialData, theLegMotionSelection.ratios[MotionRequest::kick]);
    }
  }
  else
    compensated = false;

  data.setEngineActivation(theLegMotionSelection.ratios[MotionRequest::kick]);
  data.ModifyData(theMotionRequest.kickRequest, kickEngineOutput, params);
}
