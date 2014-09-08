/*
* @file KickEngine.cpp
* This file implements a module that creates motions.
* @author <A href="mailto:judy@tzi.de">Judith Müller</A>
*/

#include "KickEngine.h"

#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Representations/MotionControl/KickRequest.h"
#include "Tools/Debugging/Modify.h"
#include "KickEngineParameters.h"
#include "Tools/InverseKinematic.h"
#include "Tools/Math/Pose3D.h"
#include <cerrno>

KickEngine::KickEngine() :
compensate(false),
compensated(false),
timeSinceLastPhase(0)
{
  params.reserve(10);

  char dirname[260];

#ifdef WINDOWS
  sprintf(dirname, "%s/Config/KickEngine/*.kmc", File::getBHDir());
  WIN32_FIND_DATA findFileData;
  std::string fileName;
  HANDLE hFind = FindFirstFile(dirname, &findFileData);

  while(hFind != INVALID_HANDLE_VALUE)
  {
    char name[512];

    fileName = findFileData.cFileName;

    sprintf(name, "KickEngine/%s", fileName.c_str());

    InMapFile stream(name);
    ASSERT(stream.exists());

    KickEngineParameters parameters;
    stream >> parameters;

    char temp[260];

    sprintf(temp, "%s", fileName.c_str());

    for(int i = 0; i < 260; i++)
    {
      if(temp[i] == '.') temp[i] = 0;
    }

    strcpy(parameters.name, temp);
    if(KickRequest::getKickMotionFromName(parameters.name) < KickRequest::numOfKickMotionIDs)
    {
      params.push_back(parameters);
    }
    else
    {
      OUTPUT(idText, text, "Warning: KickRequest is missing the id for " << parameters.name);
    }

    if(!FindNextFile(hFind, &findFileData))break;
  }

#else //LINUX
  sprintf(dirname, "%s/Config/KickEngine/", File::getBHDir());
  DIR* dir = opendir(dirname);
  ASSERT(dir);
  struct dirent* file = readdir(dir);

  while(file != NULL)
  {
    char name[260];
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
        if(name[i] == '.') name[i] = 0;
      }
      strcpy(parameters.name, name);

      if(KickRequest::getKickMotionFromName(parameters.name) < KickRequest::none)
      {
        params.push_back(parameters);
      }
      else
      {
        OUTPUT(idText, text, "Warning: KickRequest is missing the id for " << parameters.name);
        fprintf(stderr, "Warning: KickRequest is missing the id for %s \n", parameters.name);
      }
    }
    file = readdir(dir);
  }
  closedir(dir);

#endif //LINUX

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
      OUTPUT(idText, text, "Warning: The kick motion file for id " << KickRequest::getName((KickRequest::KickMotionID) i) << " is missing.");
      fprintf(stderr, "Warning: The kick motion file for id %s is missing. \n", KickRequest::getName((KickRequest::KickMotionID) i));
    }
  }

  //This is needed for adding new kicks
#ifndef RELEASE
  KickEngineParameters newKickMotion;
  strcpy(newKickMotion.name, "newKick");
  params.push_back(newKickMotion);
#endif
};

void KickEngine::update(KickEngineOutput& kickEngineOutput)
{
  if(theMotionSelection.ratios[MotionRequest::kick] > 0.f)
  {
    data.setCycleTime(theFrameInfo.cycleTime);

    if(theMotionSelection.ratios[MotionRequest::kick] < 1.f && !compensated) compensate = true;

    data.setRobotModel(theRobotModel);

    if(data.sitOutTransitionDisturbance(compensate, compensated, theFilteredSensorData, kickEngineOutput, theWalkingEngineStandOutput, theFrameInfo))
    {
      if(data.activateNewMotion(theMotionRequest.kickRequest, kickEngineOutput.isLeavingPossible) && theMotionRequest.motion == MotionRequest::kick)
      {
        data.initData(compensated, theFrameInfo, theMotionRequest, theRobotDimensions, params, theFilteredJointData, theTorsoMatrix);
        data.setCurrentKickRequest(theMotionRequest);
        data.setExecutedKickRequest(kickEngineOutput.executedKickRequest);

        data.internalIsLeavingPossible = false;
        kickEngineOutput.isLeavingPossible = false;

        kickEngineOutput.odometryOffset = Pose2D();

        for(int i = JointData::LShoulderPitch; i < JointData::numOfJoints; ++i)
          kickEngineOutput.jointHardness.hardness[i] = 100;

        kickEngineOutput.isStable = true;
      }//this gotta go to config file and be more common

      if(data.checkPhaseTime(theFrameInfo, theRobotDimensions, theFilteredJointData, theTorsoMatrix))
      {
        data.calcPhaseState();
        data.calcPositions(kickEngineOutput, theFilteredJointData);
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
#ifndef RELEASE
        data.debugFormMode(params);
#endif
        data.balanceCOM(kickEngineOutput, theRobotDimensions, theMassCalibration, theFilteredJointData);
        data.calcJoints(kickEngineOutput, theRobotDimensions, theHeadJointRequest);
        data.mirrorIfNecessary(kickEngineOutput);
      }
      data.addGyroBalance(kickEngineOutput, theJointCalibration, theFilteredSensorData, theMotionSelection.ratios[MotionRequest::kick]);
    }
  }
  else
  {
    compensated = false;
  }

  data.setEngineActivation(theMotionSelection.ratios[MotionRequest::kick]);
  data.ModifyData(theMotionRequest.kickRequest, kickEngineOutput, params);
}

MAKE_MODULE(KickEngine, Motion Control)

