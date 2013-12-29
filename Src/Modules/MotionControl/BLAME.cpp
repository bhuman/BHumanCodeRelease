/*
* @file BLAME.cpp
* This file implements a module that creates motions.
* @author <A href="mailto:judy@tzi.de">Judith Müller</A>
*/

#include "BLAME.h"

#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Representations/MotionControl/BikeRequest.h"
#include "Tools/Debugging/Modify.h"
#include "BIKEParameters.h"
#include "Tools/InverseKinematic.h"
#include "Tools/Math/Pose3D.h"
#include <cerrno>

BLAME::BLAME():
  compensate(false),
  compensated(false)
{
  params.reserve(10);

  char dirname[260];

#ifdef WIN32
  sprintf(dirname, "%s/Config/Bike/*.bmc", File::getBHDir());
  WIN32_FIND_DATA findFileData;
  std::string fileName;
  HANDLE hFind = FindFirstFile(dirname, &findFileData);


  while(hFind != INVALID_HANDLE_VALUE)
  {
    char name[512];

    fileName = findFileData.cFileName;

    sprintf(name, "Bike/%s", fileName.c_str());


    InMapFile stream(name);
    ASSERT(stream.exists());

    BIKEParameters parameters;
    stream >> parameters;

    char temp[260];

    sprintf(temp, "%s", fileName.c_str());


    for(int i = 0; i < 260; i++)
    {
      if(temp[i] == '.') temp[i] = 0;
    }

    strcpy(parameters.name, temp);
    if(BikeRequest::getBMotionFromName(parameters.name) < BikeRequest::numOfBMotionIDs)
    {
      params.push_back(parameters);
    }
    else
    {
      OUTPUT(idText, text, "Warning: BikeRequest is missing the id for " << parameters.name);
    }

    if(!FindNextFile(hFind, &findFileData))break;
  }

#else //LINUX
  sprintf(dirname, "%s/Config/Bike/", File::getBHDir());
  DIR* dir = opendir(dirname);
  ASSERT(dir);
  struct dirent* file = readdir(dir);

  while(file != NULL)
  {
    char name[260];
    sprintf(name, "Bike/%s", file->d_name);

    if(strstr(name, ".bmc"))
    {
      InMapFile stream(name);
      ASSERT(stream.exists());

      BIKEParameters parameters;
      stream >> parameters;

      sprintf(name, "%s", file->d_name);
      for(int i = 0; i < 260; i++)
      {
        if(name[i] == '.') name[i] = 0;
      }
      strcpy(parameters.name, name);

      if(BikeRequest::getBMotionFromName(parameters.name) < BikeRequest::none)
      {
        params.push_back(parameters);
      }
      else
      {
        OUTPUT(idText, text, "Warning: BikeRequest is missing the id for " << parameters.name);
        fprintf(stderr, "Warning: BikeRequest is missing the id for %s \n", parameters.name);
      }

    }
    file = readdir(dir);
  }
  closedir(dir);

#endif //LINUX

  for(int i = 0; i < BikeRequest::numOfBMotionIDs - 2; ++i)
  {
    int id = -1;
    for(unsigned int p = 0; p < params.size(); ++p)
    {
      if(BikeRequest::getBMotionFromName(&params[p].name[0]) == i)
      {
        id = i;
        break;
      }
    }
    if(id == -1)
    {
      OUTPUT(idText, text, "Warning: The bike motion file for id " << BikeRequest::getName((BikeRequest::BMotionID) i) << " is missing.");
      fprintf(stderr, "Warning: The bike motion file for id %s is missing. \n", BikeRequest::getName((BikeRequest::BMotionID) i));
    }
  }

  //This is needed for adding new kicks
#ifndef RELEASE
  BIKEParameters newBMotion;
  strcpy(newBMotion.name, "newKick");
  params.push_back(newBMotion);
#endif
};

void BLAME::update(BikeEngineOutput& blameOutput)
{
  if(theMotionSelection.ratios[MotionRequest::bike] > 0.f)
  {
    data.setCycleTime(theFrameInfo.cycleTime);

    if(theMotionSelection.ratios[MotionRequest::bike] < 1.f && !compensated) compensate = true;

    data.setRobotModel(theRobotModel);

    if(data.sitOutTransitionDisturbance(compensate, compensated, theFilteredSensorData, blameOutput, theWalkingEngineStandOutput, theFrameInfo))
    {
      if(data.activateNewMotion(theMotionRequest.bikeRequest, blameOutput.isLeavingPossible))
      {
        data.initData(compensated, theFrameInfo, theMotionRequest, theRobotDimensions, params, theFilteredJointData, theTorsoMatrix);
        data.setCurrentBikeRequest(theMotionRequest);
        data.setExecutedBikeRequest(blameOutput.executedBikeRequest);

        blameOutput.isLeavingPossible = false;

        blameOutput.odometryOffset = Pose2D();

        for(int i = JointData::LShoulderPitch; i < JointData::numOfJoints; ++i)
          blameOutput.jointHardness.hardness[i] = 100;

        blameOutput.isStable = true;
      }

      if(data.checkPhaseTime(theFrameInfo, theRobotDimensions, theFilteredJointData, theTorsoMatrix))
      {
        data.calcPhaseState();
        data.calcPositions(blameOutput, theFilteredJointData);
        data.setStaticReference();
      }
      else
      {
        blameOutput.isLeavingPossible = true;
      }

      //  if(data.isMotionAlmostOver()) //last three phases are unstable
      //	blameOutput.isStable = false;

      if(data.calcJoints(blameOutput, theRobotDimensions, theFilteredJointData))
      {
#ifndef RELEASE
        data.debugFormMode(params);
#endif
        data.balanceCOM(blameOutput, theRobotDimensions, theMassCalibration, theFilteredJointData);
        data.calcJoints(blameOutput, theRobotDimensions, theFilteredJointData);
        data.mirrorIfNecessary(blameOutput);
      }
      data.addGyroBalance(blameOutput, theJointCalibration, theFilteredSensorData, theMotionSelection.ratios[MotionRequest::bike]);
    }
  }
  else
  {
    compensated = false;
  }


  data.setEngineActivation(theMotionSelection.ratios[MotionRequest::bike]);
  data.ModifyData(theMotionRequest.bikeRequest, blameOutput, params);
}

MAKE_MODULE(BLAME, Motion Control)

