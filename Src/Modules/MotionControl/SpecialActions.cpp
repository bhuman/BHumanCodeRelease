/**
* @file SpecialActions.cpp
* This file implements a module that creates the motions of special actions.
* @author <A href="mailto:dueffert@informatik.hu-berlin.de">Uwe Düffert</A>
* @author Martin Lötzsch
* @author Max Risler
* @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
*/

#include "SpecialActions.h"
#include "Platform/BHAssert.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Math/Random.h"

PROCESS_WIDE_STORAGE(SpecialActions) SpecialActions::theInstance = 0;

void SpecialActions::MotionNetData::load(In& stream)
{
  for(int i = 0; i < SpecialActionRequest::numOfSpecialActionIDs; ++i)
    stream >> label_extern_start[i];
  label_extern_start[SpecialActionRequest::numOfSpecialActionIDs] = 0;

  int numberOfNodes;
  stream >> numberOfNodes;

  if(nodeArray)
    delete[] nodeArray;

  nodeArray = new MotionNetNode[numberOfNodes];

  for(int i = 0; i < numberOfNodes; ++i)
  {
    short s;
    stream >> s;

    switch(s)
    {
    case 2:
      nodeArray[i].d[0] = (short) MotionNetNode::typeTransition;
      stream >> nodeArray[i].d[1] >> nodeArray[i].d[JointData::numOfJoints + 3];
      break;
    case 1:
      nodeArray[i].d[0] = (short) MotionNetNode::typeConditionalTransition;
      stream >> nodeArray[i].d[1] >> nodeArray[i].d[2] >> nodeArray[i].d[JointData::numOfJoints + 3];
      break;
    case 4:
      nodeArray[i].d[0] = (short) MotionNetNode::typeHardness;
      for(int j = 1; j < JointData::numOfJoints + 3; j++)
        stream >> nodeArray[i].d[j];
      break;
    case 3:
      nodeArray[i].d[0] = (short) MotionNetNode::typeData;
      for(int j = 1; j < JointData::numOfJoints + 1; ++j)
      {
        stream >> nodeArray[i].d[j];
        if(nodeArray[i].d[j] != JointData::off    &&
           nodeArray[i].d[j] != JointData::ignore)
          nodeArray[i].d[j] = fromDegrees(nodeArray[i].d[j]);
      }
      for(int k = JointData::numOfJoints + 1; k < JointData::numOfJoints + 4; ++k)
        stream >> nodeArray[i].d[k];
      break;
    }
  }
}

SpecialActions::SpecialActions() :
  wasEndOfSpecialAction(false),
//hardnessInterpolationStart(0),
  hardnessInterpolationCounter(0),
  hardnessInterpolationLength(0),
  wasActive(false),
  dataRepetitionCounter(0),
  lastSpecialAction(SpecialActionRequest::numOfSpecialActionIDs),
  mirror(false)
{
  theInstance = this;

  InConfigFile file("specialActions.dat");
  if(!file.exists() || file.eof())
  {
    OUTPUT(idText, text, "SpecialActions : Error, 'specialActions.dat' not found.");
  }
  else
    motionNetData.load(file);

  // create an uninitialised motion request to set startup motion
  currentNode = motionNetData.label_extern_start[SpecialActionRequest().specialAction];

  // read entries from file
  InMapFile cm("specialActions.cfg");
  if(!cm.exists())
  {
    OUTPUT(idText, text, "SpecialActions : Error, 'specialActions.cfg' not found.");
  }
  else
  {
    OdometryParams infos;
    cm >> infos;
    for(std::vector<SpecialActionInfo>::const_iterator it = infos.specialActionInfos.begin(); it != infos.specialActionInfos.end(); it++)
    {
      infoTable[it->id] = SpecialActionInfo(*it);
      if(it->type == SpecialActionInfo::once || it->type == SpecialActionInfo::homogeneous)
      {
        infoTable[it->id].odometryOffset.rotation = it->odometryOffset.rotation;
        if(it->type == SpecialActionInfo::homogeneous)
        {
          // convert from mm/seconds to mm/tick
          float motionCycleTime = theFrameInfo.cycleTime;
          infoTable[it->type].odometryOffset.translation.x *= motionCycleTime;
          infoTable[it->type].odometryOffset.translation.y *= motionCycleTime;
          // convert from rad/seconds to rad/tick
          infoTable[it->type].odometryOffset.rotation *= motionCycleTime;
        }
      }
    }
  }
}

bool SpecialActions::getNextData(const SpecialActionRequest& specialActionRequest,
                                 SpecialActionsOutput& specialActionsOutput)
{
  while((MotionNetNode::NodeType)short(motionNetData.nodeArray[currentNode].d[0]) != MotionNetNode::typeData)
  {
    switch((MotionNetNode::NodeType)short(motionNetData.nodeArray[currentNode].d[0]))
    {
    case MotionNetNode::typeHardness:
      lastHardnessRequest = specialActionsOutput.jointHardness;//currentHardnessRequest;
      motionNetData.nodeArray[currentNode].toHardnessRequest(currentHardnessRequest, hardnessInterpolationLength);
      hardnessInterpolationCounter = hardnessInterpolationLength;
      currentNode++;
      break;
    case MotionNetNode::typeConditionalTransition:
      if(motionNetData.nodeArray[currentNode].d[2] != (short) specialActionRequest.specialAction)
      {
        currentNode++;
        break;
      }
      //no break here: if condition is true, continue with transition!
    case MotionNetNode::typeTransition:
      // follow transition
      if(currentNode == 0)  //we come from extern
        currentNode = motionNetData.label_extern_start[(short) specialActionRequest.specialAction];
      else
        currentNode = short(motionNetData.nodeArray[currentNode].d[1]);
      mirror = specialActionRequest.mirror;
      // leave if transition to external motion
      if(currentNode == 0)
        return false;
      break;
    case MotionNetNode::typeData:
      break;
    }
  }

  motionNetData.nodeArray[currentNode].toJointRequest(currentRequest, dataRepetitionLength, interpolationMode, deShakeMode);
  dataRepetitionCounter = dataRepetitionLength;

  specialActionsOutput.executedSpecialAction.specialAction = motionNetData.nodeArray[currentNode++].getSpecialActionID();
  specialActionsOutput.executedSpecialAction.mirror = mirror;
  specialActionsOutput.isMotionStable = infoTable[specialActionsOutput.executedSpecialAction.specialAction].isMotionStable;

  //get currently executed special action from motion net traversal:
  if(specialActionsOutput.executedSpecialAction.specialAction != lastSpecialAction)
  {
    currentInfo = infoTable[specialActionsOutput.executedSpecialAction.specialAction];
    lastSpecialAction = specialActionsOutput.executedSpecialAction.specialAction;
  }

  return true;
}

void SpecialActions::calculateJointRequest(JointRequest& jointRequest)
{
  float ratio, f, t;

  //joint angles
  if(interpolationMode)
  {
    ratio = dataRepetitionCounter / (float) dataRepetitionLength;
    for(int i = 0; i < JointData::numOfJoints; ++i)
    {
      f = lastRequest.angles[i];
      if(!mirror)
        t = currentRequest.angles[i];
      else
        t = currentRequest.mirror((JointData::Joint)i);
      // if fromAngle is off or ignore use JointData for further calculation
      if(f == JointData::off || f == JointData::ignore)
        f = theFilteredJointData.angles[i];

      // if toAngle is off or ignore -> turn joint off/ignore
      if(t == JointData::off || t == JointData::ignore)
        jointRequest.angles[i] = t;
      //interpolate
      else
        jointRequest.angles[i] = (float)(t + (f - t) * ratio);
    }
  }
  else
  {
    if(!mirror)
      jointRequest = currentRequest;
    else
      jointRequest.mirror(currentRequest);
  }

  //hardness stuff
  if(hardnessInterpolationCounter <= 0)
  {
    if(!mirror)
      jointRequest.jointHardness = currentHardnessRequest;
    else
      jointRequest.jointHardness.mirror(currentHardnessRequest);
  }
  else
  {
    ratio = ((float)hardnessInterpolationCounter) / hardnessInterpolationLength;
    int f, t;
    for(int i = 0; i < JointData::numOfJoints; i++)
    {
      f = lastHardnessRequest.hardness[i];
      if(!mirror)
        t = currentHardnessRequest.hardness[i];
      else
        t = currentHardnessRequest.mirror((JointData::Joint)i);
      if(t == f)
        jointRequest.jointHardness.hardness[i] = t;
      else
      {
        if(f == HardnessData::useDefault)
          f = theHardnessSettings.hardness[i];
        if(t == HardnessData::useDefault)
          t = mirror ? theHardnessSettings.mirror((JointData::Joint)i) : theHardnessSettings.hardness[i];
        jointRequest.jointHardness.hardness[i] = int(float(t) + float(f - t) * ratio);
      }
    }
  }
}

void SpecialActions::update(SpecialActionsOutput& specialActionsOutput)
{
  float speedFactor = 1.0f;
  MODIFY("parameters:SpecialActions:speedFactor", speedFactor);
  if(theMotionSelection.specialActionMode != MotionSelection::deactive)
  {
    specialActionsOutput.isLeavingPossible = true;
    if(dataRepetitionCounter <= 0)
    {
      if(!wasActive)
      {
        //entered from external motion
        currentNode = 0;
        for(int i = 0; i < JointData::numOfJoints; ++i)
          lastRequest.angles[i] = theFilteredJointData.angles[i];
        lastSpecialAction = SpecialActionRequest::numOfSpecialActionIDs;
      }

      // this is need when a special actions gets executed directly after another without
      // switching to a different motion for interpolating the hardness
      if(wasEndOfSpecialAction)
      {
        specialActionsOutput.jointHardness.resetToDefault();
        if(!mirror)
          lastHardnessRequest = currentHardnessRequest;
        else
          lastHardnessRequest.mirror(currentHardnessRequest);
        currentHardnessRequest.resetToDefault();
      }
      wasEndOfSpecialAction = false;
      // search next data, leave on transition to external motion
      if(!getNextData(theMotionSelection.specialActionRequest, specialActionsOutput))
      {
        wasActive = true;
        wasEndOfSpecialAction = true;
        specialActionsOutput.odometryOffset = Pose2D();
        return;
      }
    }
    else
    {
      dataRepetitionCounter -= int(theFrameInfo.cycleTime * 1000 * speedFactor);
      hardnessInterpolationCounter -= int(theFrameInfo.cycleTime * 1000 * speedFactor);
    }

    //set current joint values
    calculateJointRequest(specialActionsOutput);

    //odometry update
    if(currentInfo.type == SpecialActionInfo::homogeneous || currentInfo.type == SpecialActionInfo::once)
      if(mirror)
        specialActionsOutput.odometryOffset = Pose2D(-currentInfo.odometryOffset.rotation, currentInfo.odometryOffset.translation.x, -currentInfo.odometryOffset.translation.y);
      else
        specialActionsOutput.odometryOffset = currentInfo.odometryOffset;
    else
      specialActionsOutput.odometryOffset = Pose2D();
    if(currentInfo.type == SpecialActionInfo::once)
      currentInfo.type = SpecialActionInfo::none;

    //store value if current data line finished
    if(dataRepetitionCounter <= 0)
    {
      if(!mirror)
        lastRequest = currentRequest;
      else
        lastRequest.mirror(currentRequest);
    }
    specialActionsOutput.isLeavingPossible = false;
    if(deShakeMode)
      for(int i = JointData::LShoulderPitch; i <= JointData::RElbowRoll; ++i)
        if(randomFloat() < 0.25)
          specialActionsOutput.angles[i] = JointData::off;
  }
  wasActive = theMotionSelection.specialActionMode == MotionSelection::active;
}

bool SpecialActions::handleMessage(InMessage& message)
{
  return theInstance && theInstance->handleMessage2(message);
}

bool SpecialActions::handleMessage2(InMessage& message)
{
  if(message.getMessageID() == idMotionNet)
  {
    motionNetData.load(message.config);
    wasActive = false;
    dataRepetitionCounter = 0;
    return true;
  }
  else
    return false;
}

MAKE_MODULE(SpecialActions, Motion Control)
