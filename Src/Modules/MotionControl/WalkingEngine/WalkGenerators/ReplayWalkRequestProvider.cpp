/**
 * @file ReplayWalkRequestProvider.cpp
 *
 * This file declares a module, that can replay a recorded walk request on a robot to test the walking.
 *
 * @author Philip Reichenberg
 */

#include "ReplayWalkRequestProvider.h"
#include "Debugging/DebugDrawings.h"
#include "Platform/SystemCall.h"

MAKE_MODULE(ReplayWalkRequestProvider);

void ReplayWalkRequestProvider::update(ReplayWalkRequestGenerator& request)
{
  bool cycle = false;
  MODIFY("module:ReplayWalkRequestProvider:startRecord", recordWalkPhase);
  MODIFY("module:ReplayWalkRequestProvider:initRecord", initRecord);
  MODIFY_ONCE("module:ReplayWalkRequestProvider:cycle", cycle);
  if(theGameState.isPenalized())
  {
    indexCurrentWalkRequest = 0;
    const bool thisBumperState = theKeyStates.pressed[KeyStates::lFootLeft] || theKeyStates.pressed[KeyStates::lFootRight];
    cycle |= lastBumperState && !thisBumperState && theFrameInfo.getTimeSince(lastSwitch) > 800;
    if(cycle)
    {
      currentWalkRequest = (currentWalkRequest + 1) % motionRequests.size();
      SystemCall::say(motionRequests[currentWalkRequest].description.c_str());
      lastSwitch = theFrameInfo.time;
      OUTPUT_TEXT(motionRequests[currentWalkRequest].description);
    }
    lastBumperState = thisBumperState;
  }

  if(!recordWalkPhase)
    lastSaveTimestamp = 0;

  if(recordWalkPhase && (lastStepTargetCopyTimestamp < lastSaveTimestamp || lastSaveTimestamp == 0))
  {
    actualLastStepTarget = theWalkStepData.stepTarget;
    lastStepTargetCopyTimestamp = theFrameInfo.time;
    lastSaveTimestamp = theFrameInfo.time;
  }

  request.createPhase = [this](const MotionRequest&, const MotionPhase& lastPhase)
  {
    const bool lastLeft = theWalkGenerator.wasLastPhaseLeftPhase(lastPhase);
    if(indexCurrentWalkRequest == -1)
    {
      if(currentWalkRequest < static_cast<int>(motionRequests.size()))
        indexCurrentWalkRequest = 0;
    }
    if(indexCurrentWalkRequest >= static_cast<int>(motionRequests[currentWalkRequest].walkRequests.size()))
      return theWalkGenerator.createPhase(Pose2f(), lastPhase, 0.f);
    else if(currentWalkRequest >= 0 && indexCurrentWalkRequest >= 0)
    {
      if((indexCurrentWalkRequest == 0 && lastLeft != motionRequests[currentWalkRequest].isLeftPhaseStart) || indexCurrentWalkRequest > 0)
      {
        // Walk kick
        if(motionRequests[currentWalkRequest].walkRequests[indexCurrentWalkRequest].isWalkKickPhase)
        {
          // Fail safe, so robot does not damage itself
          const float yTarget = motionRequests[currentWalkRequest].walkRequests[indexCurrentWalkRequest].walkKickStep.keyframe[motionRequests[currentWalkRequest].walkRequests[indexCurrentWalkRequest].walkKickStep.keyframe.size() - 1].stepTarget.translation.y();
          if(yTarget != 0.f && ((yTarget < 0.f) != lastLeft || (yTarget > 0.f) == lastLeft))
          {
            OUTPUT_ERROR("Wrong Foot Support");
            indexCurrentWalkRequest = static_cast<int>(motionRequests[currentWalkRequest].walkRequests.size());
            return theWalkGenerator.createPhase(Pose2f(), lastPhase, 0.f);
          }
          const int currentIndex = indexCurrentWalkRequest;
          indexCurrentWalkRequest++;
          return theWalkGenerator.createPhaseWithNextPhase(motionRequests[currentWalkRequest].walkRequests[currentIndex].walkKickStep,
                                                           lastPhase, WalkGenerator::CreateNextPhaseCallback(), motionRequests[currentWalkRequest].walkRequests[currentIndex].delay);
        }

        // Fail safe, so robot does not damage itself
        const float yTarget = motionRequests[currentWalkRequest].walkRequests[indexCurrentWalkRequest].step.translation.y();
        if(yTarget != 0.f && ((yTarget < 0.f) != lastLeft || (yTarget > 0.f) == lastLeft))
        {
          OUTPUT_ERROR("Wrong Foot Support");
          indexCurrentWalkRequest = static_cast<int>(motionRequests[currentWalkRequest].walkRequests.size());
          return theWalkGenerator.createPhase(Pose2f(), lastPhase, 0.f);
        }

        // Normal step
        const int currentIndex = indexCurrentWalkRequest;
        indexCurrentWalkRequest++;
        return theWalkGenerator.createPhase(motionRequests[currentWalkRequest].walkRequests[currentIndex].step, lastPhase, motionRequests[currentWalkRequest].walkRequests[currentIndex].delay);
      }
      else if(indexCurrentWalkRequest == 0 && lastLeft == motionRequests[currentWalkRequest].isLeftPhaseStart)
      {
        if(lastPhase.type != MotionPhase::walk)
          return theWalkGenerator.createPhase(Pose2f(0_deg, 0.f, -1.f), lastPhase, 0.f);
        else
          return theWalkGenerator.createPhase(Pose2f(0_deg, 0.f, motionRequests[currentWalkRequest].isLeftPhaseStart != lastLeft ?
                                                     (motionRequests[currentWalkRequest].isLeftPhaseStart ? -1.f : 1.f) :
                                                     0.f), lastPhase, 0.f);
      }
    }
    return theWalkGenerator.createPhase(Pose2f(), lastPhase, 0.f);
  };

  request.savePhase = [this](const MotionPhase& lastPhase)
  {
    if(recordWalkPhase && lastPhase.type == MotionPhase::walk)
    {
      const bool lastLeft = theWalkGenerator.wasLastPhaseLeftPhase(lastPhase);
      if(initRecord)
      {
        initRecord = false;
        indexCurrentWalkRequest = -1;
      }
      if(motionRequests.size() == 0 || indexCurrentWalkRequest == -1)
      {
        indexCurrentWalkRequest = 0;
        motionRequests.push_back(WalkPhaseData());
        currentWalkRequest = static_cast<int>(motionRequests.size()) - 1;
        motionRequests[currentWalkRequest].isLeftPhaseStart = lastLeft;
      }
      Pose2f step;
      WalkKickStep walkKickStep;
      bool wasInWalkKick = false;
      theWalkGenerator.getLastWalkPhase(walkKickStep, step, lastPhase);
      for(WalkKickStep::StepKeyframe& keyframe : walkKickStep.keyframe)
        keyframe.reachedWaitPosition = false;
      wasInWalkKick = theWalkGenerator.wasLastPhaseInWalkKick(lastPhase);
      ReplayRequest replayRequest;
      replayRequest.walkKickStep = walkKickStep;
      replayRequest.isWalkKickPhase = wasInWalkKick;
      if(wasInWalkKick)
      {
        replayRequest.step = step; // for InWalkKick use the calculated one
        replayRequest.walkKickStep.isReplayWalkRequest = true;
      }
      else
      {
        replayRequest.step = actualLastStepTarget; // otherwise use the one from the log
        replayRequest.walkKickStep.currentKickVariant = std::optional<WalkKickVariant>(); // set empty
      }
      motionRequests[currentWalkRequest].walkRequests.push_back(replayRequest);
      lastSaveTimestamp = theFrameInfo.time;
      OUTPUT_TEXT("recorded");
    }
  };
}
