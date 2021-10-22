/**
 * @file TeamTalk.cpp
 *
 * This file implements the implementation of the TeamSay and TeamPlaySound skill.
 *
 * @author Jan Blumenkamp
 */

#include "Platform/SystemCall.h"
#include "Representations/BehaviorControl/KickoffState.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/TeamTalk.h"
#include "Representations/Modeling/RobotPose.h"
#include <queue>
#include <string>

SKILL_IMPLEMENTATION(TeamTalkImpl,
{,
  CALLS(TeamPropagator),
  CALLS(TeamSpeaker),
  IMPLEMENTS(TeamCountdown),
  IMPLEMENTS(TeamPropagator),
  IMPLEMENTS(TeamSpeaker),
  MODIFIES(TeamTalk),
  REQUIRES(KickoffState),
  REQUIRES(FrameInfo),
  REQUIRES(TeamData),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),
});

static std::vector<std::string> availableTeamTalk = {"Ball in play", "1", "2", "3", "4", "5", "6", "7", "8", "9"};
static char findIndex(std::string value)
{
  auto it = std::find(availableTeamTalk.begin(), availableTeamTalk.end(), value);
  return it == availableTeamTalk.end() ? -1 : static_cast<char>(std::distance(availableTeamTalk.begin(), it));
}

class TeamTalkImpl : public TeamTalkImplBase
{
  void execute(const TeamCountdown& s) override
  {
    if(!(wasDefensiveKickoff = wasDefensiveKickoff || !theKickoffState.allowedToEnterCenterCircle))
      return;

    int announcer = theRobotInfo.number;
    Vector2f announcerDis = theRobotPose.translation;
    for(const auto& teammate : theTeamData.teammates)
    {
      if(teammate.status != Teammate::PLAYING)
        continue;
      const Vector2f mateDis = teammate.theRobotPose.translation;
      if(announcerDis.x() <= mateDis.x() - 1000.f || (std::abs(announcerDis.x() - mateDis.x()) < 1000.f && std::abs(announcerDis.y()) > std::abs(mateDis.y())))
      {
        announcer = teammate.number;
        announcerDis = mateDis;
      }
    }

    if(announcer != theRobotInfo.number)
    {
      for(const auto& teammate : theTeamData.teammates)
        if(teammate.number == announcer)
          theTeamPropagatorSkill(teammate.theTeamTalk.say, teammate.theTeamTalk.timestamp);
    }
    else if(!countDownFinished)
    {
      int countdown = 9 - (s.stateTime-100) / 1000 - 1;
      const std::string nextTeamTalk = countdown == 0 ? "Ball in play" : std::to_string(countdown);
      char index = -1;

      if(s.stateTime < 9750 && theKickoffState.allowedToEnterCenterCircle)
      {
        SystemCall::say("Ball in play");
        theTeamPropagatorSkill(static_cast<char>(availableTeamTalk.size()), 0);
        countDownFinished = true;
      }
      else if(s.stateTime < 100 && (index = findIndex("9")) != -1 && index != lastSent)
        theTeamPropagatorSkill(index, theFrameInfo.time + 900);
      else if(s.stateTime <= 10100 && s.stateTime >= 850 && (s.stateTime % (s.stateTime / 1000 * 1000 + 850)) < 100 && (index = findIndex(nextTeamTalk)) != -1 && index != lastSent)
      {
        theTeamPropagatorSkill(index, theFrameInfo.time + 1000);
        if(nextTeamTalk == "Ball in play")  countDownFinished = true;
      }
    }
    theTeamSpeakerSkill();
  }

  void execute(const TeamPropagator& s) override
  {
    theTeamTalk.say = s.index;
    theTeamTalk.timestamp = s.timestamp;
    if(s.index < 0 || s.index >= static_cast<int>(availableTeamTalk.size()))
      teamTalkQueue = {};
    else
    {
      teamTalkQueue.push({s.index, s.timestamp});
      lastSent = s.index;
    }
  }

  void execute(const TeamSpeaker&) override
  {
    while(!teamTalkQueue.empty())
    {
      std::pair<char, unsigned int> announcement = teamTalkQueue.front();
      if(announcement.first < 0 || announcement.first >= static_cast<int>(availableTeamTalk.size()) ||
         (announcement.first == lastTeamTalk && announcement.second < lastTeamTalkTimestamp+1000) ||
         theFrameInfo.time > announcement.second+100)
        teamTalkQueue.pop();
      else if(announcement.second < theFrameInfo.time)
      {
        SystemCall::say(availableTeamTalk.at(announcement.first).c_str());
        lastTeamTalk = announcement.first;
        lastTeamTalkTimestamp = announcement.second;
        teamTalkQueue.pop();
      }
      else
        break;
    }
  }

  void reset(const TeamCountdown &) override
  {
    teamTalkQueue = {};
    wasDefensiveKickoff = false;
    countDownFinished = false;
    lastTeamTalk = -1, lastSent = -1;
    lastTeamTalkTimestamp = 0;
  }
  void reset(const TeamPropagator &) override {}
  void reset(const TeamSpeaker&) override {}

  std::queue<std::pair<char, unsigned int>> teamTalkQueue;
  bool wasDefensiveKickoff = false, countDownFinished = false;
  char lastTeamTalk = -1, lastSent = -1;
  unsigned int lastTeamTalkTimestamp = 0;
};

MAKE_SKILL_IMPLEMENTATION(TeamTalkImpl);
