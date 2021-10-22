/**
 * @file Talk.cpp
 *
 * This file implements the implementation of the Say and PlaySound skill.
 *
 * @author Arne Hasselbring
 * @author Jan Blumenkamp
 */

#include "Platform/SystemCall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Communication/GameInfo.h"
#include <string>

SKILL_IMPLEMENTATION(TalkImpl,
{,
  CALLS(Say),
  IMPLEMENTS(CountDownHalfTime),
  IMPLEMENTS(Say),
  IMPLEMENTS(PlaySound),
  REQUIRES(GameInfo),
});

class TalkImpl : public TalkImplBase
{
  void execute(const CountDownHalfTime&) override
  {
    const int secondsRemaining = static_cast<int>(theGameInfo.secsRemaining);
    if(secondsRemaining >= 0 && secondsRemaining == lastCountDownStep-1)
    {
      theSaySkill(std::to_string(secondsRemaining));
      lastCountDownStep = secondsRemaining;
    }
  }

  void execute(const Say& s) override
  {
    if(s.text != lastThingSaid)
    {
      SystemCall::say(s.text.c_str(), s.speed);
      lastThingSaid = s.text;
    }
  }

  void execute(const PlaySound& p) override
  {
    if(p.name != lastSoundPlayed)
    {
      SystemCall::playSound(p.name.c_str());
      lastSoundPlayed = p.name;
    }
  }

  void reset(const CountDownHalfTime&) override
  {
    lastCountDownStep = 11;
  }

  void reset(const Say&) override
  {
    lastThingSaid.clear();
  }

  void reset(const PlaySound&) override
  {
    lastSoundPlayed.clear();
  }

  std::string lastThingSaid; /**< The text of the last synthesized sound. */
  std::string lastSoundPlayed; /**< The name of the last sound file played. */
  int lastCountDownStep = 11;
};

MAKE_SKILL_IMPLEMENTATION(TalkImpl);
