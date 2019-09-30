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
#include <string>

SKILL_IMPLEMENTATION(TalkImpl,
{,
  IMPLEMENTS(Say),
  IMPLEMENTS(PlaySound),
});

class TalkImpl : public TalkImplBase
{
  void execute(const Say& s) override
  {
    if(s.text != lastThingSaid)
    {
      SystemCall::say(s.text.c_str());
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
};

MAKE_SKILL_IMPLEMENTATION(TalkImpl);
