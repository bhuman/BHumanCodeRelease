/**
 * @file Talk.cpp
 *
 * This file implements the Say and PlaySound skills.
 *
 * @author Arne Hasselbring
 * @author Jan Blumenkamp
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) PlaySound,
       args((const std::string&) name,
            (bool) force),
       vars((std::string)("") lastSoundPlayed)) /**< The name of the last sound file played. */
{
  initial_state(execute)
  {
    transition
    {
      if(name == lastSoundPlayed)
        goto done;
    }
    action
    {
      SystemCall::playSound(name.c_str(), force);
      lastSoundPlayed = name;
    }
  }

  target_state(done)
  {
    transition
    {
      if(name != lastSoundPlayed)
        goto execute;
    }
  }
}

option((SkillBehaviorControl) Say,
       args((const std::string&) text,
            (bool) force,
            (float) speed),
       vars((std::string)("") lastTextSaid)) /**< The last text that was spoken. */
{
  initial_state(execute)
  {
    transition
    {
      if(text == lastTextSaid)
        goto done;
    }
    action
    {
      SystemCall::say(text.c_str(), force, speed);
      lastTextSaid = text;
    }
  }

  target_state(done)
  {
    transition
    {
      if(text != lastTextSaid)
        goto execute;
    }
  }
}
