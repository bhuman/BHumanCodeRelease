/**
 * Plays a sound if this option was not called in the previous cycle.
 * @param name The name of the sound file.
 */
option(PlaySound, (const std::string&) name)
{
  initial_state(playSound)
  {
    transition
    {
      if(state_time)
        goto waitForNewSound;
    }
    action
    {
      SystemCall::playSound(name.c_str());
      lastSoundPlayed = name;
    }
  }

  target_state(waitForNewSound)
  {
    transition
    {
      if(name != lastSoundPlayed)
        goto playSound;
    }
  }
}

std::string lastSoundPlayed = "";
