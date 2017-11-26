/**
 * Plays a sound if this option was not called in the previous cycle.
 * @param name The name of the sound file.
 */
option(PlaySamples, (std::vector<short>&) samples)
{
  initial_state(playSamples)
  {
    transition
    {
      if(state_time)
        goto waitForNewSound;
    }
    action
    {
      SystemCall::playSamples(samples);
      lastSamplesPlayed = &samples;
    }
  }

  target_state(waitForNewSound)
  {
    transition
    {
      if(&samples != lastSamplesPlayed)
        goto playSamples;
    }
  }
}

std::vector<short>* lastSamplesPlayed = nullptr;