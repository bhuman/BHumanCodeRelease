/**
 * KeyStateEnhancer.cpp
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "KeyStateEnhancer.h"

MAKE_MODULE(KeyStateEnhancer, sensing);

void KeyStateEnhancer::update(EnhancedKeyStates& enhancedKeyStates)
{
  enhancedKeyStates.getHitStreakFor = [&enhancedKeyStates, this](KeyStates::Key key, unsigned timeOut, unsigned allowedTimeBetweenHitStreak) -> unsigned
  {
    return getHitStreakOf(key, enhancedKeyStates.pressed[key], timeOut, allowedTimeBetweenHitStreak);
  };

  enhancedKeyStates.isPressedFor = FunctionImpl::Function<bool(KeyStates::Key, unsigned)>();
  enhancedKeyStates.isPressedFor = [&enhancedKeyStates, lastEnhancedKeyStates = enhancedKeyStates](KeyStates::Key key, unsigned duration) -> bool
  {
    return enhancedKeyStates.pressedDuration[key] >= duration && lastEnhancedKeyStates.pressedDuration[key] < duration;
  };

  lastTimes.push_front(theFrameInfo);

  FOREACH_ENUM(KeyStates::Key, i)
  {
    if(theKeyStates.pressed[i])
    {
      if(enhancedKeyStates.pressed[i] != theKeyStates.pressed[i] || rangeBuffer[i].empty())
        rangeBuffer[i].push_front(Range<unsigned>(theFrameInfo.time));
      else
        rangeBuffer[i].front().add(theFrameInfo.time);

      enhancedKeyStates.pressedDuration[i] = rangeBuffer[i].front().getSize();
    }
    else
      enhancedKeyStates.pressedDuration[i] = 0;

    enhancedKeyStates.hitStreak[i] = getHitStreakOf(i, theKeyStates.pressed[i], releaseTimeOut, successiveTimeOut);
  }

  static_cast<KeyStates&>(enhancedKeyStates) = theKeyStates;
}

unsigned KeyStateEnhancer::getHitStreakOf(KeyStates::Key key, bool keyState, unsigned timeOut, unsigned allowedTimeBetweenHitStreak)
{
  if(keyState || rangeBuffer[key].empty() // if keyState (pressed) the streak is not finished
    || theFrameInfo.getTimeSince(rangeBuffer[key].front().max) <= static_cast<int>(allowedTimeBetweenHitStreak) // the streak might not be finished so we do not announce
    || (lastTimes.full() && (lastTimes.back().getTimeSince(rangeBuffer[key].front().max) > static_cast<int>(allowedTimeBetweenHitStreak)))) // there might have been one but we do not longer announce
    return 0;

  unsigned hitStreak = 0;
  for(auto iter = rangeBuffer[key].begin(); iter != rangeBuffer[key].end() && iter->getSize() < timeOut; ++iter, ++hitStreak) // break if we at the end || a button is pressed to long
    if(*iter != rangeBuffer[key].front() && (iter - 1)->min - iter->max > allowedTimeBetweenHitStreak)
        break;                                                                                                                // the time between two hits exceeds the allowed one

  return hitStreak;
}
