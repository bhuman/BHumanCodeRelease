/**
 * @file KeyStates.h
 *
 * Declaration of struct KeyStates
 *
 * @author unknown
 * @author Jesse Richter-Klug
 */

#pragma once

#include "Tools/Streams/EnumIndexedArray.h"
#include "Tools/Function.h"

/**
 * The struct represents the states of the keys.
 */
STREAMABLE(KeyStates,
{
  ENUM(Key,
  {,
    // touch sensors:
    headFront,
    headMiddle,
    headRear,
    lHandBack,
    lHandLeft,
    lHandRight,
    rHandBack,
    rHandLeft,
    rHandRight,

    // bumpers:
    lFootLeft,
    lFootRight,
    rFootLeft,
    rFootRight,
    chest,
  });

  KeyStates(),

  (ENUM_INDEXED_ARRAY(bool, Key)) pressed,
});

inline KeyStates::KeyStates()
{
  pressed.fill(false);
}

STREAMABLE_WITH_BASE(EnhancedKeyStates, KeyStates,
{
  /*
   * calculates the 'current' hit streak for a given key state sensor
   *
   * @param key the sensor
   * @param timeOut is if the sensor is triggered longer than this, the trigger state gets rejected
   * @param allowedTimeBetweenHitStreak allowed time between to hits to be considered as streak
   * @param allowedButtonStruggleTime allowed time between to hits to be NOT considered as two seperate hits
   *
   * @return the size of the current finished hit streak recently
   */
  FUNCTION(unsigned(KeyStates::Key key, unsigned timeOut, unsigned allowedTimeBetweenHitStreak)) getHitStreakFor;
  FUNCTION(unsigned(KeyStates::Key key, unsigned timeOut, unsigned allowedTimeBetweenHitStreak, unsigned allowedButtonStruggleTime)) getStruggleHitStreakFor;
  ,
  (ENUM_INDEXED_ARRAY(unsigned, Key)) hitStreak,       ///< hit streak per key calculated with default parameters of the provider (finished recently)
  (ENUM_INDEXED_ARRAY(unsigned, Key)) pressedDuration, ///< duration per key since this one is pressed
                                                       ///<     (0 == pressed[key] for the first time this frame || !pressed[key]) -> True
});
