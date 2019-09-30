/**
 * KeyStateEnhancer.h
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Tools/RingBuffer.h"
#include "Tools/Range.h"
#include "Tools/Module/Module.h"

#include <array>

MODULE(KeyStateEnhancer,
{,
  REQUIRES(KeyStates),
  REQUIRES(FrameInfo),

  PROVIDES(EnhancedKeyStates),
  DEFINES_PARAMETERS(
  {,
    // default values used for EnhancedKeyStates::hitStreak
    (unsigned)(1000u) releaseTimeOut,   ///< if the sensor is triggered longer than this the trigger state gets rejected
    (unsigned)(200u) successiveTimeOut, ///< allowed time between two hits to be considered as streak
    (unsigned)(0) buttonStruggleTime,   ///< allowed time between two hits to be NOT considered as two separate hits
  }),
});

class KeyStateEnhancer : public KeyStateEnhancerBase
{
  void update(EnhancedKeyStates& enhancedKeyStates) override;

  unsigned getHitStreakOf(KeyStates::Key key, bool keyState, unsigned timeOut, unsigned allowedTimeBetweenHitStreak = 0, unsigned allowedButtonStruggleTime = 0);

  std::array<RingBuffer<Range<unsigned>, 10>, KeyStates::Key::numOfKeys> rangeBuffer;
  RingBuffer<FrameInfo, 2> lastTimes;
};
