/**
* @file HammingSoundTransmitter.h
* @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Communication/NoWirelessReceivedData.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/Infrastructure/DummyRepresentation.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include <queue>

MODULE(HammingSoundTransmitter,
{,
  USES(BehaviorStatus),
  REQUIRES(FrameInfo),
  REQUIRES(NoWirelessReceivedData),
  PROVIDES(DummyRepresentation),
  DEFINES_PARAMETERS(
  {,
    (int)(310) lengthOfSound,
  }),
});

class HammingSoundTransmitter : public HammingSoundTransmitterBase
{
public:
  HammingSoundTransmitter();
  void update(DummyRepresentation& dummyRepresentation);
  
private:
  std::array<bool, 4> switchSound;
  std::queue<int> soundQueue;
  unsigned lastTimeSoundPlayed = 0;

  size_t nextByteIndex;

  void reset();

  void playNextSound();
};
