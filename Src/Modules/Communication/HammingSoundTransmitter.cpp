/**
* @file HammingSoundTransmitter.cpp
* @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
*/

#include "HammingSoundTransmitter.h"
#include "Platform/SystemCall.h"
#include "Tools/Debugging/Modify.h"
#include "Tools/HammingEncodingBase4.h"
#include "Tools/Math/Random.h"
#include <limits>

MAKE_MODULE(HammingSoundTransmitter, communication)

HammingSoundTransmitter::HammingSoundTransmitter()
{
  switchSound[0] = switchSound[1] = switchSound[2] = switchSound[3] = true;
  reset();
}

void HammingSoundTransmitter::reset()
{
  while(!soundQueue.empty())
    soundQueue.pop();
  nextByteIndex = theNoWirelessReceivedData.data.size();
}

void HammingSoundTransmitter::update(DummyRepresentation& dummyRepresentation)
{
  if(theNoWirelessReceivedData.lastReceived == theFrameInfo.time)
    nextByteIndex = 0;

  if(theBehaviorStatus.activity == BehaviorStatus::noWifi)
  {
    reset();
    return;
  }

  if(nextByteIndex < theNoWirelessReceivedData.data.size())
  {
    const unsigned char byte = theNoWirelessReceivedData.data[nextByteIndex++];

    std::array<unsigned char, 4> quaternary;
    HammingEncodingBase4::toBase4Inverse(byte, quaternary);

    std::array<unsigned char, 8> hamming;
    HammingEncodingBase4::encodeHamming84(quaternary, hamming);

    OUTPUT_TEXT("Sending value: " << byte
                  << " Hamming: " << hamming[0] << hamming[1] << hamming[2]
                                  << hamming[3] << hamming[4] << hamming[5]
                                  << hamming[6] << hamming[7]);

    for(int i = 0; i < 8; ++i)
      soundQueue.push(hamming[i]);
  }

  playNextSound();
}

void  HammingSoundTransmitter::playNextSound()
{
  if(soundQueue.empty() || theFrameInfo.getTimeSince(lastTimeSoundPlayed) < lengthOfSound)
    return;

  lastTimeSoundPlayed = theFrameInfo.time;

  const int value = soundQueue.front();
  soundQueue.pop();

  int sound = value * 2;
  switchSound[value] = !switchSound[value];
  if(switchSound[value])
    sound += 1;

  switch(sound)
  {
    case 0: SystemCall::playSound("NoWiFi/0-1750.wav"); break;
    case 1: SystemCall::playSound("NoWiFi/0-2000.wav"); break;
    case 2: SystemCall::playSound("NoWiFi/1-2250.wav"); break;
    case 3: SystemCall::playSound("NoWiFi/1-2500.wav"); break;
    case 4: SystemCall::playSound("NoWiFi/2-2750.wav"); break;
    case 5: SystemCall::playSound("NoWiFi/2-3000.wav"); break;
    case 6: SystemCall::playSound("NoWiFi/3-3250.wav"); break;
    case 7: SystemCall::playSound("NoWiFi/3-3500.wav"); break;
    default: ASSERT(false);
  }

  OUTPUT_TEXT("Sending Sound: " << sound);
}
