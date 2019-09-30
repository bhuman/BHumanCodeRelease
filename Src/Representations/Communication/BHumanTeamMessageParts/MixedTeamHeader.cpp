/**
 * @file MixedTeamHeader.cpp
 *
 * This file implements methods of the Mixed Team header.
 *
 * @author Arne Hasselbring
 */

#include "MixedTeamHeader.h"
#include <cstdint>

template<typename T>
inline void writeVal(void*& data, T value)
{
  *reinterpret_cast<T*&>(data)++ = value;
}

template<typename T>
inline T readVal(const void*& data)
{
  return *reinterpret_cast<T*&>(data)++;
}

int MixedTeamHeader::sizeOfMixedTeamHeader()
{
  return 2;
}

void MixedTeamHeader::write(void* data) const
{
  uint8_t flagContainer = 0;
  (flagContainer <<= 1) |= wantsToPlayBall ? 1 : 0;
  (flagContainer <<= 1) |= isPenalized ? 1 : 0;
  writeVal<uint8_t>(data, flagContainer);
  writeVal<uint8_t>(data, someBehaviorNumber < 0 ? 0xff : static_cast<uint8_t>(std::min(0xfe, someBehaviorNumber)));
}

bool MixedTeamHeader::read(const void* data)
{
  const uint8_t flagContainer = readVal<const uint8_t>(data);
  isPenalized = (flagContainer & 0x01) != 0;
  wantsToPlayBall = (flagContainer & 0x02) != 0;
  someBehaviorNumber = readVal<const uint8_t>(data);
  if(someBehaviorNumber == 0xff)
    someBehaviorNumber = -1;
  return true;
}
