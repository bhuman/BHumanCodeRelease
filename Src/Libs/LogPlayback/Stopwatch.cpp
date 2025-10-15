/**
 * @file Stopwatch.cpp
 *
 * This file implements a class that wraps \c idStopwatch messages.
 *
 * @author Thomas Röfer
 */

#include "Stopwatch.h"

void Stopwatch::read(In& stream)
{
  namesToIndices.clear();
  durations.clear();
  unsigned short size = 0;
  unsigned short index = 0;
  std::string name;
  stream >> size;
  namesToIndices.clear();
  while(size--)
  {
    stream >> index >> name;
    namesToIndices[name] = index;
  }
  stream >> size;
  durations.resize(size);
  unsigned duration = 0;
  while(size--)
  {
    stream >> index >> duration;
    ASSERT(index < durations.size());
    durations[index] = duration;
  }
  stream >> frameStartTime >> frameNumber;
}

void Stopwatch::write(Out& stream) const
{
  stream << static_cast<unsigned short>(namesToIndices.size());
  for(const auto& nameToIndex : namesToIndices)
    stream << nameToIndex.second << nameToIndex.first;
  stream << static_cast<unsigned short>(durations.size());
  for(size_t i = 0; i < durations.size(); ++i)
    stream << static_cast<unsigned short>(i) << durations[i];
  stream << frameStartTime << frameNumber;
}

void reg()
{
  REG_CLASS(Stopwatch);
  // The members cannot be registered.
}
