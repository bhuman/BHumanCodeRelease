/**
 * @file LowPassFilterPR.h
 * @author Philip Reichenberg
 */

#include "LowPassFilterPR.h"

void LowPassFilterPR::update(const float value)
{
  buffer.push_front(value);
  if(!buffer.full())
  {
    lastFirstFilteredValue = firstFilteredValue = currentValue;
    currentValue = currentValue * (1.f - lowPassFactor) + value * lowPassFactor;
  }
  else
  {
    int counterSignSwitch = 0;
    std::size_t currentIndex = buffer.capacity() - 1;
    float newValue = buffer[currentIndex];
    currentIndex--;
    float nextValue = buffer[currentIndex];
    float lastSign = firstFilteredValue - lastFirstFilteredValue;
    currentValue = firstFilteredValue; // reset back to value we had, when we updated our value based on our oldest sample
    lastFirstFilteredValue = firstFilteredValue;
    while(currentIndex < buffer.capacity()) // index is an unsigned and will become bigger than capacity when reducing it after being already at 0
    {
      const float newSign = nextValue - currentValue;
      counterSignSwitch += newSign * lastSign < 0 ? 1 : 0;
      if(counterSignSwitch < 2 && newSign * lastSign >= 0) // value changed in the same direction as before -> believe it more
        newValue = nextValue * fastFactor + currentValue * (1.f - fastFactor);
      else if(counterSignSwitch < 2) // value changed the direction -> do not believe it
        newValue = nextValue * lowPassFactor + newValue * (1.f - lowPassFactor);
      else
      {
        const float useLowwPassFactor = std::min(lowPassFactor, (1.f - lowPassFactor));
        newValue = nextValue * useLowwPassFactor + currentValue * (1.f - useLowwPassFactor);
      }

      currentValue = newValue;
      if(currentIndex == buffer.capacity() - 2 && buffer.capacity() >= 2)
        firstFilteredValue = newValue;

      currentIndex--;
      lastSign = newSign;
      nextValue = buffer[currentIndex];
    }
  }
}

void LowPassFilterPR::clear()
{
  buffer.clear();
  lastFirstFilteredValue = 0;
  firstFilteredValue = 0;
  currentValue = 0;
}
