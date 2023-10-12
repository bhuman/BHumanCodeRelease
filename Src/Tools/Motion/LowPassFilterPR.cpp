/**
 * @file LowPassFilterPR.h
 * @author Philip Reichenberg
 */

#include "LowPassFilterPR.h"

void LowPassFilterPR::update(const float value)
{
  buffer.push_front(value);
  if(!buffer.full())
    currentValue = currentValue * lowPassFactor + value * (1.f - lowPassFactor);
  else
  {
    int counterSignSwitch = 0;
    std::size_t currentIndex = buffer.capacity() - 1;
    float newValue = buffer[currentIndex];
    currentIndex--;
    float nextValue = buffer[currentIndex];
    float lastSign = 0;
    while(currentIndex < buffer.capacity())
    {
      const float newSign = nextValue - currentValue;
      counterSignSwitch += newSign * lastSign < 0 ? 1 : 0;
      if(counterSignSwitch < 2 && currentIndex != 0 && newSign * lastSign >= 0)
        newValue = nextValue * fastFactor + currentValue * (1.f - fastFactor);
      else if(counterSignSwitch < 2)
        newValue = nextValue * lowPassFactor + newValue * (1.f - lowPassFactor);
      else
      {
        const float useLowwPassFactor = std::min(lowPassFactor, (1.f - lowPassFactor));
        newValue = nextValue * useLowwPassFactor + currentValue * (1.f - useLowwPassFactor);
      }

      currentValue = newValue;

      currentIndex--;
      lastSign = newSign;
      nextValue = buffer[currentIndex];
    }
  }
}

void LowPassFilterPR::clear()
{
  buffer.clear();
}
