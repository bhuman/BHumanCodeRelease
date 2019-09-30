#include "Random.h"

#ifndef TARGET_ROBOT
#include <chrono>

std::mt19937& Random::getGenerator()
{
  static thread_local std::mt19937 generator(static_cast<unsigned int>(std::chrono::system_clock::now().time_since_epoch().count()));

  return generator;
}

#else
#include "Tools/ImageProcessing/SIMD.h"

unsigned int Random::HardwareGenerator::min() { return 0; }
unsigned int Random::HardwareGenerator::max() { return static_cast<unsigned int>(-1); }

unsigned int Random::HardwareGenerator::operator()() const
{
  unsigned int value;
  while(!_rdrand32_step(&value));
  return value;
}

void Random::HardwareGenerator::discard(unsigned long long z) const
{
  for(; z; z--)
    (*this)();
}

Random::HardwareGenerator& Random::getGenerator()
{
  static HardwareGenerator generator;
  return generator;
}
#endif
