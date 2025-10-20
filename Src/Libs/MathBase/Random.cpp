#include "Random.h"

#if !defined TARGET_ROBOT || !defined __x86_64__
#include <chrono>

std::mt19937& Random::getGenerator()
{
  static thread_local std::mt19937 generator(static_cast<unsigned int>(std::chrono::system_clock::now().time_since_epoch().count()));

  return generator;
}

#else
#include <immintrin.h>

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
