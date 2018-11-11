#include "Random.h"
#include <chrono>

std::mt19937& Random::getGenerator()
{
  static thread_local std::mt19937 generator(static_cast<unsigned int>(std::chrono::system_clock::now().time_since_epoch().count()));

  return generator;
}
