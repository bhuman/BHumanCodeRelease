#pragma once

#include <cstddef>

namespace Memory
{
  /** Allocate memory of given size with given alignment. */
  void* alignedMalloc(size_t size, size_t alignment = 32);

  /** Free aligned memory. */
  void alignedFree(void* ptr);
};
