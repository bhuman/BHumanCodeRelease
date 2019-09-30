#include "Platform/Memory.h"

#include <cstdlib>

void* Memory::alignedMalloc(size_t size, size_t alignment)
{
  void* ptr;
  if(!posix_memalign(&ptr, alignment, size))
    return ptr;
  else
    return nullptr;
}

void Memory::alignedFree(void* ptr)
{
  free(ptr);
}
