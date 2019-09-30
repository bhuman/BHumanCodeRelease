#include "Platform/Memory.h"

#include <Windows.h>

void* Memory::alignedMalloc(size_t size, size_t alignment)
{
  return _aligned_malloc(size, alignment);
}

void Memory::alignedFree(void* ptr)
{
  _aligned_free(ptr);
}
