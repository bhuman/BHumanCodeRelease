#include "Platform/Memory.h"

#ifdef WINDOWS
#include <Windows.h>
#else
#include <cstdlib>
#endif

void* Memory::alignedMalloc(size_t size, size_t alignment)
{
#ifdef WINDOWS
  return _aligned_malloc(size, alignment);
#else
  void* ptr;
  if(!posix_memalign(&ptr, alignment, size))
    return ptr;
  else
    return nullptr;
#endif
}

void Memory::alignedFree(void* ptr)
{
#ifdef WINDOWS
  _aligned_free(ptr);
#else
  free(ptr);
#endif
}
