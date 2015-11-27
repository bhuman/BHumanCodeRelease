/**
 * @file Tools/AlignedMemory.cpp
 *
 * Base class for all types that need aligned allocations. This is required
 * whenever they contain Eigen attributes.
 *
 * @author Thomas Röfer
 */

#include "AlignedMemory.h"
#include "Platform/SystemCall.h"

void* AlignedMemory::operator new(std::size_t size)
{
  return SystemCall::alignedMalloc(size);
}

void* AlignedMemory::operator new[](std::size_t size)
{
  return SystemCall::alignedMalloc(size);
}

void AlignedMemory::operator delete(void* ptr) throw()
{
  SystemCall::alignedFree(ptr);
}

void AlignedMemory::operator delete[](void* ptr) throw()
{
  SystemCall::alignedFree(ptr);
}

void* AlignedMemory::operator new(std::size_t size, const std::nothrow_t&) throw()
{
  return SystemCall::alignedMalloc(size);
}

void AlignedMemory::operator delete(void* ptr, const std::nothrow_t&) throw()
{
  SystemCall::alignedFree(ptr);
}
