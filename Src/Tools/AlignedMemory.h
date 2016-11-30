/**
 * @file Tools/AlignedMemory.h
 *
 * Base class for all types that need aligned allocations. This is required
 * whenever they contain Eigen attributes.
 *
 * @author Thomas Röfer
 */

#pragma once

#include <new>

struct AlignedMemory
{
  void* operator new(std::size_t size);
  void* operator new[](std::size_t size);
  void operator delete(void* ptr) throw();
  void operator delete[](void* ptr) throw();
  static void* operator new(std::size_t size, void* ptr) {return ::operator new(size, ptr);}
  static void* operator new[](std::size_t size, void* ptr) {return ::operator new[](size, ptr);}
  void operator delete(void* memory, void* ptr) throw() {return ::operator delete(memory, ptr);}
  void operator delete[](void* memory, void* ptr) throw() {return ::operator delete[](memory, ptr);}
  void* operator new(std::size_t size, const std::nothrow_t&) throw();
  void operator delete(void* ptr, const std::nothrow_t&) throw();
};
