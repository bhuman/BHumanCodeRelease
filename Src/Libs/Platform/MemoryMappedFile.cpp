/**
 * @file MemoryMappedFile.cpp
 *
 * This file implements a class that represents read-only memory mapped files.
 *
 * @author Thomas Röfer
 */

#include "MemoryMappedFile.h"
#include "BHAssert.h"
#include "File.h"
#include <cstdio>
#ifdef WINDOWS
#include <windows.h>
#include <io.h>
#else
#include <sys/mman.h>
#endif

MemoryMappedFile::MemoryMappedFile(const std::string& filename)
{
  File file(filename, "rb");
  if(file.exists())
  {
    size = file.getSize();
#ifdef WINDOWS
    handle = CreateFileMapping(reinterpret_cast<HANDLE>(_get_osfhandle(_fileno(static_cast<FILE*>(file.getNativeFile())))),
                               nullptr, PAGE_READONLY,
                               static_cast<DWORD>(size >> 32), static_cast<DWORD>(size),
                               nullptr);
    ASSERT(handle);
    data = static_cast<char*>(MapViewOfFile(handle, FILE_MAP_READ, 0, 0, size));
#else
    data = static_cast<char*>(mmap(nullptr, size, PROT_READ, MAP_SHARED, fileno(static_cast<FILE*>(file.getNativeFile())), 0));
#endif
    ASSERT(data);
  }
}

MemoryMappedFile::~MemoryMappedFile()
{
  if(data)
#ifdef WINDOWS
  {
    VERIFY(UnmapViewOfFile(data));
    CloseHandle(handle);
  }
#else
    VERIFY(munmap(data, size) != -1);
#endif
}
