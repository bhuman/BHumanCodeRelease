/**
 * @file MemoryMappedFile.h
 *
 * This file declares a class that represents read-only memory mapped files.
 *
 * @author Thomas Röfer
 */

#pragma once

#include <string>

class MemoryMappedFile
{
  char* data = nullptr; /**< The start address of the mapped file in memory. */
  size_t size = 0; /**< The size of the file and the memory block. */
#ifdef WINDOWS
  void* handle = nullptr; /**< Windows also needs a file handle for the mapping. */
#endif

public:
  /**
   * Open a file and map it to a memory block.
   * @param filename The name of the file.
   */
  MemoryMappedFile(const std::string& filename);

  /** Destructor. */
  ~MemoryMappedFile();

  /**
   * Does the file exist?
   * @return Does it exist?
   */
  bool exists() const {return data != nullptr;}

  /**
   * Returns the begin of the memory block the file is mapped to.
   * @return The address of the memory block or \c nullptr
   *         if the file does not exist.
   */
  const char* getData() {return data;}

  /**
   * Returns the size of the file and the memory block.
   * @return The size in bytes or 0 if the file does not exist.
   */
  size_t getSize() {return size;}
};
