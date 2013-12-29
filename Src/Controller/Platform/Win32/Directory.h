/**
* \file Controller/Platform/Win32/Directory.h
* Declares a platform dependend class for accessing directories.
* This is the Win32 implementation.
* \author Colin Graf
*/

#pragma once

#include <string>

/**
* A Class for accessing directories.
*/
class Directory
{
public:

  /**
  * Default constructor.
  */
  Directory();

  /**
  * Destructor.
  */
  ~Directory();

  /**
  * Opens a directory for searching files.
  * \param pattern A search pattern like "C:\\*.inf"
  * \return Whether the directory was opened successfully.
  */
  bool open(const std::string& pattern);

  /**
  * Searches the next matching entry in the opened directory.
  * \param name The name of the next matching entry.
  * \param isDir Whether the next entry is a directory.
  * \return true when a matching entry was found.
  */
  bool read(std::string& name, bool& isDir);

private:

  void* findFile; /**< Win32 FindFirstFile HANDLE */
  char ffd[320]; /**< Buffer for WIN32_FIND_DATA */
  bool bufferedEntry; /**< Whether there is a buffered search result in ffd. */
  std::string dirname; /**< The name of the directory. */
};
