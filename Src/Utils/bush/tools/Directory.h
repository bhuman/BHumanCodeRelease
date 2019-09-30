/**
 * \file Directory.h
 * Declares a class for accessing directories.
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
   * Destructor.
   */
  ~Directory();

  /**
   * Opens a directory for searching files.
   * \param pattern A search pattern like "/etc/a*.ini"
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

  void* dp = nullptr; /**< Directory descriptor. */
  std::string dirname; /**< The name of the directory. */
  std::string filepattern; /**< The pattern for file name matching (e.g. "*.dll"). */
};
