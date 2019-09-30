/**
 * @file LoggingTools.h
 *
 * This file declares a namespace with functions that are useful for the handling of log files.
 *
 * @author Arne Böckmann
 * @author Arne Hasselbring
 */

#pragma once

#include "Tools/Streams/Enum.h"
#include <string>

namespace LoggingTools
{
  ENUM(LogFileFormat,
  {,
    logFileUncompressed,
    logFileCompressed,
    logFileMessageIDs,
    logFileTypeInfo,
  });

  /**
   * Creates a log file name from lots of components.
   * @param prefix A prefix at the beginning of the log file name (e.g. the name of the logged process).
   *               Can also contain a path. The part that is not a path must contain only letters or underscores.
   * @param headName The name of the robot's head on which the log is recorded. Must contain only letters.
   * @param bodyName The name of the robot's body on which the log is recorded. Must contain only letters.
   * @param scenario The scenario in which the log is recorded. Must contain only alphanumeric characters.
   * @param location The location in which the log is recorded. Must contain only alphanumeric characters.
   * @param identifier An identifier for the log. Must contain only alphanumeric characters, hyphens or single underscores.
   * @param playerNumber The player number of the recording robot. Must be nonnegative.
   * @param suffix A suffix at the end of the log file name (before the file ending). Must be a number surrounded by parentheses.
   * @return A log file name that can be parsed into its components.
   */
  std::string createName(const std::string& prefix, const std::string& headName, const std::string& bodyName,
                         const std::string& scenario, const std::string& location, const std::string& identifier,
                         int playerNumber, const std::string& suffix = "");

  /**
   * Parses a log file name into its components.
   * All result parameters are pointers so that nullptr can be used to ignore a particular result.
   * @param logfileName The file name to parse.
   * @param prefix The prefix of the log.
   * @param headName The head name of the log.
   * @param bodyName The body name of the log.
   * @param scenario The scenario of the log.
   * @param location The location of the log.
   * @param identifier The identifier of the log.
   * @param playerNumber The player number of the log.
   * @param suffix The suffix of the log.
   */
  void parseName(const std::string& logfileName, std::string* prefix, std::string* headName, std::string* bodyName,
                 std::string* scenario, std::string* location, std::string* identifier, int* playerNumber, std::string* suffix = nullptr);
}
