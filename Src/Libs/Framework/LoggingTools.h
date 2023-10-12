/**
 * @file LoggingTools.h
 *
 * This file declares a namespace with functions that are useful for the handling of log files.
 *
 * @author Arne Böckmann
 * @author Arne Hasselbring
 */

#pragma once

#include "Streaming/Enum.h"
#include <string>

class In;
class Out;
struct Settings;

namespace LoggingTools
{
  ENUM(LogFileFormat,
  {,
    logFileUncompressed,
    logFileCompressed,
    logFileMessageIDs,
    logFileTypeInfo,
    logFileSettings,
    logFileIndices,
  });

  /**
   * Writes parts of the settings that are relevant for log files to a stream.
   * @param stream The stream to which to write.
   * @param settings The settings to write.
   */
  void writeSettings(Out& stream, const Settings& settings);

  /**
   * Reads parts of the settings that are relevant for log files from a stream.
   * @param stream The stream from which to read.
   * @param settings The settings to read.
   */
  void readSettings(In& stream, Settings& settings);

  /**
   * Skips over the logged settings in a stream.
   * @param stream The stream from which to read.
   */
  void skipSettings(In& stream);

  /**
   * Creates a log file name from lots of components.
   * @param headName The name of the robot's head on which the log is recorded. Must contain only letters.
   * @param bodyName The name of the robot's body on which the log is recorded. Must contain only letters.
   * @param scenario The scenario in which the log is recorded. Must contain only alphanumeric characters.
   * @param location The location in which the log is recorded. Must contain only alphanumeric characters.
   * @param identifier An identifier for the log. Must contain only alphanumeric characters, hyphens or single underscores.
   * @param playerNumber The player number of the recording robot. Must be nonnegative.
   * @param suffix A suffix at the end of the log file name (before the file ending). Must be a number surrounded by parentheses.
   * @return A log file name that can be parsed into its components.
   */
  std::string createName(const std::string& headName, const std::string& bodyName, const std::string& scenario,
                         const std::string& location, const std::string& identifier, int playerNumber,
                         const std::string& suffix = "");

  /**
   * Parses a log file name into its components.
   * All result parameters are pointers so that nullptr can be used to ignore a particular result.
   * @param logfileName The file name to parse.
   * @param headName The head name of the log.
   * @param bodyName The body name of the log.
   * @param scenario The scenario of the log.
   * @param location The location of the log.
   * @param identifier The identifier of the log.
   * @param playerNumber The player number of the log.
   * @param suffix The suffix of the log.
   */
  void parseName(const std::string& logfileName, std::string* headName, std::string* bodyName, std::string* scenario,
                 std::string* location, std::string* identifier, int* playerNumber, std::string* suffix = nullptr);
}
