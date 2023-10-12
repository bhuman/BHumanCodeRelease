/**
 * @file LogExtractor.h
 *
 * Definition of class LogExtractor
 *
 * @author Jan Fiedler
 */

#pragma once

#include "Streaming/MessageIDs.h"

#include <functional>
#include <map>
#include <string>

class LogPlayer;
class Streamable;

/**
 * @class LogExtractor
 *
 * This class implements functions that extract things from a log.
 * Export to the folder of the log.
 */
class LogExtractor
{
private:
  LogPlayer& logPlayer;

public:
  /**
   * @param logPlayer The The LogPlayer being worked on.
   */
  LogExtractor(LogPlayer& logPlayer);

  /**
   * Writes all audio data in the log player queue to a single wav file.
   * @param fileName the name of the file to write
   * @return if the writing was successful
   */
  bool saveAudioFile(const std::string& fileName);

  /**
   * Writes all images in the log player queue to a bunch of image files (.png).
   * @param path The path of the directory in which the images are created.
   * @param raw Save color unconverted
   * @param onlyPlaying Only save images from an upright, playing robot
   * @param takeEachNthFrame Save each Nth frame (one set of upper and lower image is considered as one frame)
   * @return if writing all files was successful
   */
  bool saveImages(const std::string& path, bool raw, bool onlyPlaying, int takeEachNthFrame);

  /**
   * Analyze if the measured joint angles are jumping, which indicates defect sensors.
   * @return true if analyzing was successful
   */
  bool analyzeRobotStatus();

private:
  /**
   * Go through the log and execute an action after each frame.
   * @param representations Map with all needed Representations.
   * @param executeAction The action that is executed after each frame.
   *
   * @param frameType Type of the frame.
   * @return whether the action was successful or not.
   */
  bool goThroughLog(const std::map<const MessageID, Streamable*>& representations, const std::function<bool(const std::string& frameType)>& executeAction);
};
