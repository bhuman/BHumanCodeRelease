/**
 * @file Controller/LogExtractor.h
 *
 * Definition of class LogExtractor
 *
 * @author Jan Fiedler
 */

#pragma once

#include "Tools/MessageQueue/MessageIDs.h"

#include <functional>
#include <map>
#include <string>

class LogPlayer;
class Out;
class Streamable;
struct TypeInfo;

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
   * Writes all messages in the log player queue to a log file.
   * @param fileName the name of the file to write
   * @param typeInfo Type information of logged data types. Will be ignored if
   *                      it is a nullptr or logger already has type information.
   * @return Whether the writing was successful
   */
  bool save(const std::string& fileName, const TypeInfo* typeInfo);

  /**
   * Splits and saves the log in the given amount of Files
   * @param fileName the name of the file to write
   * @param typeInfo Type information of logged data types. Will be ignored if
   *                      it is a nullptr or logger already has type information.
   * @param split the number of files to split into
   * @return returns if splitting was succesful
   */
  bool split(const std::string& fileName, const TypeInfo* typeInfo, const int& split);

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
   * Writes all inertial sensor data from the log into a semicolon-separated
   * dataset file.
   * @param path The path of the file to which the data is written.
   * @return whether writing the file was successful or not
   */
  bool saveInertialSensorData(const std::string& path);

  /**
   * Writes all joint angle and request data from the log into a semicolon-separated
   * dataset file.
   * @param path The path of the file to which the data is written.
   * @return whether writing the file was successful or not
   */
  bool saveJointAngleData(const std::string& path);

  /**
   * Writes a csv with all module timings.
   * @return true if writing was successful
   */
  bool writeTimingData(const std::string& fileName);

  /**
   * Saves images of ballSpots and related metadata according to imported labels
   * @param path The path of the file to which the data is written. In addition,
   *             a directory with the same name but without extension will be
   *             created that contains the images.
   * @return true if saving was successful
   */
  bool saveLabeledBallSpots(const std::string& path);

private:
  /**
   * The method creates a new folder with logname in the current logfolder, replacing the prefix.
   * @param prefix The prefix.
   * @return The path of the folder.
   */
  std::string createNewFolder(const std::string& prefix) const;

  /**
   * The method writes a csv.
   * @param fileName The short file name.
   * @param writeHeader Append the head to the file.
   * @param representations Map with all needed Representations.
   * @param writeInFile Addend one frame to the file.
   * @param noEndl If x lines are written in each fram, do not append an endl.
   *
   * @param file The file to append to.
   * @param sep The separator.
   * @return whether writing the file was successful or not.
   */
  bool saveCSV(const std::string& fileName, const std::function<void(Out& file, const std::string& sep)>& writeHeader, const std::map<const MessageID, Streamable*>& representations, const std::function<void(Out& file, const std::string& sep)>& writeInFile, const bool noEndl = false);

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
