/**
 * @file MofCompiler.h
 *
 * This file declares a single function to compile the motion net for special actions.
 *
 * @author Uwe Düffert
 * @author Martin Lötzsch
 */

#pragma once

#include "Representations/MotionControl/SpecialActionRequest.h"

#include <vector>

class MofCompiler
{
private:
  static const int MAXLAB = 5000;
  int numOfLabels = 0;
  char* label_motion[MAXLAB];
  char* label_name[MAXLAB];
  short label_number[MAXLAB];

  static const int MAXLIN = 32000;
  int numOfLines = 0;
  char motion[512];
  int actMotionID = -1;
  char* line_data[MAXLIN];
  short line_number[MAXLIN];
  short line_motionID[MAXLIN];

  static const int MAXFIL = 500;
  int numOfFiles = 0;
  char* file_name[MAXFIL];
  short file_startindex[MAXFIL];

  int jumpTable[SpecialActionRequest::numOfSpecialActionIDs];

  char* printBuffer;
  size_t printBufferSize;

public:
  ~MofCompiler();

  /**
   * The function compiles all mofs.
   * @param buffer A buffer that receives any error message output. It may contain
   *               several lines separated by \n.
   * @param size The length of the buffer.
   * @return Success of compilation.
   */
  bool compileMofs(char* buffer, size_t size, std::vector<float>& motionData);

private:
  /**
   * The function replaces printf so that the output is written into a buffer.
   * @param format Normal printf format string.
   * @return Normal printf return value.
   */
  int myprintf(const char* format, ...);

  /**
   * Generate MotionNetData.cpp
   * @return True if successful
   */
  bool generateMotionNet(std::vector<float>& motionData);

  /**
   * Parse all mof files except extern.mof to generate motion net
   * @return True if successful
   */
  bool parseMofs();

  /**
   * Parse extern.mof to generate jump table from extern into motion net
   * @return True if successful
   */
  bool parseExternMof();
};
