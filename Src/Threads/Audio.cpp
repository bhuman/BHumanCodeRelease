/**
 * @file Threads/Audio.cpp
 *
 * This file implements the execution unit for the audio thread.
 *
 * @author Jan Fiedler
 * @author Thomas Röfer
 */

#include "Audio.h"
#include "Modules/Infrastructure/LogDataProvider/LogDataProvider.h"

REGISTER_EXECUTION_UNIT(Audio)

bool Audio::beforeFrame()
{
  return LogDataProvider::isFrameDataComplete();
}
