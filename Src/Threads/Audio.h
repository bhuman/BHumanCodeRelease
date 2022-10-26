/**
 * @file Threads/Audio.h
 *
 * This file declares the execution unit for the audio thread.
 *
 * @author Jan Fiedler
 * @author Thomas Röfer
 */

#pragma once

#include "Framework/FrameExecutionUnit.h"

/**
 * @class Audio
 *
 * The execution unit for the audio thread.
 */
class Audio : public FrameExecutionUnit
{
public:
  bool beforeFrame() override;
  bool afterFrame() override;
};
