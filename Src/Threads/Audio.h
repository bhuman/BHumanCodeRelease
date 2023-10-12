/**
 * @file Threads/Audio.h
 *
 * This file declares the execution unit for the audio thread.
 *
 * @author Jan Fiedler
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/Framework/BHExecutionUnit.h"

/**
 * @class Audio
 *
 * The execution unit for the audio thread.
 */
class Audio : public BHExecutionUnit
{
public:
  bool beforeFrame() override;
  bool afterFrame() override;
};
