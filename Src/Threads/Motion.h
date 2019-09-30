/**
 * @file Threads/Motion.h
 *
 * This file declares the execution unit for the motion thread.
 *
 * @author Jan Fiedler
 */

#pragma once

#include "Tools/Framework/FrameExecutionUnit.h"

/**
 * @class Motion
 *
 * The execution unit for the motion thread.
 */
class Motion : public FrameExecutionUnit
{
public:
  bool beforeFrame() override;
  void afterModules() override;
  bool afterFrame() override;
};
