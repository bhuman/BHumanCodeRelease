/**
 * @file Threads/Cognition2D.h
 *
 * This file declares the execution unit for the cognition thread.
 *
 * @author Jan Fiedler
 * @author Arne Hasselbring
 */

#pragma once

#include "Framework/FrameExecutionUnit.h"

/**
 * @class Cognition
 *
 * The execution unit for the cognition thread.
 */
class Cognition2D : public FrameExecutionUnit
{
public:

  bool beforeFrame() override;
  void afterModules() override;
};
