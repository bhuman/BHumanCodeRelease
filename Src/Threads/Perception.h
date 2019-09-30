/**
 * @file Threads/Perception.h
 *
 * This file declares the execution unit for threads that handle the perception.
 *
 * @author Jan Fiedler
 */

#pragma once

#include "Tools/Framework/FrameExecutionUnit.h"

/**
 * @class Perception
 *
 * The execution unit for a perception thread.
 */
class Perception : public FrameExecutionUnit
{
public:
  bool beforeFrame() override;
  void beforeModules() override;
  bool afterFrame() override;
};
