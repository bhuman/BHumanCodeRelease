/**
 * @file Threads/Perception.h
 *
 * This file declares the execution unit for threads that handle the perception.
 *
 * @author Jan Fiedler
 */

#pragma once

#include "Tools/Framework/BHExecutionUnit.h"

/**
 * @class Perception
 *
 * The execution unit for a perception thread.
 */
class Perception : public BHExecutionUnit
{
public:
  bool beforeFrame() override;
  void beforeModules() override;
  bool afterFrame() override;
};
