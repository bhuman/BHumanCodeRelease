/**
 * @file Threads/Motion.h
 *
 * This file declares the execution unit for the motion thread.
 *
 * @author Jan Fiedler
 */

#pragma once

#include "Tools/Framework/BHExecutionUnit.h"

/**
 * @class Motion
 *
 * The execution unit for the motion thread.
 */
class Motion : public BHExecutionUnit
{
public:
  bool beforeFrame() override;
  void afterModules() override;
  bool afterFrame() override;
};
