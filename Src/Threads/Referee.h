/**
 * @file Referee.h
 *
 * This file declares a thread that handles the referee pose detection.
 *
 * @author Ayleen Lührsen
 */

#pragma once

#include "Tools/Framework/BHExecutionUnit.h"

class Referee : public BHExecutionUnit
{
public:
  bool beforeFrame() override;
  bool afterFrame() override;
};
