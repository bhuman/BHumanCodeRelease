/**
 * @file Tools/Framework/FrameExecutionUnit.cpp
 *
 * This file declares a frame execution unit, created from a list of execution unit creators.
 *
 * @author Jan Fiedler
 */

#include "FrameExecutionUnit.h"
#include "Platform/SystemCall.h"

ExecutionUnitCreatorBase* ExecutionUnitCreatorBase::first = nullptr;

bool FrameExecutionUnit::afterFrame()
{
  return SystemCall::getMode() != SystemCall::physicalRobot;
}
