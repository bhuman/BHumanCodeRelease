/**
 * @file Representations/MotionControl/GetUpEngineOutputLog.cpp
 * @author Philip Reichenberg
 */

#include "GetUpEngineOutputLog.h"
#include "Tools/Debugging/DebugDrawings.h"

void GetUpEngineOutputLog::draw()
{
  PLOT("representation:GetUpEngineOutputLog:balanceX", lineInfo.balanceValue.x());
  PLOT("representation:GetUpEngineOutputLog:balanceY", lineInfo.balanceValue.y());
  PLOT("representation:GetUpEngineOutputLog:comX", lineInfo.comDif.x());
  PLOT("representation:GetUpEngineOutputLog:comY", lineInfo.comDif.y());
}
