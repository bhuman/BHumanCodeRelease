/**
* @file PenaltyMarkWithPenaltyAreaLinePerceptor.cpp
*
* Implementation of a module that tries to find the combination of
* a penalty mark and the line of the penalty area that is
* closest to it (the line between penalty mark and the field's
* center line) in the current field percepts. This is the combination
* that this modul is looking for:
*
*                  (goal)
*-----------------------------------------
*          |                   |
*          |         + (mark)  |
*          ===================== (<- this line)
*
* @author Tim Laue
*/

#include "PenaltyMarkWithPenaltyAreaLinePerceptor.h"
#include "Tools/Math/Geometry.h"


PenaltyMarkWithPenaltyAreaLinePerceptor::PenaltyMarkWithPenaltyAreaLinePerceptor()
{
  modelDistancePenaltyMarkToLine = theFieldDimensions.xPosOpponentPenaltyMark - theFieldDimensions.xPosOpponentPenaltyArea;
  timeWhenPenaltyMarkLastSeen = 0;
}

void PenaltyMarkWithPenaltyAreaLinePerceptor::update(PenaltyMarkWithPenaltyAreaLine& penaltyMarkWithPenaltyAreaLine)
{
  penaltyMarkWithPenaltyAreaLine.clear();
  penaltyMarkWithPenaltyAreaLine.isValid = false;
  // Check for backwards "jump" in log file:
  if(theFrameInfo.time < timeWhenPenaltyMarkLastSeen)
    timeWhenPenaltyMarkLastSeen = 0;
  // Buffer a new penalty mark:
  if(thePenaltyMarkPercept.wasSeen)
  {
    penaltyMarkPosition = thePenaltyMarkPercept.positionOnField;
    timeWhenPenaltyMarkLastSeen = theFrameInfo.time;
  }
  // If there is no new penalty mark, try to update the buffered one by odometry:
  else if(theFrameInfo.getTimeSince(timeWhenPenaltyMarkLastSeen) < bufferTimePenaltyMark)
  {
    const Pose2f odometryOffset = theOdometer.odometryOffset.inverse();
    penaltyMarkPosition = odometryOffset * penaltyMarkPosition;
  }
  // There is no penalty mark that we can currently use:
  else
  {
    return;
  }
  // There seems to be a penalty mark available, so let's find the corresponding line:
  if(findMatchingLine())
  {
    computePose(penaltyMarkWithPenaltyAreaLine);
    penaltyMarkWithPenaltyAreaLine.isValid = true;
  }
}

bool PenaltyMarkWithPenaltyAreaLinePerceptor::findMatchingLine()
{
  const float minimumLineLengthSqr = minimumLineLength * minimumLineLength;
  // Iterate over all observed lines and try to find the penalty area border.
  // The loop stops when one candidate is found as the matching should be unambigous.
  for(unsigned int i = 0; i < theFieldLines.lines.size(); ++i)
  {
    const auto& line = theFieldLines.lines[i];
    // Check for length constraint:
    const float lineLengthSqr = (line.first - line.last).squaredNorm();
    if(lineLengthSqr < minimumLineLengthSqr)
      continue;
    // Check distance to penalty mark:
    const Geometry::Line lineObject(line.first, line.last - line.first);
    const float distanceToLine = std::abs(Geometry::getDistanceToEdge(lineObject, penaltyMarkPosition));
    if(distanceToLine > modelDistancePenaltyMarkToLine - maximumDeviation && distanceToLine < modelDistancePenaltyMarkToLine + maximumDeviation)
    {
      lineStart = line.first;
      lineEnd = line.last;
      return true;
    }
  }
  return false;
}

void PenaltyMarkWithPenaltyAreaLinePerceptor::computePose(PenaltyMarkWithPenaltyAreaLine& penaltyMarkWithPenaltyAreaLine)
{
  Vector2f lineDirection = lineEnd - lineStart;
  lineDirection.normalize();
  const Vector2f projectedMark = Geometry::getOrthogonalProjectionOfPointOnLine(lineStart, lineDirection, penaltyMarkPosition);
  const Vector2f directionToMark = penaltyMarkPosition - projectedMark;
  Pose2f perceivedPose(directionToMark.angle(), projectedMark);
  penaltyMarkWithPenaltyAreaLine = perceivedPose;
}

MAKE_MODULE(PenaltyMarkWithPenaltyAreaLinePerceptor, perception);
