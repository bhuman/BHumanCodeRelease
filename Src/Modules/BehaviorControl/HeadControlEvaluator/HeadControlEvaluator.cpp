/**
 * @author <a href="mailto:andisto@tzi.de">Andreas Stolpmann</a>
 */

#include "HeadControlEvaluator.h"
#include "Tools/Debugging/Debugging.h"

MAKE_MODULE(HeadControlEvaluator, cognitionInfrastructure)

void HeadControlEvaluator::update(HeadControlEvaluation& headControlEvaluation)
{
  DEBUG_RESPONSE_ONCE("module:HeadControlEvaluator:reset")
  {
    headControlEvaluation.ballCount = 0;
    headControlEvaluation.centerCircleCount = 0;
    headControlEvaluation.goalPostCount = 0;
    headControlEvaluation.lineCount = 0;
    headControlEvaluation.lineIntersectionCount = 0;
    headControlEvaluation.oppRobotCount = 0;
    headControlEvaluation.ownRobotCount = 0;
    headControlEvaluation.totalGoalCount = 0;
    headControlEvaluation.unkownRobotCount = 0;
  }

  // Ball
  if(theBallPercept.status == BallPercept::seen)
    headControlEvaluation.ballCount++;

  // CenterCircle
  if(theFrameInfo.getTimeSince(theCirclePercept.lastSeen) < 10)
    headControlEvaluation.centerCircleCount++;

  // Lines
  headControlEvaluation.lineCount += static_cast<unsigned>(theFieldLines.lines.size());
  headControlEvaluation.lineIntersectionCount += static_cast<unsigned>(theFieldLineIntersections.intersections.size());

  // Players
  for(const auto& player : thePlayersPercept.players)
  {
    if(!player.detectedFeet)
      continue;
    if(!player.detectedJersey)
    {
      headControlEvaluation.unkownRobotCount++;
    }
    else
    {
      if(player.ownTeam)
        headControlEvaluation.ownRobotCount++;
      else
        headControlEvaluation.oppRobotCount++;
    }
  }
}
