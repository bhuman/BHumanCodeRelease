/**
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "OpponentBreakingSupporterPositionProvider.h"

MAKE_MODULE(OpponentBreakingSupporterPositionProvider, modeling)

void OpponentBreakingSupporterPositionProvider::update(OpponentBreakingSupporterPosition& opponentOpponentBreakingSupporter)
{
  const std::vector<const Obstacle*> possibleOBs = findPossibleOBSs();
  if(possibleOBs.empty())
    updateWithNoInput(opponentOpponentBreakingSupporter);
  else
    updateWithInput(opponentOpponentBreakingSupporter, possibleOBs);
}

void OpponentBreakingSupporterPositionProvider::updateWithNoInput(OpponentBreakingSupporterPosition& opponentOpponentBreakingSupporter)
{
  opponentOpponentBreakingSupporter.isValid = false;//TODO some timesout
}

void OpponentBreakingSupporterPositionProvider::updateWithInput(OpponentBreakingSupporterPosition& opponentOpponentBreakingSupporter, const std::vector<const Obstacle*> possibleOBs)
{
  opponentOpponentBreakingSupporter.asObstacle = *possibleOBs.front();//todo selection
  opponentOpponentBreakingSupporter.isValid = true;
}

std::vector<const Obstacle*> OpponentBreakingSupporterPositionProvider::findPossibleOBSs()
{
  static const unsigned usefullObstacleTypes = bit(Obstacle::someRobot) | bit(Obstacle::opponent) | bit(Obstacle::fallenOpponent);

  auto isInsideSearchArea = [&](const Obstacle& obstacle)
  {
    static const Vector2f areaCenter(theFieldDimensions.xPosOwnPenaltyMark, theFieldDimensions.yPosCenterGoal);
    Vector2f obstacleCenterRelToArea = obstacle.center - areaCenter;
    obstacleCenterRelToArea.y() /= yFactorForOvalArea;
    return obstacleCenterRelToArea.squaredNorm() < sqrMaxDistanceToSearchAreaCenter;
  };

  std::vector<const Obstacle*> usefullObstacles;
  for(const Obstacle& obstacle : theTeamPlayersModel.obstacles)
    if(bit(obstacle.type) & usefullObstacleTypes && isInsideSearchArea(obstacle))
      usefullObstacles.emplace_back(&obstacle);

  return usefullObstacles;
}
