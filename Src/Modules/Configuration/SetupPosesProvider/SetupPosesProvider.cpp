/**
 * @file SetupPosesProvider.cpp
 *
 * This file implements a module that provides the poses of the robots before
 * entering the field. Poses are based on setupPoses.cfg configuration file.
 * However, if any robot with a number > 7 is in the starting team, the poses
 * are rearranged.
 * Default lineup is clockwise with ascending numbers, starting in the own left corner with the number 1:
 *
 * //  4        5  //  (edge of center circle)
 * //              //
 * //  3        6  //  (middle between 2/4 and 5/7)
 * //              //
 * //  2        7  //  (edge of penalty area)
 * //              //
 * //  1           //  (edge of goal area [not larger penalty area])
 * //   Own Goal   //
 *
 * If any robot is missing as it is a substitute (not because of a penalty!),
 * the other robots step up to fill the gap to to preserve the constraint of ascending numbers.
 * Example (robot 2 is a substitute and robot 9 is playing instead):
 *
 * //  5        6  //  (edge of center circle)
 * //              //
 * //  4        7  //  (middle between 2/4 and 5/7)
 * //              //
 * //  3        9  //  (edge of penalty area)
 * //              //
 * //  1           //  (edge of goal area [not larger penalty area])
 * //   Own Goal   //
 *
 * OK?
 *
 * @author Tim Laue
 */

#include "SetupPosesProvider.h"

SetupPosesProvider::SetupPosesProvider()
{
  representationHasBeenInitialized = false;
  // We assume that the default configuration is valid
  for(int i = 1; i <= 7; i++)
    robotOrderFromGC.push_back(i);
}

void SetupPosesProvider::update(SetupPoses& setupPoses)
{
  if(!representationHasBeenInitialized)
  {
    setupPoses.poses = poses;
    representationHasBeenInitialized = true;
  }
  // Only change things in initial and after receiving new information from the GameController:
  if(theGameState.isInitial() && theGameState.gameControllerActive && updateRobotOrder())
  {
    if(robotOrderFromGC.size() <= setupPoses.poses.size())
    {
      for(std::vector<int>::size_type i = 0; i<robotOrderFromGC.size(); i++)
        setupPoses.poses[i].playerNumber = robotOrderFromGC[i];
    }
  }
}

bool SetupPosesProvider::updateRobotOrder()
{
  std::vector<int> currentRobotOrder;
  for(unsigned long i=0; i<theGameState.ownTeam.playerStates.size(); i++)
  {
    if(theGameState.ownTeam.playerStates[i] != GameState::substitute)
      currentRobotOrder.push_back(static_cast<int>(i)+1);
  }
  if(currentRobotOrder != robotOrderFromGC)
  {
    robotOrderFromGC = currentRobotOrder;
    return true;
  }
  return false;
}

MAKE_MODULE(SetupPosesProvider);
