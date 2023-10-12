/**
 * @file SetupPoses.cpp
 *
 * Implementation of a representation that contains information about
 * the pose from which the robots enter the pitch when the game state
 * switches from INITIAL to READY.
 *
 * @author Tim Laue
 * @author Arne Hasselbring
 */

#include "SetupPoses.h"
#include "Representations/Infrastructure/GameState.h"
#include "Debugging/Debugging.h"
#include "Debugging/Debugging.h"
#include "Framework/Blackboard.h"
#include "Framework/Settings.h"
#include "Math/Angle.h"
#include "Streaming/Global.h"
#include "Streaming/Output.h"

void SetupPoses::draw() const
{
  DEBUG_RESPONSE_ONCE("representation:SetupPoses:place")
  {
    const int playerNumber = []
    {
      if(Blackboard::getInstance().exists("GameState"))
      {
        auto& gameState = static_cast<const GameState&>(Blackboard::getInstance()["GameState"]);
        return gameState.playerNumber;
      }
      else if(Global::settingsExist())
        return Global::getSettings().playerNumber;
      else
        return 1;
    }();
    const auto& pose = getPoseOfRobot(playerNumber);
    OUTPUT(idConsole, text, "mv " << pose.position.x() << " " << pose.position.y() << " 300 0 0 " << toDegrees((pose.turnedTowards - pose.position).angle()));
  }
}

const SetupPoses::SetupPose& SetupPoses::getPoseOfRobot(int number) const
{
  ASSERT(poses.size() > 0);
  if(poses.size() == 1)
    return poses[0];
  for(const auto& pose : poses)
    if(pose.playerNumber == number)
      return pose;
  return poses.back(); // Default: Return last element of list
}
