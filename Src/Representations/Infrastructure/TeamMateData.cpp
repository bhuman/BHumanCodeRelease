/**
 * @file TeammateData.cpp
 * Declaration of a class representing information about the teammates.
 * @author Colin Graf, Alexis Tsogias
 */

#include "TeammateData.h"
#include "Representations/Infrastructure/RoboCupGameControlData.h"

TeammateData::TeammateData()
{
  for(int i = 0; i < numOfPlayers; ++i)
  {
    timeStamps[i] = 0;
    isActive[i] = false;
    isFullyActive[i] = false;
    isPenalized[i] = false;
    hasGroundContact[i] = false;
    isUpright[i] = false;
    timeLastGroundContact[i] = 0;
    cameraHeights[i] = 0;
    isBHumanPlayer[i] = true;
    intention[i] = 0;
  }
}

void TeammateData::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:TeammateData", "drawingOnField");
  for(int i = 1; i < numOfPlayers; ++i)
  {
    if(timeStamps[i])
    {
      ColorRGBA posCol;
      if(isFullyActive[i])
      {
        posCol = ColorRGBA(0, 255, 0);
      }
      else if(isActive[i])
      {
        posCol = ColorRGBA(255, 255, 0);
      }
      else
      {
        posCol = ColorRGBA(255, 0, 0);
      }

      const Vector2<>& rPos = robotPoses[i].translation;
      float radius = robotPoses[i].deviation;
      if(radius < 50.f)
      {
        radius = 50.f;
      }

      Vector2<> dirPos = robotPoses[i] * Vector2<>(radius, 0);
      // Circle arround Player
      CIRCLE("representation:TeammateData", rPos.x, rPos.y, radius, 20, Drawings::ps_solid,
             posCol, Drawings::bs_null, ColorRGBA::white);
      // Direction of the Robot
      LINE("representation:TeammateData", rPos.x, rPos.y, dirPos.x, dirPos.y, 20,
           Drawings::ps_solid, posCol);
      // Player number
      DRAWTEXT("representation:TeammateData", rPos.x + 100, rPos.y, 100, ColorRGBA::black, i);
      // Role
      DRAWTEXT("representation:TeammateData", rPos.x + 100, rPos.y - 150, 100,
               ColorRGBA::black, Role::getName(behaviorStatus[i].role));
      // time to reach ball
      DRAWTEXT("representation:TeammateData", rPos.x + 100, rPos.y - 300, 100,
               ColorRGBA::black, "" << behaviorStatus[i].estimatedTimeToReachBall);
      // intention
      std::string intentionStr;
      switch(intention[i])
      {
        case DROPIN_INTENTION_DEFAULT:
          intentionStr = "default";
          break;
        case DROPIN_INTENTION_KEEPER:
          intentionStr = "keeper";
          break;
        case DROPIN_INTENTION_DEFENSIVE:
          intentionStr = "defensive";
          break;
        case DROPIN_INTENTION_KICK:
          intentionStr = "kick";
          break;
        case DROPIN_INTENTION_LOST:
          intentionStr = "lost";
          break;
        default:
          intentionStr = "unknown";
      }
      DRAWTEXT("representation:TeammateData", rPos.x + 100, rPos.y - 450, 100,
        ColorRGBA::black, "Intention " << intentionStr);

      Vector2<> bPos = robotPoses[i] * ballModels[i].estimate.position;
      ColorRGBA ballCol;
      if(currentTimestamp - ballModels[i].timeWhenLastSeen < networkTimeout / 2)
      {
        ballCol = ColorRGBA(0, 255, 0);
      }
      else if(currentTimestamp - ballModels[i].timeWhenLastSeen < networkTimeout)
      {
        ballCol = ColorRGBA(255, 255, 0);
      }
      else
      {
        ballCol = ColorRGBA(255, 0, 0);
      }
      //Line from Robot to Ball
      LINE("representation:TeammateData", rPos.x, rPos.y, walkingTo[i].x, walkingTo[i].y, 10, Drawings::ps_dash, ColorRGBA::magenta);

      // Ball position
      CIRCLE("representation:TeammateData", bPos.x, bPos.y, 50, 20, Drawings::ps_solid,
             ballCol, Drawings::bs_solid, ballCol);
      //Line from Robot to Ball
      LINE("representation:TeammateData", rPos.x, rPos.y, bPos.x, bPos.y, 20, Drawings::ps_dash, ballCol);
    }
  }
}

unsigned TeammateData::getIntentionForRole(Role::RoleType role)
{
  switch(role)
  {
    case Role::striker:
      return DROPIN_INTENTION_KICK;
    case Role::keeper:
      return DROPIN_INTENTION_KEEPER;
    case Role::defender:
      return DROPIN_INTENTION_DEFENSIVE;
    default:
      return DROPIN_INTENTION_DEFAULT;
  }
}

TeammateDataCompressed::TeammateDataCompressed(const TeammateData& teammateData)
{
  currentTimestamp = teammateData.currentTimestamp;
  firstTeammate = teammateData.firstTeammate;
  for(int i = TeammateData::firstPlayer; i < TeammateData::numOfPlayers; ++i)
  {
    timeStamps[i] = teammateData.timeStamps[i];
    isActive[i] = teammateData.isActive[i];
    isFullyActive[i] = teammateData.isFullyActive[i];
    ballModels[i] = BallModelCompressed(teammateData.ballModels[i]);
    robotPoses[i] = RobotPoseCompressed(teammateData.robotPoses[i]);
    robotsSideConfidence[i] = teammateData.robotsSideConfidence[i];
    roles[i] = teammateData.behaviorStatus[i].role;
    walkingTo[i] = teammateData.walkingTo[i];
    intention[i] = teammateData.intention[i];
  }
  dropInData = teammateData.dropInData;
}

TeammateDataCompressed::operator TeammateData() const
{
  TeammateData teammateData;
  teammateData.currentTimestamp = currentTimestamp;
  teammateData.firstTeammate = firstTeammate;
  for(int i = TeammateData::firstPlayer; i < TeammateData::numOfPlayers; ++i)
  {
    teammateData.timeStamps[i] = timeStamps[i];
    teammateData.isActive[i] = isActive[i];
    teammateData.isFullyActive[i] = isFullyActive[i];
    teammateData.ballModels[i] = BallModel(ballModels[i]);
    teammateData.robotPoses[i] = RobotPose(robotPoses[i]);
    teammateData.robotsSideConfidence[i] = robotsSideConfidence[i];
    teammateData.behaviorStatus[i].role = roles[i];
    teammateData.walkingTo[i] = walkingTo[i];
    teammateData.intention[i] = intention[i];
  }
  teammateData.dropInData = dropInData;

  return teammateData;
}