#include "ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/Approx.h"
#include "Tools/Math/Random.h"
#include "Tools/Modeling/Obstacle.h"
#include "Tools/Module/Blackboard.h"

void ObstacleModel::operator >> (BHumanMessage& m) const
{
  static_assert(BHULKS_STANDARD_MESSAGE_STRUCT_VERSION == 8, "This method is not adjusted for the current message version");

  m.theBHULKsStandardMessage.obstacles.clear();

  std::sort(const_cast<std::vector<Obstacle>&>(obstacles).begin(), const_cast<std::vector<Obstacle>&>(obstacles).end(), [](const Obstacle& a, const Obstacle& b) {return a.center.squaredNorm() < b.center.squaredNorm(); });

  const int numOfObstacle = std::min(BHULKS_STANDARD_MESSAGE_MAX_NUM_OF_OBSTACLES, static_cast<int>(obstacles.size()));
  for(int i = 0; i < numOfObstacle; ++i)
  {
    const Obstacle& o = obstacles[i];
    B_HULKs::Obstacle bhObstacle;

    bhObstacle.center[0] = o.center.x();
    bhObstacle.center[1] = o.center.y();
    bhObstacle.timestampLastSeen = o.lastSeen;
    bhObstacle.type = B_HULKs::ObstacleType(o.type);

    m.theBHULKsStandardMessage.obstacles.emplace_back(bhObstacle);
    m.theBHumanArbitraryMessage.queue.out.bin << o.covariance(0, 0);
    m.theBHumanArbitraryMessage.queue.out.bin << o.covariance(0, 1);
    m.theBHumanArbitraryMessage.queue.out.bin << o.covariance(1, 1);
    m.theBHumanArbitraryMessage.queue.out.bin << o.left;
    m.theBHumanArbitraryMessage.queue.out.bin << o.right;
  }

  if(!obstacles.empty())
    m.theBHumanArbitraryMessage.queue.out.finishMessage(id());
}

void ObstacleModel::operator << (const BHumanMessage& m)
{
  obstacles.clear();

  OutBinarySize obstacleArbitrarySize;
  {
    Obstacle o;
    obstacleArbitrarySize << o.covariance(0, 0);
    obstacleArbitrarySize << o.covariance(0, 1);
    obstacleArbitrarySize << o.covariance(1, 1);
    obstacleArbitrarySize << o.left;
    obstacleArbitrarySize << o.right;
  }

  for(const B_HULKs::Obstacle& bhObstacle : m.theBHULKsStandardMessage.obstacles)
  {
    obstacles.emplace_back();
    Obstacle& o = obstacles.back();
    o.center.x() = bhObstacle.center[0];
    o.center.y() = bhObstacle.center[1];
    o.lastSeen = m.toLocalTimestamp(bhObstacle.timestampLastSeen);
    o.type = Obstacle::Type(bhObstacle.type);

    //Fill with default (if we get arbitrary part, it will overwrite later)
    o.covariance << 1000.f, 0.f, 0.f, 1000.f;
    o.setLeftRight(Obstacle::getRobotDepth());
  }
}

bool ObstacleModel::handleArbitraryMessage(InMessage& m, const std::function<unsigned(unsigned)>& toLocalTimestamp)
{
  ASSERT(m.getMessageID() == id());
  for(Obstacle& o : obstacles)
  {
    float covXX, covXY, covYY;
    m.bin >> covXX;
    m.bin >> covXY;
    m.bin >> covYY;
    o.covariance << covXX, covXY, covXY, covYY;

    m.bin >> o.left;
    m.bin >> o.right;
  }

  return true;
}

void ObstacleModel::verify() const
{
#ifndef NDEBUG
  for(const auto& obstacle : obstacles)
  {
    ASSERT(std::isfinite(obstacle.center.x()));
    ASSERT(std::isfinite(obstacle.center.y()));

    ASSERT(std::isfinite(obstacle.left.x()));
    ASSERT(std::isfinite(obstacle.left.y()));

    ASSERT(std::isfinite(obstacle.right.x()));
    ASSERT(std::isfinite(obstacle.right.y()));

    ASSERT(std::isfinite(obstacle.velocity.x()));
    ASSERT(std::isfinite(obstacle.velocity.y()));

    //ASSERT((obstacle.left - obstacle.right).squaredNorm() < sqr(2000.f));

    ASSERT(std::isnormal(obstacle.covariance(0, 0)));
    ASSERT(std::isnormal(obstacle.covariance(1, 1)));
    ASSERT(std::isfinite(obstacle.covariance(0, 1)));
    ASSERT(std::isfinite(obstacle.covariance(1, 0)));
    ASSERT(Approx::isEqual(obstacle.covariance(0, 1), obstacle.covariance(1, 0), 1e-20f));
  }
#endif
}

void ObstacleModel::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:ObstacleModel:rectangle", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:ObstacleModel:centerCross", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:ObstacleModel:leftRight", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:ObstacleModel:circle", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:ObstacleModel:orientation", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:ObstacleModel:covariance", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:ObstacleModel:velocity", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:ObstacleModel:fallen", "drawingOnField");
  DECLARE_DEBUG_DRAWING3D("representation:ObstacleModel", "robot");

  static const ColorRGBA colors[] =
  {
    ColorRGBA::blue,
    ColorRGBA::red,
    ColorRGBA::yellow,
    ColorRGBA::black
  };

  const ColorRGBA ownColor = colors[Blackboard::getInstance().exists("OwnTeamInfo") ?
      static_cast<const OwnTeamInfo&>(Blackboard::getInstance()["OwnTeamInfo"]).teamColor : TEAM_BLACK];

  const ColorRGBA opponentColor = colors[Blackboard::getInstance().exists("OpponentTeamInfo") ?
      static_cast<const OpponentTeamInfo&>(Blackboard::getInstance()["OpponentTeamInfo"]).teamColor : TEAM_RED];

  ColorRGBA color;
  for(const auto& obstacle : obstacles)
  {
    switch(obstacle.type)
    {
      case Obstacle::goalpost:
      {
        color = ColorRGBA::white;
        break;
      }
      case Obstacle::fallenTeammate:
      case Obstacle::teammate:
      {
        color = ownColor;
        break;
      }
      case Obstacle::fallenOpponent:
      case Obstacle::opponent:
      {
        color = opponentColor;
        break;
      }
      case Obstacle::fallenSomeRobot:
      case Obstacle::someRobot:
      {
        color = ColorRGBA::orange;
        break;
      }
      default:
      {
        color = ColorRGBA::violet;
        break;
      }
    }
    const Vector2f& center = obstacle.center;
    const Vector2f& left = obstacle.left;
    const Vector2f& right = obstacle.right;

    CYLINDER3D("representation:ObstacleModel", center.x(), center.y(), -210, 0, 0, 0, (left - right).norm(), 10, color);
    CROSS("representation:ObstacleModel:centerCross", center.x(), center.y(), Obstacle::getRobotDepth(), 10, Drawings::solidPen, color);

    float obstacleRadius = (left - right).norm() * .5f;
    Angle robotRotation = Blackboard::getInstance().exists("RobotPose") ? static_cast<const RobotPose&>(Blackboard::getInstance()["RobotPose"]).rotation : Angle();
    Vector2f frontRight(-Obstacle::getRobotDepth(), -obstacleRadius);
    frontRight = center + frontRight;
    RECTANGLE2("representation:ObstacleModel:rectangle", frontRight, obstacleRadius * 2, obstacleRadius * 2, -robotRotation, 16, Drawings::PenStyle::solidPen, ColorRGBA::black, Drawings::solidBrush, color);

    LINE("representation:ObstacleModel:leftRight", center.x(), center.y(), left.x(), left.y(), 20, Drawings::dottedPen, color);
    LINE("representation:ObstacleModel:leftRight", center.x(), center.y(), right.x(), right.y(), 20, Drawings::dottedPen, color);
    CIRCLE("representation:ObstacleModel:circle", center.x(), center.y(), obstacleRadius, 10, Drawings::dottedPen, color, Drawings::noBrush, color);
    COVARIANCE2D("representation:ObstacleModel:covariance", obstacle.covariance, center);

    if(obstacle.velocity.squaredNorm() > 0)
      ARROW("representation:ObstacleModel:velocity", center.x(), center.y(),
            center.x() + 2 * obstacle.velocity.x(), center.y() + 2 * obstacle.velocity.y(), 10, Drawings::solidPen, ColorRGBA::black);

    if(obstacle.type >= Obstacle::fallenSomeRobot)
    {
      DRAWTEXT("representation:ObstacleModel:fallen", center.x(), center.y(), 100, color, "FALLEN");
    }

    // draw orientation
    if(obstacle.detectedOrientation == 2)
    {
      float x = std::cos(obstacle.orientation) * obstacleRadius;
      float y = std::sin(obstacle.orientation) * obstacleRadius;

      LINE("representation:ObstacleModel:orientation", obstacle.center.x() + x, obstacle.center.y() + y, obstacle.center.x(), obstacle.center.y(), 10, Drawings::solidPen, ColorRGBA::gray);
    }
    else if(obstacle.detectedOrientation == 1)
    {
      float x = std::cos(obstacle.orientation) * obstacleRadius;
      float y = std::sin(obstacle.orientation) * obstacleRadius;

      LINE("representation:ObstacleModel:orientation", obstacle.center.x() + x, obstacle.center.y() + y, obstacle.center.x() - x, obstacle.center.y() - y, 10, Drawings::solidPen, ColorRGBA::gray);
    }
  }
}
