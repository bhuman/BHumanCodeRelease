/**
 * @file BallSearchParticlesProvider.cpp
 *
 * This file implements particles for the ball search.
 *
 * @author Moritz Oppermann
 */

#include "BallSearchParticlesProvider.h"
#include "Debugging/DebugDrawings.h"
#include "Tools/BehaviorControl/Strategy/Agent.h"

MAKE_MODULE(BallSearchParticlesProvider);

void BallSearchParticlesProvider::update(BallSearchParticles& theBallSearchParticles)
{
  if(theExtendedGameState.stateLastFrame != theGameState.state || theBallSearchParticles.particles.empty())
  {
    reset(theBallSearchParticles);
  }

  if(theCameraMatrix.isValid)
  {
    //builds the polygon of visible field parts of the robot
    std::vector<Vector2f> polygon;
    Projection::computeFieldOfViewInFieldCoordinates(theRobotPose, theCameraMatrix, theCameraInfo, theFieldDimensions, polygon);
    const std::list<SectorWheel::Sector> wheel = calculateObstacleSectors();

    //give the referee some time to place the ball
    bool refereeToleranceNeeded = theGameState.isCornerKick() || theGameState.isKickIn() || theGameState.isGoalKick();
    if(!refereeToleranceNeeded || theFrameInfo.getTimeSince(theGameState.timeWhenStateStarted) > refereeTolerance)
    {
      for(size_t i = 0; i < theBallSearchParticles.particles.size();)
      {
        if(isParticleInView(theBallSearchParticles.particles[i], polygon) && !isBlockedByObstacle(theBallSearchParticles.particles[i], wheel))
        {
          if(theGameState.isCornerKick() || theGameState.isKickIn() || theGameState.isGoalKick())
          {
            theBallSearchParticles.particles.erase(theBallSearchParticles.particles.begin() + i);
            continue;
          }
          else
            theBallSearchParticles.particles[i] = createRandomParticle();
        }
        i++;
      }
    }
  }

  theBallSearchParticles.positionToSearch = [this](const Agent& agent) -> Vector2f
  {
    return positionToSearch(agent);
  };

  theBallSearchParticles.anyParticlesInRegion = [this](const Agent& agent) -> bool
  {
    return anyParticlesInRegion(agent);
  };

  draw();
}

void BallSearchParticlesProvider::draw() const
{
  DECLARE_DEBUG_DRAWING("module:BallSearchParticlesProvider:particles", "drawingOnField");
  for(const Vector2f& particle : theBallSearchParticles.particles)
  {
    CIRCLE("module:BallSearchParticlesProvider:particles",
           particle.x(),
           particle.y(),
           30,
           10,
           Drawings::solidPen,
           ColorRGBA::orange,
           Drawings::solidBrush,
           ColorRGBA::orange);
  }
}

void BallSearchParticlesProvider::reset(BallSearchParticles& theBallSearchParticles) const
{
  theBallSearchParticles.particles.clear();

  if(theGameState.isKickIn())
  {
    // place half of the particles on each touchline
    const int particlesPerSide = numOfParticles / 2;
    const float distBetweenParticles = (2 * theFieldDimensions.xPosOpponentGoalLine) / particlesPerSide;
    for(int i = 0; i < particlesPerSide; i++)
    {
      theBallSearchParticles.particles.emplace_back(theFieldDimensions.xPosOwnGoalLine + distBetweenParticles * i, theFieldDimensions.yPosRightTouchline);
      theBallSearchParticles.particles.emplace_back(theFieldDimensions.xPosOwnGoalLine + distBetweenParticles * i, theFieldDimensions.yPosLeftTouchline);
    }
  }
  else if(theGameState.isCornerKick())
  {
    if(theGameState.isForOwnTeam() || !theGameState.kickingTeamKnown)
    {
      theBallSearchParticles.particles.emplace_back(theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosRightTouchline);
      theBallSearchParticles.particles.emplace_back(theFieldDimensions.xPosOpponentGoalLine, -theFieldDimensions.yPosRightTouchline);
    }
    if(theGameState.isForOpponentTeam() || !theGameState.kickingTeamKnown)
    {
      theBallSearchParticles.particles.emplace_back(-theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosRightTouchline);
      theBallSearchParticles.particles.emplace_back(-theFieldDimensions.xPosOpponentGoalLine, -theFieldDimensions.yPosRightTouchline);
    }
  }
  else if(theGameState.isGoalKick())
  {
    if(theGameState.isForOwnTeam() || !theGameState.kickingTeamKnown)
    {
      theBallSearchParticles.particles.emplace_back(-theFieldDimensions.xPosOpponentGoalArea, theFieldDimensions.yPosRightGoalArea);
      theBallSearchParticles.particles.emplace_back(-theFieldDimensions.xPosOpponentGoalArea, -theFieldDimensions.yPosRightGoalArea);
    }
    if(theGameState.isForOpponentTeam() || !theGameState.kickingTeamKnown)
    {
      theBallSearchParticles.particles.emplace_back(theFieldDimensions.xPosOpponentGoalArea, theFieldDimensions.yPosRightGoalArea);
      theBallSearchParticles.particles.emplace_back(theFieldDimensions.xPosOpponentGoalArea, -theFieldDimensions.yPosRightGoalArea);
    }
  }
  else
  {
    while(theBallSearchParticles.particles.size() < numOfParticles)
    {
      theBallSearchParticles.particles.emplace_back(createRandomParticle());
    }
  }
}

Vector2f BallSearchParticlesProvider::createRandomParticle() const
{
  return { Random::uniform(theFieldDimensions.xPosOwnGoalLine, theFieldDimensions.xPosOpponentGoalLine),
           Random::uniform(theFieldDimensions.yPosRightTouchline, theFieldDimensions.yPosLeftTouchline) };
}

std::list<SectorWheel::Sector> BallSearchParticlesProvider::calculateObstacleSectors() const
{
  SectorWheel sectorWheel;
  sectorWheel.begin(theRobotPose.translation, maxViewDistance);
  for(const Obstacle& obstacle : theObstacleModel.obstacles)
  {
    const Vector2f obstacleOnField = theRobotPose * obstacle.center;
    if(obstacleOnField.x() > (theFieldDimensions.xPosOpponentGoalLine + theFieldDimensions.xPosOpponentGoal) * 0.5f)
      continue;
    const float obstacleWidth = (obstacle.left - obstacle.right).norm() + obstacleOffset;
    const float obstacleDistance = std::sqrt(std::max((obstacleOnField - theRobotPose.translation).squaredNorm() - sqr(obstacleWidth / 2.f), 1.f));
    const float obstacleRadius = std::atan(obstacleWidth / (2.f * obstacleDistance));
    const Angle obstacleDirection = (obstacleOnField - theRobotPose.translation).angle();
    sectorWheel.addSector(Rangea(Angle::normalize(obstacleDirection - obstacleRadius), Angle::normalize(obstacleDirection + obstacleRadius)), obstacleDistance, SectorWheel::Sector::obstacle);
  }
  return sectorWheel.finish();
}

bool BallSearchParticlesProvider::isParticleInView(const Vector2f& particle, const std::vector<Vector2f>& polygon) const
{
  return Geometry::isPointInsideConvexPolygon(polygon.data(), static_cast<int>(polygon.size()), particle);
}

bool BallSearchParticlesProvider::isBlockedByObstacle(const Vector2f& particle, const std::list<SectorWheel::Sector>& wheel) const
{
  for(const auto& sector : wheel)
  {
    if(Geometry::isPointInsideArc(particle, theRobotPose.translation, sector.angleRange, sector.distance))
    {
      return false;
    }
  }
  return true;
}

Vector2f BallSearchParticlesProvider::positionToSearch(const Agent& agent) const
{
  std::vector<std::vector<unsigned>> grid;
  const unsigned cellCountX = static_cast<unsigned>(theFieldDimensions.xPosOpponentGoalLine * 2 / cellWidth + 1);
  const unsigned cellCountY = static_cast<unsigned>(theFieldDimensions.yPosLeftTouchline * 2 / cellHeight + 1);
  // building the grid from the upper left to the bottom right corner of the field
  for(unsigned y = 0; y < cellCountY; y++)
  {
    std::vector<unsigned> row;
    for(unsigned x = 0; x < cellCountX; x++)
    {
      row.emplace_back(0);
    }
    grid.emplace_back(row);
  }

  for(const Vector2f& particle : theBallSearchParticles.particles)
  {
    if(Geometry::isPointInsideConvexPolygon(agent.baseArea.data(), static_cast<int>(agent.baseArea.size()), particle))
    {
      grid[static_cast<unsigned>((theFieldDimensions.yPosLeftTouchline + particle.y()) / cellHeight)]
          [static_cast<unsigned>((theFieldDimensions.xPosOpponentGoalLine + particle.x()) / cellWidth)]++;
      CROSS("module:BallSearchParticlesProvider:particles", particle.x(), particle.y(), 50, 20, Drawings::solidPen, ColorRGBA::red);
    }
  }

  // find grid cell with most particles
  unsigned max = 0;
  unsigned maxIndexI = 0;
  unsigned maxIndexJ = 0;
  for(unsigned i = 0; i < cellCountY; i++)
  {
    for(unsigned j = 0; j < cellCountX; j++)
    {
      if(grid[i][j] > max)
      {
        max = grid[i][j];
        maxIndexI = i;
        maxIndexJ = j;
      }
    }
  }

  const Vector2f positionToLookNext = { theFieldDimensions.xPosOwnGoalLine + maxIndexJ * cellWidth, theFieldDimensions.yPosRightTouchline + maxIndexI * cellHeight };

  CROSS("module:BallSearchParticlesProvider:particles", positionToLookNext.x(), positionToLookNext.y(), 100, 40, Drawings::solidPen, ColorRGBA::blue);

  return positionToLookNext;
}

bool BallSearchParticlesProvider::anyParticlesInRegion(const Agent& agent) const
{
  for(const Vector2f& particle : theBallSearchParticles.particles)
  {
    if(Geometry::isPointInsideConvexPolygon(agent.baseArea.data(), static_cast<int>(agent.baseArea.size()), particle))
      return true;
  }
  return false;
}
