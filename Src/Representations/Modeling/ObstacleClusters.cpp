#include "Representations/Modeling/ObstacleClusters.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/ImageProcessing/ColorModelConversions.h"

void ObstacleClusters::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:ObstacleClusters", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:ObstacleClusters:Center", "drawingOnField");
  DECLARE_DEBUG_DRAWING3D("representation:ObstacleClusters:Center", "robot");

  TRANSLATE3D("representation:ObstacleClusters:Center", 0, 0, -230);
  for(const GaussianPositionDistribution& d :  obstacles)
  {
    ColorRGBA color = ColorRGBA::violet;
    color.a = 75;

    COVARIANCE2D("representation:ObstacleClusters", d.covariance, d.robotPosition);
    CROSS("representation:ObstacleClusters:Center", d.robotPosition.x, d.robotPosition.y, 100, 50, 2.f, color);

    CROSS3D("representation:ObstacleClusters:Center", d.robotPosition.x, d.robotPosition.y, 1.f, 50, 2.f, color);
    LINE3D("representation:ObstacleClusters:Center", 0, 0, 1.f, d.robotPosition.x, d.robotPosition.y, 1.f, 2.f, color);
  }
}

ObstacleClustersCompressed::Obstacle::Obstacle(const GaussianPositionDistribution& g)
: position(g.robotPosition),
  c00(g.covariance.c[0][0]),
  c01(g.covariance.c[0][1]),
  c11(g.covariance.c[1][1]) {}

ObstacleClustersCompressed::Obstacle::operator GaussianPositionDistribution() const
{
  return GaussianPositionDistribution(Vector2<>(position), Matrix2x2<>(c00, c01, c01, c11));
}

ObstacleClustersCompressed::ObstacleClustersCompressed(const ObstacleClusters& obstacles, unsigned int maxNumOfObstaclesToSend)
{
  unsigned int offset = 0;
  const size_t numOfInputObstacles = obstacles.obstacles.size();
  size_t numOfUsedObstacles = numOfInputObstacles;
  if(numOfUsedObstacles > maxNumOfObstaclesToSend)
  {
    numOfUsedObstacles = maxNumOfObstaclesToSend;
    offset = static_cast<unsigned int>(random(static_cast<int>(numOfInputObstacles)));
  }
  this->obstacles.reserve(numOfUsedObstacles);
  for(size_t i = 0; i < numOfUsedObstacles; i++)
    this->obstacles.push_back(obstacles.obstacles[(offset + i) % numOfInputObstacles]);
}

ObstacleClustersCompressed::operator ObstacleClusters() const
{
  ObstacleClusters clusters;
  clusters.obstacles.reserve(obstacles.size());
  for(const GaussianPositionDistribution& pos : obstacles)
  {
    clusters.obstacles.push_back(pos);
  }
  return clusters;
}
