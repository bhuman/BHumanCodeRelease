#include "Representations/Modeling/ObstacleClusters.h"
#include "Tools/Debugging/DebugDrawings.h"

void ObstacleClusters::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:ObstacleClusters", "drawingOnField");

  for(const GaussianPositionDistribution& d :  obstacles)
  {
    COVARIANCE2D("representation:ObstacleClusters", d.covariance, d.robotPosition);
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
  const unsigned int numOfInputObstacles = obstacles.obstacles.size();
  unsigned int numOfUsedObstacles = numOfInputObstacles;
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
