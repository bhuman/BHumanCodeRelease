#include "ObstacleClusterer.h"
#include "Tools/Math/Geometry.h"

MAKE_MODULE(ObstacleClusterer, Modeling)

void ObstacleClusterer::update(ObstacleClusters& clusters)
{
  //FIXME this method is an evil hack made on GO2013!!! reimplement it

  clusters.obstacles.clear();
  const float halfWidth = theObstacleWheel.coneWidth / 2;

  std::list<Vector2<> > spotBuffer;
  for(const ObstacleWheel::Cone& cone : theObstacleWheel.cones)
  {
    if(cone.hasObstacle)
    {
      Vector2<> spot(cone.distance + robotRadius, 0);
      spot.rotate(cone.angle + halfWidth);
      if(isInsideField(spot))
      {
        spotBuffer.push_back(spot);
      }
    }
  }

  std::list<Cluster> tempClusters;
  int i = 0;
  while((!spotBuffer.empty()) && i < 360) //i only exists to avoid possible endless loops :)
  {
    tempClusters.push_back(Cluster());
    extractCluster(spotBuffer, tempClusters.back());
    ++i;
  }

  for(Cluster& c : tempClusters)
  {
    calculateCenterOfMass(c);
    Vector2<int> pointInImg;
    Geometry::calculatePointInImage(c.centerOfMass, theCameraMatrix, theCameraInfo, pointInImg);

        /**note: This covariance is probably as wrong as it gets. The spots from the obstacle wheel are
                 moved every frame using the odometry. Their covariance should get worse with every move.
                 It should get better with every new measurement. Additionally using relative2FieldCoord
                 and calculatepointInImage introduces additional errors which are not reflected by the
                 covariance. I doubt that this is any better than just using 1 0 0 1 as cov...*/
    clusters.obstacles.push_back(GaussianPositionDistribution(c.centerOfMass, getCovOfPixelInWorld(Vector2<float>(pointInImg), 0.0f)));
  }
}

Matrix2x2<> ObstacleClusterer::getCovOfPixelInWorld(const Vector2<>& correctedPointInImage, float pointZInWorld) const
{
  const Vector3<> unscaledVectorToPoint(theCameraInfo.focalLength, theCameraInfo.opticalCenter.x - correctedPointInImage.x, theCameraInfo.opticalCenter.y - correctedPointInImage.y);
  const Vector3<> unscaledWorld = theCameraMatrix.rotation * unscaledVectorToPoint;
  const float h = theCameraMatrix.translation.z - pointZInWorld;
  const float scale = h / -unscaledWorld.z;
  Vector2<> pointInWorld(unscaledWorld.x * scale, unscaledWorld.y * scale);
  const float distance = pointInWorld.abs();
  Vector2<> cossin = distance == 0.f ? Vector2<>(1.f, 0.f) : pointInWorld * (1.f / distance);
  Matrix2x2<> rot(cossin, Vector2<>(-cossin.y, cossin.x));
  Matrix2x2<> cov(Vector2<>(sqr(h / std::tan((distance == 0.f ? pi_2 : std::atan(h / distance)) - robotRotationDeviation.x) - distance), 0.f),
                 Vector2<>(0.f, sqr(std::tan(robotRotationDeviation.y) * distance)));
  return rot * cov * rot.transpose();
}

bool ObstacleClusterer::isInsideField(const Vector2<>& spot)
{
  Vector2<> pointInWorld = Geometry::relative2FieldCoord(theRobotPose, spot.x, spot.y);
  const float yMax = theFieldDimensions.yPosLeftSideline + fieldSizeThreshold;
  const float yMin = theFieldDimensions.yPosRightSideline - fieldSizeThreshold;
  const float xMax = theFieldDimensions.xPosOpponentGroundline + fieldSizeThreshold;
  const float xMin = theFieldDimensions.xPosOwnGroundline - fieldSizeThreshold;
  return pointInWorld.y < yMax && pointInWorld.y > yMin &&
         pointInWorld.x < xMax && pointInWorld.x > xMin;
}

void ObstacleClusterer::extractCluster(std::list<Vector2<> >& spots, Cluster& cluster)
{
  if(!spots.empty())
  {
    cluster.spots.push_back(spots.back());
    spots.pop_back();
    for(auto it = cluster.spots.begin(); it != cluster.spots.end(); ++it)
    {
      extractFitting(spots, cluster, *it);
    }
  }
}

void ObstacleClusterer::extractFitting(std::list<Vector2<> >& spots, Cluster& cluster, Vector2<> reference)
{
  for(std::list<Vector2<> >::iterator it = spots.begin(); it != spots.end();)
  {
    if(((*it) - reference).absFloat() < mergeDistance)
    {
      cluster.spots.push_back(*it);
      it = spots.erase(it);
    }
    else
    {
      ++it;
    }
  }
}

void ObstacleClusterer::calculateCenterOfMass(Cluster& cluster)
{
  Vector2<> sum(0,0); //the center of mass
  for(const Vector2<>& s : cluster.spots)
  {
    sum += s;
  }
  Vector2<> avg(sum);
  avg *= (1.0f/cluster.spots.size());
  cluster.centerOfMass.x = avg.x;
  cluster.centerOfMass.y = avg.y;
}
