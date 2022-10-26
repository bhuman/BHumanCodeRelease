/**
 * @file ObstaclesPolygonProvider.h
 *
 * @author Nele Matschull
 * @author Jonah Jaeger
 * @author Yannik Meinken
 */

#include "Representations/Perception/ObstaclesPercepts/ObstaclesPolygonPercept.h"
#include "Math/Eigen.h"
#include "Math/Geometry.h"
#include "Framework/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Perception/ImagePreprocessing/ColorScanLineRegions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/BallPercepts/BallPercept.h"

#include <functional>

MODULE(ObstaclesPolygonProvider,
{,
  REQUIRES(ColorScanLineRegionsHorizontal),
  REQUIRES(ColorScanLineRegionsVerticalClipped),
  REQUIRES(FieldDimensions),
  REQUIRES(CameraMatrix),
  REQUIRES(CameraInfo),
  REQUIRES(BallPercept),
  PROVIDES(ObstaclesPolygonPercept),
  DEFINES_PARAMETERS(
  {,
    //(type)(value) name, /**< description. */
    (int)(2) maxDeviationFromLine, /**< maximal deviation from a point to the line between the neighboring points for the point to be discarded*/
    (float)(2.f) ballRadiusFactor, /**< factor of the radius around the ball to be filtered as free*/
    (double)(0.00001) lengthWeigthFactor, /*Factor how fast the weigthing because of length should be*/
    (int)(20) distanceToBorder, /*Max distance a obstacle can be at the Border (on the Image)*/
  }),
});

class ObstaclesPolygonProvider : public ObstaclesPolygonProviderBase
{
  //forward/backward(scanline(points on this scanline(start(x,y), end(x,y))))
  //list because many deletions at the front and middle
  std::array< std::vector<std::list<std::array<Vector2f, 2>>>, 2> nonGreenRegions;
  std::vector<std::list<std::array<Vector2f, 2>>> groundNonGreenRegions;

  const float maxFieldLineWidth = 1.5f * theFieldDimensions.fieldLinesWidth;  /**< maximal width of a field line*/
  const float minFieldLineWidth = 0.7f * theFieldDimensions.fieldLinesWidth; /**< minimal width of a field line*/

  /**
   *
   */
  void update(ObstaclesPolygonPercept& theObstaclesPolygonPercept) override;

private:

  /**
   * computes a orthogonal vector.
   * length of the output vector can vary but is at least 1
   *
   * @param vec input vector
   * @return a vector that is orthogonal to the input vector
   */
  Vector2f orthogonalVector(Vector2f vec) const;

  bool overlap(std::array<Vector2f, 2> line1, std::array<Vector2f, 2> line2) const;

  /**
   * computes whether or not to lines completely overlap
   *
   * @param overlapped shorter line that gets overlapped by the other
   * @param overlapping longer line that overlap the shorter
   *
   * @return true if both ends of the shorter line (overlapped) overlap with the longer line
   */
  bool completeOverlap(std::array<Vector2f, 2> overlapped, std::array<Vector2f, 2> overlapping) const;

  double lengthWeight(std::array<Vector2f, 2> firstLine, std::array<Vector2f, 2> secondLine)const;
};
