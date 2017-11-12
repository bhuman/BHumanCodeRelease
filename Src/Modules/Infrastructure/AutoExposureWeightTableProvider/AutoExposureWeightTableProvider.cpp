/**
 * @file AutoExposureWeightTableProvider.cpp
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "AutoExposureWeightTableProvider.h"
#include "Tools/ImageProcessing/InImageSizeCalculations.h"
#include "Tools/Math/Transformation.h"

MAKE_MODULE(AutoExposureWeightTableProvider, cognitionInfrastructure)

void AutoExposureWeightTableProvider::update(AutoExposureWeightTable& autoExposureWeightTable)
{
  // take from config - static wise
  if(useStaticConfigValues)
  {
    autoExposureWeightTable = configuredAutoExposureWeightTable;
    return;
  }

  // set to default
  autoExposureWeightTable = resetAutoExposureWeightTable;

  const unsigned compressFactorWidth = static_cast<unsigned>(theCameraInfo.width / autoExposureWeightTable.table.cols());
  const unsigned compressFactorHeight = static_cast<unsigned>(theCameraInfo.height / autoExposureWeightTable.table.rows());

  // ignore cells with body contour
  for(unsigned scanLineIndex = theScanGrid.lowResStart; scanLineIndex < theScanGrid.lines.size(); scanLineIndex += theScanGrid.lowResStep)
  {
    const unsigned x = theScanGrid.lines[scanLineIndex].x / compressFactorWidth;

    int y = theScanGrid.lines[scanLineIndex].yMax / compressFactorHeight;
    if(y < autoExposureWeightTable.table.rows())
      autoExposureWeightTable.table(y, x) = static_cast<unsigned char>(std::min(static_cast<unsigned>(autoExposureWeightTable.table(y, x)), 1u * resetAutoExposureWeightTable.table(y, x) *
        (theScanGrid.lines[scanLineIndex].yMax - y * compressFactorHeight) / compressFactorHeight));

    while(++y < autoExposureWeightTable.table.rows())
      autoExposureWeightTable.table(y, x) = 0;
  }

  //ignore cells above horizon
  for(int x = compressFactorWidth / 2; x < theCameraInfo.width; x += compressFactorWidth)
  {
    Vector2f p(x, 0);
    Vector2f pInH = theImageCoordinateSystem.toHorizonBased(p);
    if(pInH.y() > 0)
      continue;

    int y = static_cast<int>(std::min(-pInH.y() / compressFactorHeight, 1.f * autoExposureWeightTable.table.rows()));

    if(y < autoExposureWeightTable.table.rows() && y >= 0)
      autoExposureWeightTable.table(y, x / compressFactorWidth) = static_cast<unsigned char>(std::max(0, static_cast<int>(autoExposureWeightTable.table(y, x / compressFactorWidth)) - static_cast<int>(1.f * resetAutoExposureWeightTable.table(y, x / compressFactorWidth) *
        (-pInH.y() - y * compressFactorHeight) / compressFactorHeight)));

    while(--y >= 0)
      autoExposureWeightTable.table(y, x / compressFactorWidth) = 0;
  }

  //prefer cell(s) with ball (world model prediction)
  Vector2f pointInImage;
  if(theWorldModelPrediction.ballIsValid && Transformation::robotToImage(theWorldModelPrediction.ballPosition, theCameraMatrix, theCameraInfo, pointInImage)
     && pointInImage.x() >= 0 && pointInImage.y() >= 0 && pointInImage.x() < theCameraInfo.width && pointInImage.y() < theCameraInfo.height)
  {
    autoExposureWeightTable.table /= factorReduceAllBeforeApplyBallPrediction;
    autoExposureWeightTable.table(static_cast<int>(pointInImage.y() / compressFactorHeight), static_cast<int>(pointInImage.x() / compressFactorWidth)) = 100;
    const float radiusApproximated = IISC::getImageBallRadiusByLowestPoint(pointInImage, theCameraInfo, theCameraMatrix, theBallSpecification);
    if(pointInImage.y() - radiusApproximated > 0)
      autoExposureWeightTable.table(static_cast<int>((pointInImage.y() - radiusApproximated) / compressFactorHeight), static_cast<int>(pointInImage.x() / compressFactorWidth)) = 100;
  }
}
