/**
 * @file AutoExposureWeightTableProvider.cpp
 *
 * This file implements a module that determines how different areas of a camera image should be
 * weighted when computing the auto exposure. The auto exposure weight table consists of 4x4
 * weights for corresponding image areas. The module limits the area considered to a
 * definable distance from the robots. In addition, the robot's body is excluded. If the ball
 * is expected to be in the image, it will influence the exposure computation with a
 * configurable ratio.
 *
 * @author Thomas Röfer
 */

#include "AutoExposureWeightTableProvider.h"
#include "Platform/SystemCall.h"
#include "Tools/Boundary.h"
#include "Tools/Global.h"
#include "Tools/ImageProcessing/InImageSizeCalculations.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Settings.h"
#include <algorithm>
#include <cmath>

MAKE_MODULE(AutoExposureWeightTableProvider, infrastructure)

void AutoExposureWeightTableProvider::update(AutoExposureWeightTable& theAutoExposureWeightTable)
{
  AutoExposureWeightTable::Table& table = theAutoExposureWeightTable.tables[theCameraInfo.camera];
  
  DEBUG_RESPONSE("module:AutoExposureWeightTableProvider:alwaysChange")
  {
    for(int i = 0; i < table.size(); ++i)
      table.data()[i] = (table.data()[i] + 1) % (AutoExposureWeightTable::maxWeight + 1);
    return;
  }

  // Use static table or reset all weights to 0.
  if(useStaticTables)
    table = staticTables[theCameraInfo.camera];
  else
  {
    table = AutoExposureWeightTable::Table::Zero();
    ASSERT(theCameraInfo.width % table.cols() == 0);
    ASSERT(theCameraInfo.height % table.rows() == 0);
    const int cellWidth = static_cast<int>(theCameraInfo.width / table.cols());
    const int cellHeight = static_cast<int>(theCameraInfo.height / table.rows());
    std::vector<int> rows(table.cols());
    int numOfFieldCells = 0;

    // Set weights above body contour and below distance threshold to 1.
    for(int col = 0; col < table.cols(); ++col)
    {
      int y = theCameraInfo.height;
      theBodyContour.clipBottom(col * cellWidth, y);
      theBodyContour.clipBottom((col + 1) * cellWidth - 1, y);

      int& row = rows[col];
      row = y / cellHeight - 1;
      Vector2f onField;
      while(row >= 0
            && Transformation::imageToRobot(col * cellWidth, row * cellHeight, theCameraMatrix, theCameraInfo, onField)
            && onField.norm() <= distanceThreshold
            && Transformation::imageToRobot((col + 1) * cellWidth - 1, row * cellHeight, theCameraMatrix, theCameraInfo, onField)
            && onField.norm() <= distanceThreshold)
      {
        table(row--, col) = 1;
        ++numOfFieldCells;
      }
    }

    // Prioritize area including the ball if its detection is not too old
    if(theFrameInfo.getTimeSince(theWorldModelPrediction.timeWhenBallLastSeen) <= ballValidDelay)
    {
      // Project ball center to image.
      Pose3f cameraAboveBallCenter = theCameraMatrix;
      cameraAboveBallCenter.translation.z() -= theBallSpecification.radius;
      Vector2f ballCenter;
      if(Transformation::robotToImage(theWorldModelPrediction.ballPosition, cameraAboveBallCenter, theCameraInfo, ballCenter))
      {
        // Determine expected radius in image and compute corresponding area.
        float radius = IISC::getImageBallRadiusByCenter(ballCenter, theCameraInfo, theCameraMatrix, theBallSpecification);
        Boundaryf ballArea(Rangef(ballCenter.x() - radius, ballCenter.x() + radius),
                           Rangef(ballCenter.y() - radius, ballCenter.y() + radius));

        // Is any part of the area inside the image?
        if(ballArea.x.max >= 0 && ballArea.x.min < theCameraInfo.width
           && ballArea.y.max >= 0 && ballArea.y.min < theCameraInfo.height)
        {
          // Compute ball limits in weight table coordinates.
          Boundaryi ballLimits(Rangei(std::max(static_cast<int>(ballArea.x.min), 0) / cellWidth,
                                      std::min(static_cast<int>(ballArea.x.max), theCameraInfo.width - 1) / cellWidth),
                               Rangei(std::max(static_cast<int>(ballArea.y.min), 0) / cellHeight,
                                      std::min(static_cast<int>(ballArea.y.max), theCameraInfo.height - 1) / cellHeight));

          // Remove default weight from ball area
          for(int row = ballLimits.y.min; row <= ballLimits.y.max; ++row)
            for(int col = ballLimits.x.min; col <= ballLimits.x.max; ++col)
              if(table(row, col))
              {
                table(row, col) = 0;
                --numOfFieldCells;
              }

          unsigned char ballCellWeight = 1;

          if(numOfFieldCells)
          {
            int numOfBallCells = (ballLimits.x.getSize() + 1) * (ballLimits.y.getSize() + 1);
            float fieldWeightRatio = 1.f - ballWeightRatio;
            float ballWeight = std::round(numOfFieldCells * ballWeightRatio / fieldWeightRatio / numOfBallCells);
            if(ballWeight > static_cast<float>(AutoExposureWeightTable::maxWeight))
              table.setZero(); // ballCellWeight is still 1, everything else is zero
            else
              ballCellWeight = static_cast<unsigned char>(std::max(1, std::min(static_cast<int>(AutoExposureWeightTable::maxWeight),
                                                                               static_cast<int>(ballWeight))));
          }

          // Fill ball area with ball weights
          for(int row = ballLimits.y.min; row <= ballLimits.y.max; ++row)
            for(int col = ballLimits.x.min; col <= ballLimits.x.max; ++col)
              table(row, col) = ballCellWeight;

          // Skip special handling of all weights being 0, because this cannot be the case anymore.
          numOfFieldCells = 1;
        }
      }
    }

    // If no weight was set, add some weights around the body contour.
    if(numOfFieldCells == 0)
    {
      for(int col = 0; col < table.cols(); ++col)
        table(std::max(rows[col], 0), col) = 1;
    }
  }
}
