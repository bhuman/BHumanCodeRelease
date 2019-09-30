/**
 * @file AutoExposureWeightTable.cpp
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "AutoExposureWeightTable.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Module/Blackboard.h"

void AutoExposureWeightTable::verify() const
{
  if(Blackboard::getInstance().exists("CameraInfo"))
    ASSERT(tables[static_cast<const CameraInfo&>(Blackboard::getInstance()["CameraInfo"]).camera].maxCoeff() <= maxWeight);
}

void AutoExposureWeightTable::draw() const
{
  DEBUG_DRAWING("representation:AutoExposureWeightTable", "drawingOnImage")
    if(Blackboard::getInstance().exists("CameraInfo"))
    {
      const CameraInfo& theCameraInfo = static_cast<const CameraInfo&>(Blackboard::getInstance()["CameraInfo"]);
      const Table& table = this->tables[theCameraInfo.camera];

      const int cellWidth = static_cast<int>(theCameraInfo.width / table.cols());
      const int cellHeight = static_cast<int>(theCameraInfo.height / table.rows());

      for(int y = 0, y0 = 0, y1 = cellHeight; y1 <= theCameraInfo.height; y0 += cellHeight, y1 += cellHeight, ++y)
        for(int x = 0, x0 = 0, x1 = cellWidth; x1 <= theCameraInfo.width; x0 += cellWidth, x1 += cellWidth, ++x)
        {
          RECTANGLE("representation:AutoExposureWeightTable", x0, y0, x1, y1, 5, Drawings::solidPen, ColorRGBA::black);
          DRAWTEXT("representation:AutoExposureWeightTable", x0 + cellWidth / 5, y0 + 4 * cellHeight / 5, 2 * cellHeight / 5, ColorRGBA::white, "" << table(y, x));
        }
    }
}
