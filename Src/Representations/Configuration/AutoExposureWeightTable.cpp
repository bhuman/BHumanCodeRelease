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
    ASSERT(tables[static_cast<const CameraInfo&>(Blackboard::getInstance()["CameraInfo"]).camera].maxCoeff() <= 100);
}

void AutoExposureWeightTable::draw() const
{
  DEBUG_DRAWING("representation:AutoExposureWeightTable", "drawingOnImage")
    if(Blackboard::getInstance().exists("CameraInfo"))
    {
      const CameraInfo& theCameraInfo = static_cast<const CameraInfo&>(Blackboard::getInstance()["CameraInfo"]);
      const Matrix5uc& table = this->tables[theCameraInfo.camera];

      const int width_5 = static_cast<int>(theCameraInfo.width / table.cols());
      const int width_25 = static_cast<int>(width_5 / table.cols());
      const int height_5 = static_cast<int>(theCameraInfo.height / table.rows());
      const int height_25 = static_cast<int>(height_5 / table.rows());

      const int max = std::max(uint8_t(1u), table.maxCoeff());
      for(int y = 0, y0 = 0, y1 = height_5; y1 <= theCameraInfo.height; y0 += height_5, y1 += height_5, ++y)
        for(int x = 0, x0 = 0, x1 = width_5; x1 <= theCameraInfo.width; x0 += width_5, x1 += width_5, ++x)
        {
          FILLED_RECTANGLE("representation:AutoExposureWeightTable", x0, y0, x1, y1, 5, Drawings::solidPen, ColorRGBA::black, Drawings::solidBrush, ColorRGBA(0, 0, 0, 255 * table(y, x) / max));
          DRAWTEXT("representation:AutoExposureWeightTable", x0 + width_25, y0 + 4 * height_25, height_25 * 2, ColorRGBA::white, "" << table(y, x));
        }
    }
}
