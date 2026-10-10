/**
 * @file CustomCalibrationLinesProvider.h
 *
 * @author Adam Cihasev
 */

#pragma once

#include <optional>
#include "Framework/Module.h"
#include "Representations/Configuration/CalibrationRequest.h"
#include "Representations/Configuration/CameraCalibrationStatus.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/FieldPercepts/CustomCalibrationLines.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"

MODULE(CustomCalibrationLinesProvider,
{,
  REQUIRES(CameraCalibrationStatus),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(CalibrationRequest),
  PROVIDES(CustomCalibrationLines),
});

class CustomCalibrationLinesProvider : public CustomCalibrationLinesProviderBase
{
public:
  void update(CustomCalibrationLines& theCustomCalibrationLines) override;
  void createCalibrationLine(CustomCalibrationLines& theCustomCalibrationLines);
  void drawCalibrationLines(const CustomCalibrationLines& theCustomCalibrationLines) const;
  bool areInputsValid() const;
  bool areInputsComplete() const;

  void drawInput() const;
  void drawPoint(const Vector2f& point, const ColorRGBA& color, const std::string label) const;
  void drawLine(const Vector2f& from, const Vector2f& to, const ColorRGBA& color) const;

private:

  bool sampling; /**< Are we collecting samples? */
  Vector2f pointA = Vector2f::Zero(); /**< The first point of line in the image. */
  Vector2f pointB = Vector2f::Zero(); /**< The second point of line in the image. */
  CustomCalibrationLines::Type lineType = CustomCalibrationLines::undefined; /**< The type of the line*/

  bool submitLine = false; /**< Whether the line should be created. */
};
