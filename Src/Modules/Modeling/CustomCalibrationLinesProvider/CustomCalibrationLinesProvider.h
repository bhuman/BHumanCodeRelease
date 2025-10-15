/**
 * @file CustomCalibrationLinesProvider.h
 *
 * @author Adam Cihasev
 */

#pragma once

#include <optional>
#include "Framework/Module.h"
#include "Representations/Modeling/CustomCalibrationLines.h"
#include "Representations/Configuration/CameraCalibrationStatus.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Configuration/CalibrationRequest.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"

MODULE(CustomCalibrationLinesProvider,
{,
  REQUIRES(CameraCalibrationStatus),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(CalibrationRequest),
  REQUIRES(GroundContactState),
  REQUIRES(ImageCoordinateSystem),
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
  bool isCameraMismatch() const;
  bool doPointsMatchSampleCamera() const;

  void drawInput() const;
  void drawPoint(const Vector2f& point, CameraInfo::Camera camera, const ColorRGBA& color, const std::string label) const;
  void drawLine(const Vector2f& from, const Vector2f& to, CameraInfo::Camera camera, const ColorRGBA& color) const;

private:

  std::optional<CameraInfo::Camera> sampleCamera; /**< The camera used for the sample configuration. */
  CameraInfo::Camera pointACamera = CameraInfo::upper; /**< The camera of the first point of the line. */
  CameraInfo::Camera pointBCamera = CameraInfo::upper; /**< The camera of the second point of the line. */
  Vector2f pointA = Vector2f::Zero(); /**< The first point of line in the image. */
  Vector2f pointB = Vector2f::Zero(); /**< The second point of line in the image. */
  CustomCalibrationLines::Type lineType = CustomCalibrationLines::undefined; /**< The type of the line*/

  bool submitLine = false; /**< Whether the line should be created. */
};
