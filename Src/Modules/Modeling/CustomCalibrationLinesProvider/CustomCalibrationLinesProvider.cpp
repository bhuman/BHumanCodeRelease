/**
 * @file CustomCalibrationLinesProvider.cpp
 *
 * @author Adam Cihasev
 */

#include "CustomCalibrationLinesProvider.h"
#include "Tools/Math/Transformation.h"

MAKE_MODULE(CustomCalibrationLinesProvider);

void CustomCalibrationLinesProvider::update(CustomCalibrationLines& theCustomCalibrationLines)
{
  DECLARE_DEBUG_DRAWING("module:CustomCalibrationLinesProvider:upper", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CustomCalibrationLinesProvider:lower", "drawingOnImage");

  MODIFY_ONCE("module:CustomCalibrationLinesProvider:pointACamera", pointACamera);
  MODIFY_ONCE("module:CustomCalibrationLinesProvider:pointBCamera", pointBCamera);
  MODIFY_ONCE("module:CustomCalibrationLinesProvider:pointA", pointA);
  MODIFY_ONCE("module:CustomCalibrationLinesProvider:pointB", pointB);
  MODIFY_ONCE("module:CustomCalibrationLinesProvider:lineType", lineType);

  DEBUG_RESPONSE_ONCE("module:CustomCalibrationLinesProvider:submitLine")
  {
    submitLine = true;
  }

  if(theCameraCalibrationStatus.sampleConfigurationStatus == SampleConfigurationStatus::finished || !theGroundContactState.contact)
  {
    for(auto& line : theCustomCalibrationLines.lines)
    {
      line = LinesPercept::Line(Vector2f::Zero(), Vector2f::Zero());
    }
    sampleCamera.reset();
    pointA = Vector2f::Zero();
    pointB = Vector2f::Zero();
    lineType = CustomCalibrationLines::undefined;
    pointACamera = CameraInfo::upper;
    return;
  }

  if(theCalibrationRequest.targetState == CameraCalibrationStatus::State::recordSamples)
  {
    sampleCamera = theCalibrationRequest.sampleConfigurationRequest->camera;
  }

  if(submitLine && theCalibrationRequest.sampleConfigurationRequest && theCalibrationRequest.sampleConfigurationRequest->camera == theCameraInfo.camera)
  {
    if(areInputsValid())
    {
      createCalibrationLine(theCustomCalibrationLines);
    }
    submitLine = false;
  }
  drawInput();
  drawCalibrationLines(theCustomCalibrationLines);
}

void CustomCalibrationLinesProvider::createCalibrationLine(CustomCalibrationLines& theCustomCalibrationLines)
{
  // Sort points by x coordinate
  if(pointA.x() > pointB.x())
  {
    std::swap(pointA, pointB);
    // Cameras must match, so we don't need to swap camera info
  }

  LinesPercept::Line oldLine = theCustomCalibrationLines.lines[lineType];
  LinesPercept::Line newLine;

  if(!Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(pointA), theCameraMatrix, theCameraInfo, newLine.firstField) ||
     !Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(pointB), theCameraMatrix, theCameraInfo, newLine.lastField))
  {
    OUTPUT_ERROR("Failed to convert image points to field coordinates.");
    return;
  }

  newLine.line.base = newLine.firstField;
  newLine.line.direction = newLine.lastField - newLine.firstField;

  newLine.firstImg = pointA.cast<int>();
  newLine.lastImg = pointB.cast<int>();

  theCustomCalibrationLines.lines[lineType] = newLine;

  OUTPUT_TEXT("Created " << TypeRegistry::getEnumName(lineType)
              << " from (" << pointA.x() << ", " << pointA.y() << ") "
              << "to (" << pointB.x() << ", " << pointB.y() << ") "
              << "in the " << (pointACamera == CameraInfo::upper ? "upper" : "lower") << " camera.");
}

void CustomCalibrationLinesProvider::drawCalibrationLines(const CustomCalibrationLines& theCustomCalibrationLines) const
{
  for(size_t i = 0; i < theCustomCalibrationLines.lines.size(); ++i)
  {
    const LinesPercept::Line& line = theCustomCalibrationLines.lines[i];
    if(line.isEmpty())
      continue;

    ColorRGBA color;
    switch(i)
    {
      case CustomCalibrationLines::closeGoalAreaConnectingLine:
        color = ColorRGBA::cyan;
        break;
      case CustomCalibrationLines::innerGoalAreaParallelLine:
        color = ColorRGBA::magenta;
        break;
      case CustomCalibrationLines::outerGoalAreaParallelLine:
        color = ColorRGBA::orange;
        break;
      case CustomCalibrationLines::farGoalAreaConnectingLine:
        color = ColorRGBA::brown;
        break;
      default:
        color = ColorRGBA::blue;
        break;
    }
    drawLine(line.firstImg.cast<float>(), line.lastImg.cast<float>(), pointACamera, color);
  }
}

bool CustomCalibrationLinesProvider::areInputsValid() const
{
  if(!areInputsComplete())
  {
    OUTPUT_TEXT("Inputs are not complete: pointA or pointB is zero.");
    return false;
  }
  if(isCameraMismatch())
  {
    OUTPUT_TEXT("Camera mismatch: pointA and pointB are in different cameras.");
    return false;
  }
  if(!doPointsMatchSampleCamera())
  {
    OUTPUT_TEXT("Points do not match the sample camera. Is the calibration even running?");
    return false;
  }
  return true;
}

bool CustomCalibrationLinesProvider::areInputsComplete() const
{
  return !pointA.isZero() && !pointB.isZero() && lineType != CustomCalibrationLines::undefined;
}

bool CustomCalibrationLinesProvider::isCameraMismatch() const
{
  return !pointA.isZero() && !pointB.isZero() && pointACamera != pointBCamera;
}

bool CustomCalibrationLinesProvider::doPointsMatchSampleCamera() const
{
  if(!sampleCamera.has_value())
  {
    return false;
  }
  if((!pointA.isZero() && pointACamera != *sampleCamera) ||
     (!pointB.isZero() && pointBCamera != *sampleCamera))
  {
    return false;
  }
  return true;
}

void CustomCalibrationLinesProvider::drawInput() const
{
  const bool aSet = !pointA.isZero();
  const bool bSet = !pointB.isZero();

  if(aSet)
  {
    drawPoint(pointA, pointACamera, ColorRGBA::blue, "Point A");
  }

  if(bSet)
  {
    drawPoint(pointB, pointBCamera, ColorRGBA::blue, "Point B");
  }

  if(aSet && bSet && !isCameraMismatch())
  {
    drawLine(pointA, pointB, pointBCamera, ColorRGBA::blue);
  }
}

void CustomCalibrationLinesProvider::drawPoint([[maybe_unused]] const Vector2f& point, CameraInfo::Camera camera, [[maybe_unused]] const ColorRGBA& color, [[maybe_unused]] const std::string label) const
{
  if(camera == CameraInfo::upper)
  {
    DRAW_TEXT("module:CustomCalibrationLinesProvider:upper", point.x(), point.y() + 15, 10, ColorRGBA::black, label);
    DOT("module:CustomCalibrationLinesProvider:upper", point.x(), point.y(), color, color);
  }
  else
  {
    DRAW_TEXT("module:CustomCalibrationLinesProvider:lower", point.x(), point.y() + 15, 10, ColorRGBA::black, label);
    DOT("module:CustomCalibrationLinesProvider:lower", point.x(), point.y(), color, color);
  }
}

void CustomCalibrationLinesProvider::drawLine([[maybe_unused]] const Vector2f& from, [[maybe_unused]] const Vector2f& to, CameraInfo::Camera camera, [[maybe_unused]] const ColorRGBA& color) const
{
  if(camera == CameraInfo::upper)
    LINE("module:CustomCalibrationLinesProvider:upper", from.x(), from.y(), to.x(), to.y(), 1, Drawings::PenStyle::solidPen, color);
  else
    LINE("module:CustomCalibrationLinesProvider:lower", from.x(), from.y(), to.x(), to.y(), 1, Drawings::PenStyle::solidPen, color);
}
