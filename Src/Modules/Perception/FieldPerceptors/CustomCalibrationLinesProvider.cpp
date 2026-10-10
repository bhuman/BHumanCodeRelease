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
  DECLARE_DEBUG_DRAWING("module:CustomCalibrationLinesProvider", "drawingOnImage");

  MODIFY_ONCE("module:CustomCalibrationLinesProvider:pointA", pointA);
  MODIFY_ONCE("module:CustomCalibrationLinesProvider:pointB", pointB);
  MODIFY_ONCE("module:CustomCalibrationLinesProvider:lineType", lineType);

  DEBUG_RESPONSE_ONCE("module:CustomCalibrationLinesProvider:submitLine")
  {
    submitLine = true;
  }

  if(theCameraCalibrationStatus.sampleConfigurationStatus == SampleConfigurationStatus::finished)
  {
    for(auto& line : theCustomCalibrationLines.lines)
    {
      line = LinesPercept::Line(Vector2f::Zero(), Vector2f::Zero());
    }
    sampling = false;
    pointA = Vector2f::Zero();
    pointB = Vector2f::Zero();
    lineType = CustomCalibrationLines::undefined;
    return;
  }

  if(theCalibrationRequest.targetState == CameraCalibrationStatus::State::recordSamples)
  {
    sampling = true;
  }

  if(submitLine && theCalibrationRequest.sampleConfigurationRequest)
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

  if(!Transformation::imageToRobot(pointA, theCameraMatrix, theCameraInfo, newLine.firstField) ||
     !Transformation::imageToRobot(pointB, theCameraMatrix, theCameraInfo, newLine.lastField))
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
              << "to (" << pointB.x() << ", " << pointB.y() << ").");
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
      case CustomCalibrationLines::goalLineGoalAreaParallelLine:
        color = ColorRGBA::magenta;
        break;
      case CustomCalibrationLines::innerGoalAreaParallelLine:
        color = ColorRGBA::orange;
        break;
      case CustomCalibrationLines::farGoalAreaConnectingLine:
        color = ColorRGBA::brown;
        break;
      default:
        color = ColorRGBA::blue;
        break;
    }
    drawLine(line.firstImg.cast<float>(), line.lastImg.cast<float>(), color);
  }
}

bool CustomCalibrationLinesProvider::areInputsValid() const
{
  if(!areInputsComplete())
  {
    OUTPUT_TEXT("Inputs are not complete: pointA or pointB is zero.");
    return false;
  }
  return true;
}

bool CustomCalibrationLinesProvider::areInputsComplete() const
{
  return !pointA.isZero() && !pointB.isZero() && lineType != CustomCalibrationLines::undefined;
}

void CustomCalibrationLinesProvider::drawInput() const
{
  const bool aSet = !pointA.isZero();
  const bool bSet = !pointB.isZero();

  if(aSet)
  {
    drawPoint(pointA, ColorRGBA::blue, "Point Ctrl");
  }

  if(bSet)
  {
    drawPoint(pointB, ColorRGBA::blue, "Point Shift");
  }

  if(aSet && bSet)
  {
    drawLine(pointA, pointB, ColorRGBA::blue);
  }
}

void CustomCalibrationLinesProvider::drawPoint([[maybe_unused]] const Vector2f& point, [[maybe_unused]] const ColorRGBA& color, [[maybe_unused]] const std::string label) const
{
  DRAW_TEXT("module:CustomCalibrationLinesProvider", point.x(), point.y() + 15, 10, ColorRGBA::black, label);
  DOT("module:CustomCalibrationLinesProvider", point.x(), point.y(), 0, color, color);
}

void CustomCalibrationLinesProvider::drawLine([[maybe_unused]] const Vector2f& from, [[maybe_unused]] const Vector2f& to, [[maybe_unused]] const ColorRGBA& color) const
{
  LINE("module:CustomCalibrationLinesProvider", from.x(), from.y(), to.x(), to.y(), 1, Drawings::PenStyle::solidPen, color);
}
