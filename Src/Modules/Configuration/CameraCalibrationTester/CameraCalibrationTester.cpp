/**
 * @file CameraCalibrationTester.cpp
 * @author René Schröder
 */

#include "CameraCalibrationTester.h"
#include "Tools/Debugging/DebugDrawings.h"

#include <limits>

MAKE_MODULE(CameraCalibrationTester, cognitionInfrastructure)

void CameraCalibrationTester::update(CameraCalibrationRating& theCameraCalibrationRating)
{
  DECLARE_DEBUG_DRAWING("module:CameraCalibrationTester:samples", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CameraCalibrationTester:fieldLines", "drawingOnImage");

  if(aborted)
    theCameraCalibrationRating.wasAborted = true;

  theCameraCalibrationRating.setResolution = [this](const bool& curCamIsUpper)
  {
    setResolution(curCamIsUpper);
  };

  theCameraCalibrationRating.check = [this](const bool& curCamIsUpper)
  {
    testPoints(curCamIsUpper);
  };

  theCameraCalibrationRating.evaluate = [&]()
  {
    evaluate(theCameraCalibrationRating);
  };

  theCameraCalibrationRating.evalOutput = [&]()
  {
    evalOutput(theCameraCalibrationRating.isGood, nullptr, nullptr);
  };

  draw();
}

void CameraCalibrationTester::evaluate(CameraCalibrationRating& theCameraCalibrationRating) const
{
  float upperSum = 0.f;
  for(float f : valuesUpper)
    upperSum += f;
  float lowerSum = 0.f;
  for(float f : valuesLower)
    lowerSum += f;
  const float errorUpper = upperSum / valuesUpper.size();
  const float errorLower = lowerSum / valuesLower.size();
  if(errorUpper > maxErrorUpper || errorLower > maxErrorLower)
  {
    evalOutput(false, &errorUpper, &errorLower);
    theCameraCalibrationRating.wasEvaluated = true;
    theCameraCalibrationRating.isGood = false;
  }
  else
  {
    evalOutput(true, &errorUpper, &errorLower);
    theCameraCalibrationRating.wasEvaluated = true;
    theCameraCalibrationRating.isGood = true;
  }
}

void CameraCalibrationTester::evalOutput(const bool& isGood, const float* errorUpper, const float* errorLower) const
{
  if(isGood)
  {
    SystemCall::playSound("Good.wav");
    OUTPUT_TEXT("CameraCalibration is good.");
  }
  else
  {
    SystemCall::playSound("Bad.wav");
    OUTPUT_TEXT("CameraCalibration is bad!");
  }
  if(errorLower != nullptr && errorUpper != nullptr)
    OUTPUT_TEXT("upper: " << *errorUpper << ", lower: " << *errorLower);
}

void CameraCalibrationTester::draw() const
{
  const Pose2f robotPoseInv = theRobotPose.inversePose;
  for(FieldDimensions::LinesTable::Line lineOnField : theFieldDimensions.fieldLines.lines)
  {
    // transform the line in robot relative coordinates
    lineOnField.from = robotPoseInv * lineOnField.from;
    lineOnField.to = robotPoseInv * lineOnField.to;
    Geometry::Line lineInImage;
    if(projectLineOnFieldIntoImage(Geometry::Line(lineOnField.from, lineOnField.to - lineOnField.from),
                                   theCameraMatrix, theCameraInfo, lineInImage))
    {
      LINE("module:CameraCalibrationTester:fieldLines", lineInImage.base.x(), lineInImage.base.y(),
           (lineInImage.base + lineInImage.direction).x(), (lineInImage.base + lineInImage.direction).y(),
           1, Drawings::solidPen, ColorRGBA::black);
    }
  }
}

void CameraCalibrationTester::setResolution(const bool& curCamIsUpper)
{
  /** Changing the resolution is only allowed on a physical robot */
  if(SystemCall::getMode() == SystemCall::Mode::physicalRobot)
    nextResolution = curCamIsUpper ? NextResolution::hiResUpper : NextResolution::hiResLower;
}

void CameraCalibrationTester::update(CameraResolutionRequest& theCameraResolutionRequest)
{
  CameraResolution::Resolutions nextUpper, nextLower;
  switch(nextResolution)
  {
    case NextResolution::none:
      return;
    case NextResolution::hiResUpper:
      nextUpper = CameraResolution::Resolutions::w640h480;
      nextLower = CameraResolution::Resolutions::w320h240;
      break;
    case NextResolution::hiResLower:
      nextUpper = CameraResolution::Resolutions::w320h240;
      nextLower = CameraResolution::Resolutions::w640h480;
      break;
    default:
      FAIL("Unknown resolution.");
      return;
  }
  if(theCameraResolution.resolutionUpper != nextUpper || theCameraResolution.resolutionLower != nextLower)
  {
    if(!theCameraResolutionRequest.setRequest(nextUpper, nextLower))
    {
      OUTPUT_TEXT("Changing the resolution is not permitted! Test aborted...");
      abort();
    }
  }
  nextResolution = NextResolution::none;
}

void CameraCalibrationTester::testPoints(const bool& curCamIsUpper)
{
  // Vergleiche für eine Anzahl von Samples von den FieldLines den Abstand zu den FieldDimensions
  std::vector<Vector2i> samples;
  calcSamples(samples);
  const Pose2f robotPoseInv = theRobotPose.inversePose;
  for(const Vector2i& spot : samples)
  {
    float minimum = std::numeric_limits<float>::max();
    for(FieldDimensions::LinesTable::Line lineOnField : theFieldDimensions.fieldLines.lines)
    {
      // transform the line in robot relative coordinates
      lineOnField.from = robotPoseInv * lineOnField.from;
      lineOnField.to = robotPoseInv * lineOnField.to;
      Geometry::Line lineInImage;
      float distance;
      if(!projectLineOnFieldIntoImage(Geometry::Line(lineOnField.from, lineOnField.to - lineOnField.from),
                                      theCameraMatrix, theCameraInfo, lineInImage))
        distance = 1000000.f;
      else
        distance = Geometry::getDistanceToEdge(lineInImage, spot.cast<float>());
      if(distance < minimum)
        minimum = distance;
    }
    if(curCamIsUpper)
      valuesUpper.push_back(minimum);
    else
      valuesLower.push_back(minimum);
  }
}

void CameraCalibrationTester::calcSamples(std::vector<Vector2i>& samples) const
{
  for(const LinesPercept::Line& line : theLinesPercept.lines)
  {
    for(const Vector2i& point : line.spotsInImg)
    {
      if(point.x() < 0 || point.x() >= theCameraInfo.width
         || point.y() < 0 || point.y() >= theCameraInfo.height)
        continue;
      bool goodDistanceToOtherPoints = true;
      // Check distance to other points
      for(const Vector2i& s : samples)
        if((s - point).squaredNorm() < minDis * minDis)
          goodDistanceToOtherPoints = false;
      if(goodDistanceToOtherPoints)
      {
        samples.push_back(point);
        CROSS("module:CameraCalibrationTester:samples", point.x(), point.y(), 5, 1, Drawings::solidPen, ColorRGBA::blue);
      }
    }
  }
}

bool CameraCalibrationTester::projectLineOnFieldIntoImage(const Geometry::Line& lineOnField,
  const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, Geometry::Line& lineInImage) const
{
  // Taken from the AutomaticCameraCalibrator

  const float& f = cameraInfo.focalLength;
  const Pose3f cameraMatrixInv = cameraMatrix.inverse();

  // TODO more elegant solution directly using the direction of the line?

  // start and end point of the line
  const Vector2f& p1 = lineOnField.base;
  const Vector2f p2 = p1 + lineOnField.direction;
  Vector3f p1Camera(p1.x(), p1.y(), 0);
  Vector3f p2Camera(p2.x(), p2.y(), 0);

  // points are transformed into camera coordinates
  p1Camera = cameraMatrixInv * p1Camera;
  p2Camera = cameraMatrixInv * p2Camera;

  // handle the case that points can lie behind the camera plane
  const bool p1Behind = p1Camera.x() < cameraInfo.focalLength;
  const bool p2Behind = p2Camera.x() < cameraInfo.focalLength;
  if(p1Behind && p2Behind)
    return false;
  else if(!p1Behind && !p2Behind)
  {
    // both rays can be simply intersected with the image plane
    p1Camera /= (p1Camera.x() / f);
    p2Camera /= (p2Camera.x() / f);
  }
  else
  {
    // if one point lies behind the camera and the other in front,
    // there must be an intersection of the connective line with the image plane
    const Vector3f direction = p1Camera - p2Camera;
    const float scale = (f - p1Camera.x()) / direction.x();
    const Vector3f intersection = p1Camera + direction * scale;
    if(p1Behind)
    {
      p1Camera = intersection;
      p2Camera /= (p2Camera.x() / f);
    }
    else
    {
      p2Camera = intersection;
      p1Camera /= (p1Camera.x() / f);
    }
  }
  const Vector2f p1Result(cameraInfo.opticalCenter.x() - p1Camera.y(), cameraInfo.opticalCenter.y() - p1Camera.z());
  const Vector2f p2Result(cameraInfo.opticalCenter.x() - p2Camera.y(), cameraInfo.opticalCenter.y() - p2Camera.z());
  lineInImage.base = p1Result;
  lineInImage.direction = p2Result - p1Result;
  return true;
}

void CameraCalibrationTester::abort()
{
  valuesUpper.clear();
  valuesLower.clear();
  nextResolution = NextResolution::none;
  aborted = true;
}
