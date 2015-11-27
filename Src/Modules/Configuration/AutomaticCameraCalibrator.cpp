/**
 * @file AutomaticCameraCalibrator.h
 *
 * This file implements a module that can provide a automatic camera calibration.
 *
 * @author Dana Jenett, Alexis Tsogias
 */

#include "AutomaticCameraCalibrator.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Approx.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Settings.h"
#include <limits>

using namespace std;

MAKE_MODULE(AutomaticCameraCalibrator, cognitionInfrastructure)

AutomaticCameraCalibrator::AutomaticCameraCalibrator() : state(Idle)
{
  states[Idle] = std::bind(&AutomaticCameraCalibrator::idle, this);
  states[Init] = std::bind(&AutomaticCameraCalibrator::init, this);
  states[MoveHead] = std::bind(&AutomaticCameraCalibrator::moveHead, this);
  states[WaitForHeadToReachPosition] = std::bind(&AutomaticCameraCalibrator::waitForHeadToReachPosition, this);
  states[WaitForAccumulate] = std::bind(&AutomaticCameraCalibrator::idle, this);
  states[Accumulate] = std::bind(&AutomaticCameraCalibrator::accumulate, this);
  states[WaitForUser] = std::bind(&AutomaticCameraCalibrator::waitForUser, this);
  states[WaitForOptimize] = std::bind(&AutomaticCameraCalibrator::idle, this);
  states[Optimize] = std::bind(&AutomaticCameraCalibrator::optimize, this);
}

AutomaticCameraCalibrator::~AutomaticCameraCalibrator()
{
  if(optimizer)
    delete optimizer;
}

void AutomaticCameraCalibrator::idle()
{
  // Do nothing
}

void AutomaticCameraCalibrator::init()
{
  startingCameraCalibration = theCameraCalibration;

  swapCameras(startCamera);

  firstHeadPans.clear();
  secondHeadPans.clear();
  firstHeadPans.resize(headPans.size());
  secondHeadPans.resize(headPans.size());
  if(startCamera == CameraInfo::upper)
  {
    std::reverse_copy(headPans.begin(), headPans.end(), firstHeadPans.begin());
    std::copy(headPans.begin(), headPans.end(), secondHeadPans.begin());
  }
  else
  {
    std::copy(headPans.begin(), headPans.end(), firstHeadPans.begin());
    std::reverse_copy(headPans.begin(), headPans.end(), secondHeadPans.begin());
  }
  headPositionPanBuffer.clear();
  headPositionTiltBuffer.clear();

  if(optimizer != nullptr)
  {
    delete optimizer;
    optimizer = nullptr;
  }
  successiveConvergations = 0;
  framesToWait = 0;
  samples.clear();

  state = MoveHead;
}

void AutomaticCameraCalibrator::moveHead()
{
  ASSERT(headSpeed >= 0.3);
  if(firstHeadPans.empty() && secondHeadPans.empty())
  {
    currentHeadPan = 0.0f;
    OUTPUT_TEXT("Accumulation finished. Waiting to optimize...");
    state = WaitForOptimize;
  }
  else if(!firstHeadPans.empty())
  {
    currentHeadPan = firstHeadPans.back();
    firstHeadPans.pop_back();
    state = WaitForHeadToReachPosition;
  }
  else if(firstHeadPans.empty() && secondHeadPans.size() == headPans.size())
  {
    startCamera == CameraInfo::upper ? swapCameras(CameraInfo::lower) : swapCameras(CameraInfo::upper);
    currentHeadPan = secondHeadPans.back();
    secondHeadPans.pop_back();
    state = WaitForHeadToReachPosition;
  }
  else if(firstHeadPans.empty() && !secondHeadPans.empty())
  {
    currentHeadPan = secondHeadPans.back();
    secondHeadPans.pop_back();
    state = WaitForHeadToReachPosition;
  }
  currentHeadTilt = currentCamera == CameraInfo::upper ? upperCameraHeadTilt : lowerCameraHeadTilt;

  nextHeadAngleRequest.pan = currentHeadPan;
  nextHeadAngleRequest.tilt = currentHeadTilt;
  nextHeadAngleRequest.speed = headSpeed;
}

void AutomaticCameraCalibrator::waitForHeadToReachPosition()
{
  headPositionPanBuffer.push_front(theJointAngles.angles[Joints::headYaw]);
  headPositionTiltBuffer.push_front(theJointAngles.angles[Joints::headPitch]);

  auto areBuffersFilled = [&]()
  {
    return headPositionPanBuffer.full() && headPositionTiltBuffer.full();
  };

  auto headReachedPosition = [&]()
  {
    return Approx::isEqual(headPositionPanBuffer.average(), currentHeadPan, 1e-3f)
           && Approx::isEqual(headPositionTiltBuffer.average(), currentHeadTilt, 1e-3f);
  };

  auto headStoppedMoving = [&]()
  {
    return std::abs(headPositionPanBuffer[0] - headPositionPanBuffer[1]) < 0.0001
           && std::abs(headPositionTiltBuffer[0] - headPositionTiltBuffer[1]) < 0.0001;
  };

  if(areBuffersFilled() && (headReachedPosition() || headStoppedMoving()))
  {
    headPositionPanBuffer.clear();
    headPositionTiltBuffer.clear();
    //uncomment this line if accumulation should not started automatically
    //state = WaitForAccumulate;//Accumulate; //WaitForAccumulate
    //uncomment this line if accumulation should started automatically
    //state = Accumulate;
    //The head has to wait a certain time to change the goalposition
    if(waitTill == 0)
      waitTill = SystemCall::getCurrentSystemTime() + headMotionWaitTime;

    if(SystemCall::getCurrentSystemTime() >= waitTill)
    {
      waitTill = 0;
      state = Accumulate;
    }
  }
  // TODO add timeout?
}

void AutomaticCameraCalibrator::accumulate()
{
  if(theCameraInfo.camera != currentCamera)
    return;

  auto isInsideImage = [&](const Vector2i & point)
  {
    return point.x() >= 0 && point.x() < theCameraInfo.width
           && point.y() >= 0 && point.y() < theCameraInfo.height;
  };

  std::vector<Sample> pointsOnALine;
  for(const LineSpots::Line& line : theLineSpots.lines)
  {
    for(const Vector2i& point : line.spotsInImg)
    {
      // store all necessary information in the sample
      Vector2f pointOnField;
      if(!isInsideImage(point))
        continue;
      else if(!Transformation::imageToRobot(point.x(), point.y(), theCameraMatrix, theCameraInfo, pointOnField))
      {
        OUTPUT_TEXT("MEEEK! Point not on field!" << (theCameraInfo.camera == CameraInfo::upper ? " Upper " : " Lower "));
        continue;
      }

      Sample sample;
      sample.pointInImage = point;
      sample.pointOnField = pointOnField;
      sample.torsoMatrix = theTorsoMatrix;
      sample.headYaw = theJointAngles.angles[Joints::headYaw];
      sample.headPitch = theJointAngles.angles[Joints::headPitch];
      sample.cameraInfo = theCameraInfo;

      bool sufficientDistanceToOtherSamples = true;
      for(Sample testSample : pointsOnALine)
      {
        Vector2f difference = sample.pointOnField - testSample.pointOnField;
        if(testSample.cameraInfo.camera != sample.cameraInfo.camera)
          continue;
        if(difference.norm() < minimumSampleDistance)
          sufficientDistanceToOtherSamples = false;
      }
      if(sufficientDistanceToOtherSamples)
      {
        samples.push_back(sample);
        pointsOnALine.push_back(sample);
      }
    }
  }

  accumulationTimestamp = SystemCall::getCurrentSystemTime();
  state = WaitForUser;
}

void AutomaticCameraCalibrator::waitForUser()
{
  if((SystemCall::getCurrentSystemTime() - accumulationTimestamp) > numOfSecondsToWaitForUser * 1000)
    state = MoveHead;
}

void AutomaticCameraCalibrator::optimize()
{
  if(!optimizer)
  {
    vector<float> initialParameters;
    initialParameters.resize(numOfParameterTranslations);
    // since the parameters for the robot pose are correction parameters,
    // an empty RobotPose is used instead of theRobotPose
    translateParameters(theCameraCalibration, RobotPose(), initialParameters);
    optimizer = new GaussNewtonOptimizer<Sample, AutomaticCameraCalibrator>(initialParameters,
        samples, *this, &AutomaticCameraCalibrator::computeErrorParameterVector);
    successiveConvergations = 0;
    framesToWait = 0;
  }
  else
  {
    // only do an iteration after some frames have passed
    if(framesToWait <= 0)
    {
      framesToWait = numOfFramesToWait;
      const float delta = optimizer->iterate();
      ASSERT(!std::isnan(delta));
      OUTPUT_TEXT("AutomaticCameraCalibrator: delta = " << delta);

      // the camera calibration is refreshed from the current optimizer state
      RobotPose robotPose;
      const vector<float> currentParameters = optimizer->getParameters();
      translateParameters(currentParameters, nextCameraCalibration, robotPose);

      if(abs(delta) < terminationCriterion)
        ++successiveConvergations;
      else
        successiveConvergations = 0;
      if(successiveConvergations >= minSuccessiveConvergations)
      {
        OUTPUT_TEXT("AutomaticCameraCalibrator: converged!");
        OUTPUT_TEXT("RobotPoseCorrection: " << currentParameters[robotPoseXCorrection] * 1000.0f
                    << " " << currentParameters[robotPoseYCorrection] * 1000.0f
                    << " " << toDegrees(currentParameters[robotPoseRotationCorrection]) << "deg");
        currentRobotPose.translation.x() += currentParameters[robotPoseXCorrection] * 1000.0f;
        currentRobotPose.translation.y() += currentParameters[robotPoseYCorrection] * 1000.0f;
        currentRobotPose.rotation = Angle::normalize(currentRobotPose.rotation + currentParameters[robotPoseRotationCorrection]);
        abort();

        if(setJointOffsets)
          invert(theCameraCalibration);
      }
    }
    --framesToWait;
  }
}

void AutomaticCameraCalibrator::invert(const CameraCalibration& cameraCalibration)
{
  Vector3a buffLowerCameraRotationCorrection = cameraCalibration.lowerCameraRotationCorrection;
  Vector3a buffUpperCameraRotationCorrection = cameraCalibration.upperCameraRotationCorrection;
  Angle buffBodyRotationCorrectionX = cameraCalibration.bodyRotationCorrection.x() * -1;
  Angle buffBodyRotationCorrectionY = cameraCalibration.bodyRotationCorrection.y() * -1;
  CameraCalibration nextCalibration;
  nextCalibration.lowerCameraRotationCorrection = buffLowerCameraRotationCorrection;
  nextCalibration.upperCameraRotationCorrection = buffUpperCameraRotationCorrection;
  nextCalibration.bodyRotationCorrection.x() = 0;
  nextCalibration.bodyRotationCorrection.y() = 0;
  nextCameraCalibration = nextCalibration;
  JointCalibrator::setOffsets(buffBodyRotationCorrectionX, buffBodyRotationCorrectionY);
}

void AutomaticCameraCalibrator::swapCameras(CameraInfo::Camera cameraToUse)
{
  if(SystemCall::getMode() == SystemCall::Mode::physicalRobot)
    nextResolution = cameraToUse == CameraInfo::upper ? CameraResolution::upper640 : CameraResolution::lower640;
  currentCamera = cameraToUse;
}

void AutomaticCameraCalibrator::abort()
{
  if(optimizer)
  {
    delete optimizer;
    optimizer = nullptr;
  }
  swapCameras(startCamera);
  state = Idle;
}

void AutomaticCameraCalibrator::update(CameraCalibrationNext& cameraCalibrationNext)
{
  const RobotCameraMatrix robotCameraMatrix(theRobotDimensions,
      theJointAngles.angles[Joints::headYaw],
      theJointAngles.angles[Joints::headPitch],
      theCameraCalibration,
      theCameraInfo.camera == CameraInfo::upper);
  theCameraMatrix.computeCameraMatrix(theTorsoMatrix, robotCameraMatrix, theCameraCalibration);

  nextCameraCalibration = theCameraCalibration;
  MODIFY_ONCE("module:AutomaticCameraCalibrator:robotPose", currentRobotPose);

  //Allow access to variables by other modules, required for the AutomaticCameraCalibratorHandler
  MODIFY("module:AutomaticCameraCalibrator:deletionPoint", unwantedPoint);
  MODIFY("module:AutomaticCameraCalibrator:deletionCurrentCamera", deletionOnCamera);
  MODIFY("module:AutomaticCameraCalibrator:insertionPoint", wantedPoint);
  MODIFY("module:AutomaticCameraCalibrator:insertionCurrentCamera", insertionOnCamera);
  MODIFY("module:AutomaticCameraCalibrator:setJointOffsets", setJointOffsets);

  processManualControls();
  states[state]();
  draw();

  cameraCalibrationNext.setNext(nextCameraCalibration);
}

void AutomaticCameraCalibrator::update(CameraResolutionRequest& cameraResolutionRequest)
{
  if(nextResolution != CameraResolution::noRequest &&
     theCameraResolution.resolution != nextResolution)
  {
    if(!cameraResolutionRequest.setRequest(nextResolution))
    {
      OUTPUT_TEXT("Changing the resolution is not permitted! Calibration aborted...");
      abort();
    }
  }
  nextResolution = CameraResolution::noRequest;
}

void AutomaticCameraCalibrator::update(HeadAngleRequest& headAngleRequest)
{
  if(state != Init && state != Idle && state != MoveHead)
    headAngleRequest = nextHeadAngleRequest;
}

void AutomaticCameraCalibrator::deleteSample(Vector2i point, CameraInfo::Camera camera)
{
  Vector2f pointOnField;//Needed for identification of the sample
  bool x = Transformation::imageToRobot(point.x(), point.y(), theCameraMatrix, theCameraInfo, pointOnField);
  if(!x)//Suppress warnings
    return;
  for(auto existingSample = samples.begin(); existingSample != samples.end();)
  {
    if(existingSample->cameraInfo.camera == camera)
    {
      Vector2f pointInImage;
      const RobotCameraMatrix robotCameraMatrix(
        theRobotDimensions, theJointAngles.angles[Joints::headYaw],
        theJointAngles.angles[Joints::headPitch],
        startingCameraCalibration, theCameraInfo.camera == CameraInfo::upper);
      const CameraMatrix cameraMatrix(theTorsoMatrix, robotCameraMatrix, startingCameraCalibration);
      if(Transformation::robotToImage(existingSample->pointOnField, cameraMatrix, theCameraInfo, pointInImage)
         && static_cast<int>(pointInImage.x() + 0.5) >= point.x() - deletionThreshold && static_cast<int>(pointInImage.y() + 0.5) >= point.y() - deletionThreshold
         && static_cast<int>(pointInImage.x() + 0.5) <= point.x() + deletionThreshold && static_cast<int>(pointInImage.y() + 0.5) <= point.y() + deletionThreshold)
      {
        lastDeletedSample = *existingSample;
        alreadyRevertedDeletion = false;
        lastActionWasInsertion = false;
        existingSample = samples.erase(existingSample);
        continue; // skip ++existingSample
      }
    }
    ++existingSample;
  }
}

void AutomaticCameraCalibrator::undo()
{
  if(!lastActionWasInsertion)
  {
    if(!alreadyRevertedDeletion) //Dont add a sample more than once
    {
      alreadyRevertedDeletion = true;
      samples.push_back(lastDeletedSample);
    }
  }
  else
  {
    if(!alreadyRevertedInsertion) //Dont delete a sample more than once
    {
      int counter = 0;
      bool check = false;
      for(Sample sample : samples)
      {
        if(sample.pointInImage.x() == lastInsertedSample.pointInImage.x() && sample.pointInImage.y() == lastInsertedSample.pointInImage.y())
        {
          check = true;
          break;
        }
        ++counter;
      }
      if(check)
      {
        alreadyRevertedInsertion = true;
        samples.erase(samples.begin() + counter);
      }
    }
  }
}

void AutomaticCameraCalibrator::insertSample(Vector2i point, CameraInfo::Camera camera)
{
  CameraInfo cameraToUse;
  if(camera == theCameraInfo.camera)
  {
    cameraToUse = theCameraInfo;
  }
  else
  {
    cameraToUse = CameraInfo(camera == CameraInfo::Camera::upper ? CameraInfo::Camera::upper : CameraInfo::Camera::lower);
  }
  Sample sample;
  sample.pointInImage = point;
  if(!Transformation::imageToRobot(point.x(), point.y(), theCameraMatrix, cameraToUse, sample.pointOnField))
    return;
  sample.torsoMatrix = theTorsoMatrix;
  sample.headYaw = theJointAngles.angles[Joints::headYaw];
  sample.headPitch = theJointAngles.angles[Joints::headPitch];
  sample.cameraInfo = cameraToUse;
  samples.push_back(sample);
  lastInsertedSample = sample;
  lastActionWasInsertion = true;
  alreadyRevertedInsertion = false;
}

void AutomaticCameraCalibrator::processManualControls()
{
  DEBUG_RESPONSE_ONCE("module:AutomaticCameraCalibrator:accumulate")
  {
    if(state == WaitForAccumulate)
      state = Accumulate;
  }
  DEBUG_RESPONSE_ONCE("module:AutomaticCameraCalibrator:start")
  {
    if(state == Idle)
      state = Init;
  }
  DEBUG_RESPONSE_ONCE("module:AutomaticCameraCalibrator:stop")
  {
    abort();
  }
  DEBUG_RESPONSE_ONCE("module:AutomaticCameraCalibrator:optimize")
  {
    if(state == WaitForOptimize || samples.size() > numOfParameterTranslations)
      state = Optimize;
  }
  DEBUG_RESPONSE_ONCE("module:AutomaticCameraCalibrator:deletePoint")
  {
    deleteSample(unwantedPoint, deletionOnCamera);
  }
  DEBUG_RESPONSE_ONCE("module:AutomaticCameraCalibrator:undo")
  {
    undo();
  }
  DEBUG_RESPONSE_ONCE("module:AutomaticCameraCalibrator:insertPoint")
  {
    insertSample(wantedPoint, insertionOnCamera);
  }
}

void AutomaticCameraCalibrator::draw() const
{
  DECLARE_DEBUG_DRAWING("module:AutomaticCameraCalibrator:drawFieldLines", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:AutomaticCameraCalibrator:drawSamples", "drawingOnImage");
  DEBUG_DRAWING("module:AutomaticCameraCalibrator:points", "drawingOnImage")
  {
    DRAWTEXT("module:AutomaticCameraCalibrator:points", 10, -10, 40,
             !(samples.size() > numOfParameterTranslations) ? ColorRGBA::red : ColorRGBA::green,
             "Points collected: " << static_cast<unsigned>(samples.size()));
  }

  COMPLEX_DRAWING("module:AutomaticCameraCalibrator:drawFieldLines") drawFieldLines();
  COMPLEX_DRAWING("module:AutomaticCameraCalibrator:drawSamples") drawSamples();
}

void AutomaticCameraCalibrator::drawFieldLines() const
{
  const Pose2f robotPoseInv = currentRobotPose.inverse();
  for(FieldDimensions::LinesTable::Line lineOnField : theFieldDimensions.fieldLines.lines)
  {
    lineOnField.from = robotPoseInv * lineOnField.from;
    lineOnField.to = robotPoseInv * lineOnField.to;
    Geometry::Line lineInImage;
    if(projectLineOnFieldIntoImage(Geometry::Line(lineOnField.from, lineOnField.to - lineOnField.from),
                                   theCameraMatrix, theCameraInfo, lineInImage))
    {
      LINE("module:AutomaticCameraCalibrator:drawFieldLines", lineInImage.base.x(), lineInImage.base.y(),
           (lineInImage.base + lineInImage.direction).x(), (lineInImage.base + lineInImage.direction).y(), 1,
           Drawings::solidPen, ColorRGBA::black);
    }
  }
}

void AutomaticCameraCalibrator::drawSamples() const
{
  const RobotCameraMatrix robotCameraMatrix(
    theRobotDimensions, theJointAngles.angles[Joints::headYaw],
    theJointAngles.angles[Joints::headPitch],
    startingCameraCalibration, theCameraInfo.camera == CameraInfo::upper);
  const CameraMatrix cameraMatrix(theTorsoMatrix, robotCameraMatrix, startingCameraCalibration);
  for(const Sample& sample : samples)
  {
    ColorRGBA color = sample.cameraInfo.camera == CameraInfo::upper ? ColorRGBA::orange : ColorRGBA::red;
    Vector2f pointInImage;
    if(Transformation::robotToImage(sample.pointOnField, cameraMatrix, theCameraInfo, pointInImage))
    {
      CROSS("module:AutomaticCameraCalibrator:drawSamples",
            static_cast<int>(pointInImage.x() + 0.5), static_cast<int>(pointInImage.y() + 0.5),
            5, 1, Drawings::solidPen, color);
    }
  }
}

float AutomaticCameraCalibrator::computeError(const Sample& sample, const CameraCalibration& cameraCalibration,
    const RobotPose& robotPose, bool inImage) const
{
  // build camera matrix from sample and camera calibration
  const RobotCameraMatrix robotCameraMatrix(theRobotDimensions, sample.headYaw, sample.headPitch,
      cameraCalibration, sample.cameraInfo.camera == CameraInfo::upper);
  const CameraMatrix cameraMatrix(sample.torsoMatrix, robotCameraMatrix, cameraCalibration);

  float minimum = numeric_limits<float>::max();
  if(inImage)
  {
    const Pose2f robotPoseInv = robotPose.inverse();
    for(FieldDimensions::LinesTable::Line lineOnField : theFieldDimensions.fieldLines.lines)
    {
      // transform the line in robot relative coordinates
      lineOnField.from = robotPoseInv * lineOnField.from;
      lineOnField.to = robotPoseInv * lineOnField.to;
      Geometry::Line lineInImage;
      float distance;
      if(!projectLineOnFieldIntoImage(Geometry::Line(lineOnField.from, lineOnField.to - lineOnField.from),
                                      cameraMatrix, sample.cameraInfo, lineInImage))
        distance = aboveHorizonError;
      else
        distance = Geometry::getDistanceToEdge(lineInImage, sample.pointInImage.cast<float>());

      if(distance < minimum)
        minimum = distance;
    }
  }
  else // on ground
  {
    // project point in image onto ground
    Vector3f cameraRay(sample.cameraInfo.focalLength, sample.cameraInfo.opticalCenter.x() - sample.pointInImage.x(), sample.cameraInfo.opticalCenter.y() - sample.pointInImage.y());
    cameraRay = cameraMatrix * cameraRay;
    if(cameraRay.z() >= 0)  // above horizon
      return aboveHorizonError;
    const float scale = cameraMatrix.translation.z() / -cameraRay.z();
    cameraRay *= scale;
    Vector2f pointOnGround(cameraRay.x(), cameraRay.y()); // point on ground relative to the robot
    pointOnGround = robotPose * pointOnGround; // point on ground in absolute coordinates

    for(FieldDimensions::LinesTable::Line lineOnField : theFieldDimensions.fieldLines.lines)
    {
      const Geometry::Line line(lineOnField.from, lineOnField.to - lineOnField.from);
      const float distance = Geometry::getDistanceToEdge(line, pointOnGround);
      if(distance < minimum)
        minimum = distance;
    }
  }
  return minimum;
}

float AutomaticCameraCalibrator::computeErrorParameterVector(const Sample& sample, const vector<float>& parameters) const
{
  CameraCalibration cameraCalibration = nextCameraCalibration;
  RobotPose robotPose;
  translateParameters(parameters, cameraCalibration, robotPose);

  // the correction parameters for the robot pose are added to theRobotPose
  // in the parameter space the robot pose translation unit is m to keep the order of
  // magnitude similar to the other parameters
  robotPose.translation *= 1000.0f;
  robotPose.translation += currentRobotPose.translation;
  robotPose.rotation += currentRobotPose.rotation;

  return computeError(sample, cameraCalibration, robotPose);
}

void AutomaticCameraCalibrator::translateParameters(const vector<float>& parameters,
    CameraCalibration& cameraCalibration, RobotPose& robotPose) const
{
  ASSERT(parameters.size() == numOfParameterTranslations);

  cameraCalibration.lowerCameraRotationCorrection.x() = parameters[lowerCameraRollCorrection];
  cameraCalibration.lowerCameraRotationCorrection.y() = parameters[lowerCameraTiltCorrection];
  //cameraCalibration.lowerCameraRotationCorrection.z = parameters[lowerCameraPanCorrection];

  cameraCalibration.upperCameraRotationCorrection.x() = parameters[upperCameraRollCorrection];
  cameraCalibration.upperCameraRotationCorrection.y() = parameters[upperCameraTiltCorrection];
  //cameraCalibration.upperCameraRotationCorrection.z = parameters[upperCameraPanCorrection];

  cameraCalibration.bodyRotationCorrection.x() = parameters[bodyRollCorrection];
  cameraCalibration.bodyRotationCorrection.y() = parameters[bodyTiltCorrection];

  robotPose.translation.x() = parameters[robotPoseXCorrection];
  robotPose.translation.y() = parameters[robotPoseYCorrection];
  robotPose.rotation = parameters[robotPoseRotationCorrection];
}

void AutomaticCameraCalibrator::translateParameters(const CameraCalibration& cameraCalibration,
    const RobotPose& robotPose, vector<float>& parameters) const
{
  ASSERT(parameters.size() == numOfParameterTranslations);

  parameters[lowerCameraRollCorrection] = cameraCalibration.lowerCameraRotationCorrection.x();
  parameters[lowerCameraTiltCorrection] = cameraCalibration.lowerCameraRotationCorrection.y();
  //parameters[lowerCameraPanCorrection] = cameraCalibration.lowerCameraRotationCorrection.z;

  parameters[upperCameraRollCorrection] = cameraCalibration.upperCameraRotationCorrection.x();
  parameters[upperCameraTiltCorrection] = cameraCalibration.upperCameraRotationCorrection.y();
  //parameters[upperCameraPanCorrection] = cameraCalibration.upperCameraRotationCorrection.z;

  parameters[bodyRollCorrection] = cameraCalibration.bodyRotationCorrection.x();
  parameters[bodyTiltCorrection] = cameraCalibration.bodyRotationCorrection.y();

  parameters[robotPoseXCorrection] = robotPose.translation.x();
  parameters[robotPoseYCorrection] = robotPose.translation.y();
  parameters[robotPoseRotationCorrection] = robotPose.rotation;
}

bool AutomaticCameraCalibrator::projectLineOnFieldIntoImage(const Geometry::Line& lineOnField,
    const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, Geometry::Line& lineInImage) const
{
  const float& f = cameraInfo.focalLength;
  const Pose3f cameraMatrixInv = cameraMatrix.inverse();

  // TODO more elegant solution directly using the direction of the line?

  // start and end point of the line
  Vector2f p1 = lineOnField.base;
  Vector2f p2 = p1 + lineOnField.direction;
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
