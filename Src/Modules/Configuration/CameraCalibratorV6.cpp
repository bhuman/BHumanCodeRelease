/**
* @file CameraCalibratorV6.h
*
* This file implements a module that can provide a automatic camera calibration.
*
* @author Dana Jenett, Alexis Tsogias
*/

#include "CameraCalibratorV6.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Settings.h"
#include <limits>

using namespace std;

MAKE_MODULE(CameraCalibratorV6, Cognition Infrastructure)

CameraCalibratorV6::CameraCalibratorV6() : state(Idle)
{
  states[Idle] = std::bind(&CameraCalibratorV6::idle, this);
  states[Init] = std::bind(&CameraCalibratorV6::init, this);
  states[MoveHead] = std::bind(&CameraCalibratorV6::moveHead, this);
  states[WaitForHeadToReachPosition] = std::bind(&CameraCalibratorV6::waitForHeadToReachPosition, this);
  states[WaitForAccumulate] = std::bind(&CameraCalibratorV6::idle, this);
  states[Accumulate] = std::bind(&CameraCalibratorV6::accumulate, this);
  states[WaitForUser] = std::bind(&CameraCalibratorV6::waitForUser, this);
  states[WaitForOptimize] = std::bind(&CameraCalibratorV6::idle, this);
  states[Optimize] = std::bind(&CameraCalibratorV6::optimize, this);
}

CameraCalibratorV6::~CameraCalibratorV6()
{
  if(optimizer)
  {
    delete optimizer;
  }
}

void CameraCalibratorV6::idle()
{
  // Do nothing
}

void CameraCalibratorV6::init()
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
  numOfHeadPans = static_cast<unsigned>(headPans.size());
  headPositionPanBuffer.init();
  headPositionTiltBuffer.init();

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

void CameraCalibratorV6::moveHead()
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
  else if(firstHeadPans.empty() && secondHeadPans.size() == numOfHeadPans)
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

void CameraCalibratorV6::waitForHeadToReachPosition()
{
  headPositionPanBuffer.add(theFilteredJointData.angles[JointData::HeadYaw]);
  headPositionTiltBuffer.add(theFilteredJointData.angles[JointData::HeadPitch]);


  auto areBuffersFulled = [&]()
  {
    return headPositionPanBuffer.isFilled() && headPositionTiltBuffer.isFilled();
  };

  auto headReachedPosition = [&]()
  {
    return std::abs(headPositionPanBuffer.getAverage() - currentHeadPan) < 0.001
           && std::abs(headPositionTiltBuffer.getAverage() - currentHeadTilt) < 0.001;
  };

  auto headStopedMoving = [&]()
  {
    return std::abs(headPositionPanBuffer.getEntry(0) - headPositionPanBuffer.getEntry(1)) < 0.0001
           && std::abs(headPositionTiltBuffer.getEntry(0) - headPositionTiltBuffer.getEntry(1)) < 0.0001;
  };

  if(areBuffersFulled() && (headReachedPosition() || headStopedMoving()))
  {
    headPositionPanBuffer.init();
    headPositionTiltBuffer.init();
    //uncomment this line if accumulation should not started automatically
    //state = WaitForAccumulate;//Accumulate; //WaitForAccumulate
    //uncomment this line if accumulation should started automatically
    state = Accumulate;
  }
  // TODO add timeout?
}

void CameraCalibratorV6::accumulate()
{
  if(theCameraInfo.camera != currentCamera)
    return;

  auto isInsideImage = [&](const Vector2<int>& point)
  {
    return point.x >= 0 && point.x < theCameraInfo.width
           && point.y >= 0 && point.y < theCameraInfo.height;
  };

  for(const LineSpots::Line& line : theLineSpots.lines)
  {
    for(const Vector2<int>& point : line.spotsInImg)
    {
      // store all necessary information in the sample
      Vector2<> pointOnField;
      if(!isInsideImage(point))
      {
        continue;
      }
      else if(!Transformation::imageToRobot(point.x, point.y, theCameraMatrix, theCameraInfo, pointOnField))
      {
        OUTPUT_TEXT("MEEEK! Point not on field!" << (theCameraInfo.camera == CameraInfo::upper ? " Upper " : " Lower "));
        continue;
      }

      Sample sample;
      sample.pointInImage = point;
      sample.pointOnField = pointOnField;
      sample.torsoMatrix = theTorsoMatrix;
      sample.headYaw = theFilteredJointData.angles[JointData::HeadYaw];
      sample.headPitch = theFilteredJointData.angles[JointData::HeadPitch];
      sample.cameraInfo = theCameraInfo;

      samples.push_back(sample);
    }
  }

  accumulationTimestamp = SystemCall::getCurrentSystemTime();
  state = WaitForUser;
}

void CameraCalibratorV6::waitForUser()
{
  if((SystemCall::getCurrentSystemTime() - accumulationTimestamp) > numOfSecondsToWaitForUser * 1000)
    state = MoveHead;
}

void CameraCalibratorV6::optimize()
{
  if(!optimizer)
  {
    vector<float> initialParameters;
    initialParameters.resize(numOfParameterTranslations);
    // since the parameters for the robot pose are correction parameters,
    // an empty RobotPose is used instead of theRobotPose
    translateParameters(theCameraCalibration, RobotPose(), initialParameters);
    optimizer = new GaussNewtonOptimizer<Sample, CameraCalibratorV6>(initialParameters,
        samples, *this, &CameraCalibratorV6::computeErrorParameterVector);
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
      OUTPUT_TEXT("CameraCalibratorV6: delta = " << delta);

      // the camera calibration is refreshed from the current optimizer state
      RobotPose robotPose;
      const vector<float> currentParameters = optimizer->getParameters();
      translateParameters(currentParameters, nextCameraCalibration, robotPose);

      if(abs(delta) < terminationCriterion)
      {
        ++successiveConvergations;
      }
      if(successiveConvergations >= minSuccessiveConvergations)
      {
        OUTPUT_TEXT("CameraCalibratorV6: converged!");
        OUTPUT_TEXT("RobotPoseCorrection: " << currentParameters[robotPoseXCorrection] * 1000.0f
                    << " " << currentParameters[robotPoseYCorrection] * 1000.0f
                    << " " << currentParameters[robotPoseRotationCorrection]);
        currentRobotPose.translation.x += currentParameters[robotPoseXCorrection] * 1000.0f;
        currentRobotPose.translation.y += currentParameters[robotPoseYCorrection] * 1000.0f;
        currentRobotPose.rotation = normalize(currentRobotPose.rotation + currentParameters[robotPoseRotationCorrection]);
        abort();
      }
    }
    --framesToWait;
  }
}

void CameraCalibratorV6::swapCameras(CameraInfo::Camera cameraToUse)
{
  if(SystemCall::getMode() == SystemCall::Mode::physicalRobot)
    nextResolution = cameraToUse == CameraInfo::upper ? CameraResolution::upper640 : CameraResolution::lower640;
  currentCamera = cameraToUse;
}

void CameraCalibratorV6::abort()
{
  if(optimizer)
  {
    delete optimizer;
    optimizer = nullptr;
  }
  swapCameras(startCamera);
  state = Idle;
}

void CameraCalibratorV6::update(CameraCalibrationNext& cameraCalibrationNext)
{
  const RobotCameraMatrix robotCameraMatrix(theRobotDimensions,
      theFilteredJointData.angles[JointData::HeadYaw],
      theFilteredJointData.angles[JointData::HeadPitch],
      theCameraCalibration,
      theCameraInfo.camera == CameraInfo::upper);
  theCameraMatrix.computeCameraMatrix(theTorsoMatrix, robotCameraMatrix, theCameraCalibration);

  nextCameraCalibration = theCameraCalibration;
  MODIFY_ONCE("module:CameraCalibratorV6:robotPose", currentRobotPose);

  processManualControls();
  states[state]();
  draw();

  cameraCalibrationNext.setNext(nextCameraCalibration);
}

void CameraCalibratorV6::update(CameraResolutionRequest& cameraResolutionRequest)
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

void CameraCalibratorV6::update(HeadAngleRequest& headAngleRequest)
{
  if(state != Init && state != Idle && state != MoveHead)
  {
    headAngleRequest = nextHeadAngleRequest;
  }
}

void CameraCalibratorV6::processManualControls()
{
  DEBUG_RESPONSE_ONCE("module:CameraCalibratorV6:accumulate",
  {
    if(state == WaitForAccumulate)
      state = Accumulate;
  });
  DEBUG_RESPONSE_ONCE("module:CameraCalibratorV6:start",
  {
    if(state == Idle)
      state = Init;
  });
  DEBUG_RESPONSE_ONCE("module:CameraCalibratorV6:stop",
  {
    abort();
  });
  DEBUG_RESPONSE_ONCE("module:CameraCalibratorV6:optimize",
  {
    if(state == WaitForOptimize || samples.size() > numOfParameterTranslations)
      state = Optimize;
  });
}

void CameraCalibratorV6::draw()
{
  DECLARE_DEBUG_DRAWING("module:CameraCalibratorV6:drawFieldLines", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CameraCalibratorV6:drawSamples", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CameraCalibratorV6:points", "drawingOnImage",
  {
    DRAWTEXT("module:CameraCalibratorV6:points", 10, -10, 40,
    !(samples.size() > numOfParameterTranslations) ? ColorRGBA::red : ColorRGBA::green,
    "Points collected: " << static_cast<unsigned>(samples.size()));
  });

  COMPLEX_DRAWING("module:CameraCalibratorV6:drawFieldLines", drawFieldLines(););
  COMPLEX_DRAWING("module:CameraCalibratorV6:drawSamples", drawSamples(););
}

void CameraCalibratorV6::drawFieldLines()
{
  const Pose2D robotPoseInv = currentRobotPose.invert();
  for(FieldDimensions::LinesTable::Line lineOnField : theFieldDimensions.fieldLines.lines)
  {
    lineOnField.corner = robotPoseInv + lineOnField.corner;
    Geometry::Line lineInImage;
    if(projectLineOnFieldIntoImage(Geometry::Line(lineOnField.corner, lineOnField.length),
                                   theCameraMatrix, theCameraInfo, lineInImage))
    {
      LINE("module:CameraCalibratorV6:drawFieldLines", lineInImage.base.x, lineInImage.base.y,
           (lineInImage.base + lineInImage.direction).x, (lineInImage.base + lineInImage.direction).y, 1,
           Drawings::ps_solid, ColorRGBA::black);
    }
  }
}

void CameraCalibratorV6::drawSamples()
{
  const RobotCameraMatrix robotCameraMatrix(
    theRobotDimensions, theFilteredJointData.angles[JointData::HeadYaw],
    theFilteredJointData.angles[JointData::HeadPitch],
    startingCameraCalibration, theCameraInfo.camera == CameraInfo::upper);
  const CameraMatrix cameraMatrix(theTorsoMatrix, robotCameraMatrix, startingCameraCalibration);
  for(Sample& sample : samples)
  {
    ColorRGBA color = sample.cameraInfo.camera == CameraInfo::upper ? ColorRGBA::orange : ColorRGBA::red;
    Vector2<float> pointInImage;
    if(Transformation::robotToImage(sample.pointOnField, cameraMatrix, theCameraInfo, pointInImage))
    {
      CROSS("module:CameraCalibratorV6:drawSamples",
            static_cast<int>(pointInImage.x + 0.5), static_cast<int>(pointInImage.y + 0.5),
            5, 1, Drawings::ps_solid, color);
    }
  }
}

float CameraCalibratorV6::computeError(const Sample& sample, const CameraCalibration& cameraCalibration,
                                       const RobotPose& robotPose, bool inImage) const
{
  // build camera matrix from sample and camera calibration
  const RobotCameraMatrix robotCameraMatrix(theRobotDimensions, sample.headYaw, sample.headPitch,
      cameraCalibration, sample.cameraInfo.camera == CameraInfo::upper);
  const CameraMatrix cameraMatrix(sample.torsoMatrix, robotCameraMatrix, cameraCalibration);

  float minimum = numeric_limits<float>::max();
  if(inImage)
  {
    const Pose2D robotPoseInv = robotPose.invert();
    for(FieldDimensions::LinesTable::Line lineOnField : theFieldDimensions.fieldLines.lines)
    {
      // transform the line in robot relative coordinates
      lineOnField.corner = robotPoseInv + lineOnField.corner;
      Geometry::Line lineInImage;
      float distance;
      if(!projectLineOnFieldIntoImage(Geometry::Line(lineOnField.corner, lineOnField.length),
                                      cameraMatrix, sample.cameraInfo, lineInImage))
      {
        distance = aboveHorizonError;
      }
      else
      {
        distance = Geometry::getDistanceToEdge(lineInImage, Vector2<>(sample.pointInImage));
      }
      if(distance < minimum)
      {
        minimum = distance;
      }
    }
  }
  else // on ground
  {
    // project point in image onto ground
    Vector3<> cameraRay(sample.cameraInfo.focalLength, sample.cameraInfo.opticalCenter.x - sample.pointInImage.x, sample.cameraInfo.opticalCenter.y - sample.pointInImage.y);
    cameraRay = cameraMatrix * cameraRay;
    if(cameraRay.z >= 0)  // above horizon
    {
      return aboveHorizonError;
    }
    const float scale = cameraMatrix.translation.z / -cameraRay.z;
    cameraRay *= scale;
    Vector2<> pointOnGround(cameraRay.x, cameraRay.y); // point on ground relative to the robot
    pointOnGround = robotPose * pointOnGround; // point on ground in absolute coordinates

    for(FieldDimensions::LinesTable::Line lineOnField : theFieldDimensions.fieldLines.lines)
    {
      const Geometry::Line line(lineOnField.corner, lineOnField.length);
      const float distance = Geometry::getDistanceToEdge(line, pointOnGround);
      if(distance < minimum)
      {
        minimum = distance;
      }
    }
  }
  return minimum;
}

float CameraCalibratorV6::computeErrorParameterVector(const Sample& sample, const vector<float>& parameters) const
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

void CameraCalibratorV6::translateParameters(const vector<float>& parameters,
    CameraCalibration& cameraCalibration, RobotPose& robotPose) const
{
  ASSERT(parameters.size() == numOfParameterTranslations);


  cameraCalibration.upperCameraRollCorrection = parameters[upperCameraRollCorrection];
  cameraCalibration.upperCameraTiltCorrection = parameters[upperCameraTiltCorrection];
  //cameraCalibration.upperCameraPanCorrection = parameters[upperCameraPanCorrection];

  cameraCalibration.lowerCameraRollCorrection = parameters[lowerCameraRollCorrection];
  cameraCalibration.lowerCameraTiltCorrection = parameters[lowerCameraTiltCorrection];
  //cameraCalibration.lowerCameraPanCorrection = parameters[lowerCameraPanCorrection];

  cameraCalibration.bodyTiltCorrection = parameters[bodyTiltCorrection];
  cameraCalibration.bodyRollCorrection = parameters[bodyRollCorrection];

  robotPose.translation.x = parameters[robotPoseXCorrection];
  robotPose.translation.y = parameters[robotPoseYCorrection];
  robotPose.rotation = parameters[robotPoseRotationCorrection];
}

void CameraCalibratorV6::translateParameters(const CameraCalibration& cameraCalibration,
    const RobotPose& robotPose, vector<float>& parameters) const
{
  ASSERT(parameters.size() == numOfParameterTranslations);

  parameters[upperCameraRollCorrection] = cameraCalibration.upperCameraRollCorrection;
  parameters[upperCameraTiltCorrection] = cameraCalibration.upperCameraTiltCorrection;
  //parameters[upperCameraPanCorrection] = cameraCalibration.upperCameraPanCorrection;

  parameters[lowerCameraRollCorrection] = cameraCalibration.lowerCameraRollCorrection;
  parameters[lowerCameraTiltCorrection] = cameraCalibration.lowerCameraTiltCorrection;
  //parameters[lowerCameraPanCorrection] = cameraCalibration.lowerCameraPanCorrection;

  parameters[bodyTiltCorrection] = cameraCalibration.bodyTiltCorrection;
  parameters[bodyRollCorrection] = cameraCalibration.bodyRollCorrection;

  parameters[robotPoseXCorrection] = robotPose.translation.x;
  parameters[robotPoseYCorrection] = robotPose.translation.y;
  parameters[robotPoseRotationCorrection] = robotPose.rotation;
}

bool CameraCalibratorV6::projectLineOnFieldIntoImage(const Geometry::Line& lineOnField,
    const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, Geometry::Line& lineInImage) const
{
  const float& f = cameraInfo.focalLength;
  const Pose3D cameraMatrixInv = cameraMatrix.invert();

  // TODO more elegant solution directly using the direction of the line?

  // start and end point of the line
  Vector2<> p1 = lineOnField.base;
  Vector2<> p2 = p1 + lineOnField.direction;
  Vector3<> p1Camera(p1.x, p1.y, 0);
  Vector3<> p2Camera(p2.x, p2.y, 0);

  // points are transformed into camera coordinates
  p1Camera = cameraMatrixInv * p1Camera;
  p2Camera = cameraMatrixInv * p2Camera;

  // handle the case that points can lie behind the camera plane
  const bool p1Behind = p1Camera.x < cameraInfo.focalLength;
  const bool p2Behind = p2Camera.x < cameraInfo.focalLength;
  if(p1Behind && p2Behind)
  {
    return false;
  }
  else if(!p1Behind && !p2Behind)
  {
    // both rays can be simply intersected with the image plane
    p1Camera /= (p1Camera.x / f);
    p2Camera /= (p2Camera.x / f);
  }
  else
  {
    // if one point lies behind the camera and the other in front,
    // there must be an intersection of the connective line with the image plane
    const Vector3<> direction = p1Camera - p2Camera;
    const float scale = (f - p1Camera.x) / direction.x;
    const Vector3<> intersection = p1Camera + direction * scale;
    if(p1Behind)
    {
      p1Camera = intersection;
      p2Camera /= (p2Camera.x / f);
    }
    else
    {
      p2Camera = intersection;
      p1Camera /= (p1Camera.x / f);
    }
  }
  const Vector2<> p1Result(cameraInfo.opticalCenter.x - p1Camera.y, cameraInfo.opticalCenter.y - p1Camera.z);
  const Vector2<> p2Result(cameraInfo.opticalCenter.x - p2Camera.y, cameraInfo.opticalCenter.y - p2Camera.z);
  lineInImage.base = p1Result;
  lineInImage.direction = p2Result - p1Result;
  return true;
}