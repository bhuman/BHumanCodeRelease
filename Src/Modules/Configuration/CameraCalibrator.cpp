/**
* @file CameraCalibrator.h
*
* This file implements a module that can provide a semiautomatic camera calibration.
*
* @author Alexander Härtl
*/

#include "CameraCalibrator.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Settings.h"
#include "Representations/Perception/CameraMatrix.h"
#include <limits>

using namespace std;

CameraCalibrator::CameraCalibrator() : calibrationState(Idle), lastFetchedPoint(-1, -1), optimizer(NULL)
{
}

CameraCalibrator::~CameraCalibrator()
{
  if(optimizer)
  {
    delete optimizer;
  }
}

void CameraCalibrator::update(CameraCalibration& cameraCalibration)
{
  currentCameraCalibration = &cameraCalibration;

  // this is the interface to the ImageView of the simulator
  Vector2<int> point(-1, -1);
  MODIFY("module:CameraCalibrator:point", point);

  // "declare" debug responses. This is necessary to be able to send out
  // the debug requests without the necessity to poll after every state change
  if(calibrationState != Idle)
  {
    DEBUG_RESPONSE_ONCE("module:CameraCalibrator:collectPoints",);
  }
  if(calibrationState != Accumulate)
  {
    DEBUG_RESPONSE_ONCE("module:CameraCalibrator:optimize",);
    DEBUG_RESPONSE_ONCE("module:CameraCalibrator:undo",);
    DEBUG_RESPONSE_ONCE("module:CameraCalibrator:clear",);
  }
  if(calibrationState != Optimize)
  {
    DEBUG_RESPONSE_ONCE("module:CameraCalibrator:stop",);
  }

  // the optimizer is only needed during the state Optimize
  if(calibrationState != Optimize && optimizer)
  {
    delete optimizer;
    optimizer = NULL;
  }

  switch(calibrationState)
  {
  case(Idle):
    DEBUG_RESPONSE_ONCE("module:CameraCalibrator:collectPoints",
    {
      calibrationState = Accumulate;
      MODIFY("module:CameraCalibrator:point", lastFetchedPoint);
    });
    break;
  case(Accumulate): // collect points used for the optimization
    fetchPoint();
    DEBUG_RESPONSE_ONCE("module:CameraCalibrator:undo",
    {
      samples.pop_back();
    });
    DEBUG_RESPONSE_ONCE("module:CameraCalibrator:clear",
    {
      MODIFY("module:CameraCalibrator:point", lastFetchedPoint);
      samples.clear();
    });
    DEBUG_RESPONSE_ONCE("module:CameraCalibrator:optimize",
    {
      if(!(samples.size() > numOfParameterTranslations))
        OUTPUT(idText, text, "CameraCalibrator: Error! too few samples!");
      else
        calibrationState = Optimize;
    });
    break;
  case(Optimize): // optimize the parameters of the cameraCalibration
    if(!optimizer)
    {
      vector<float> initialParameters;
      initialParameters.resize(calibrateBothCameras ? numOfParameterTranslations : numOfParametersLowerCamera);
      // since the parameters for the robot pose are correction parameters, an empty RobotPose is used instead of theRobotPose
      translateParameters(cameraCalibration, RobotPose(), initialParameters);
      optimizer = new GaussNewtonOptimizer<Sample, CameraCalibrator>(initialParameters, samples, *this, &CameraCalibrator::computeErrorParameterVector);
      successiveConvergations = 0;
      framesToWait = 0;
    }
    else
    {
      DEBUG_RESPONSE_ONCE("module:CameraCalibrator:stop", calibrationState = Idle;);
      // only do an iteration after some frames have passed
      if(framesToWait <= 0)
      {
        framesToWait = numOfFramesToWait;
        const float delta = optimizer->iterate();
        OUTPUT(idText, text, "CameraCalibrator: delta = " << delta);

        // the camera calibration is refreshed from the current optimizer state
        RobotPose robotPose;
        const vector<float> currentParameters = optimizer->getParameters();
        translateParameters(currentParameters, cameraCalibration, robotPose);

        if(abs(delta) < terminationCriterion)
        {
          ++successiveConvergations;
        }
        if(successiveConvergations >= minSuccessiveConvergations)
        {
          calibrationState = Idle;
          OUTPUT(idText, text, "CameraCalibrator: converged!");
          OUTPUT(idText, text, "RobotPoseCorrection: " << currentParameters[robotPoseCorrectionX] * 1000.0f << " " << currentParameters[robotPoseCorrectionY] * 1000.0f << " " << currentParameters[robotPoseCorrectionRot]);
        }
      }
      --framesToWait;
    }
    break;
  default:
    ASSERT(false);
  }

  DECLARE_DEBUG_DRAWING("module:CameraCalibrator:drawFieldLines", "drawingOnImage");
  COMPLEX_DRAWING("module:CameraCalibrator:drawFieldLines", drawFieldLines(););

  DECLARE_DEBUG_DRAWING("module:CameraCalibrator:points", "drawingOnImage");
  COMPLEX_DRAWING("module:CameraCalibrator:points",
  {
    DRAWTEXT("module:CameraCalibrator:points", 10, 10, 40, !(samples.size() > numOfParameterTranslations) ? ColorClasses::red : ColorClasses::green, "Points collected: " << (unsigned) samples.size());
  });
}

float CameraCalibrator::computeError(const Sample& sample, const CameraCalibration& cameraCalibration, const RobotPose& robotPose, bool inImage) const
{
  // a sample of the upper camera is ignored if only the lower camera is calibrated
  if(!calibrateBothCameras && sample.upperCamera)
  {
    return 0.0f;
  }

  // build camera matrix from sample and camera calibration
  const RobotCameraMatrix robotCameraMatrix(theRobotDimensions, sample.headYaw, sample.headPitch, cameraCalibration, sample.upperCamera);
  const CameraMatrix cameraMatrix(sample.torsoMatrix, robotCameraMatrix, cameraCalibration);

  if(inImage)
  {
    const Pose2D robotPoseInv = robotPose.invert();
    float minimum = numeric_limits<float>::max();
    for(vector<FieldDimensions::LinesTable::Line>::const_iterator i = theFieldDimensions.fieldLines.lines.begin(); i != theFieldDimensions.fieldLines.lines.end(); ++i)
    {
      FieldDimensions::LinesTable::Line lineOnField(*i);
      // transform the line in robot relative coordinates
      lineOnField.corner = robotPoseInv + lineOnField.corner;
      Geometry::Line lineInImage;
      float distance;
      if(!projectLineOnFieldIntoImage(Geometry::Line(lineOnField.corner, lineOnField.length), cameraMatrix, lineInImage))
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
    return minimum;
  }
  else // on ground
  {
    // project point in image onto ground
    Vector3<> cameraRay(theCameraInfo.focalLength, theCameraInfo.opticalCenter.x - sample.pointInImage.x, theCameraInfo.opticalCenter.y - sample.pointInImage.y);
    cameraRay = cameraMatrix * cameraRay;
    if(cameraRay.z >= 0) // above horizon
    {
      return aboveHorizonError;
    }
    const float scale = cameraMatrix.translation.z / -cameraRay.z;
    cameraRay *= scale;
    Vector2<> pointOnGround(cameraRay.x, cameraRay.y); // point on ground relative to the robot
    pointOnGround = robotPose * pointOnGround; // point on ground in absolute coordinates

    float minimum = numeric_limits<float>::max();
    for(vector<FieldDimensions::LinesTable::Line>::const_iterator i = theFieldDimensions.fieldLines.lines.begin(); i != theFieldDimensions.fieldLines.lines.end(); ++i)
    {
      const Geometry::Line line(i->corner, i->length);
      const float distance = Geometry::getDistanceToEdge(line, pointOnGround);
      if(distance < minimum)
      {
        minimum = distance;
      }
    }
    return minimum;
  }
}

float CameraCalibrator::computeErrorParameterVector(const Sample& sample, const vector<float>& parameters) const
{
  CameraCalibration cameraCalibration = *currentCameraCalibration;
  RobotPose robotPose;
  translateParameters(parameters, cameraCalibration, robotPose);

  // the correction parameters for the robot pose are added to theRobotPose
  // in the parameter space the robot pose translation unit is m to keep the order of magnitude similar to the other parameters
  robotPose.translation *= 1000.0f;
  robotPose.translation += theRobotPose.translation;
  robotPose.rotation += theRobotPose.rotation;

  return computeError(sample, cameraCalibration, robotPose);
}

void CameraCalibrator::translateParameters(const vector<float>& parameters, CameraCalibration& cameraCalibration, RobotPose& robotPose) const
{
  ASSERT(parameters.size() == numOfParameterTranslations || parameters.size() == numOfParametersLowerCamera);

  cameraCalibration.cameraTiltCorrection = parameters[cameraTiltCorrection];
  cameraCalibration.cameraRollCorrection = parameters[cameraRollCorrection];
  cameraCalibration.bodyTiltCorrection = parameters[bodyTiltCorrection];
  cameraCalibration.bodyRollCorrection = parameters[bodyRollCorrection];

  robotPose.translation.x = parameters[robotPoseCorrectionX];
  robotPose.translation.y = parameters[robotPoseCorrectionY];
  robotPose.rotation = parameters[robotPoseCorrectionRot];

  if(parameters.size() == numOfParameterTranslations)
  {
    cameraCalibration.upperCameraRollCorrection = parameters[upperCameraX];
    cameraCalibration.upperCameraTiltCorrection = parameters[upperCameraY];
    cameraCalibration.upperCameraPanCorrection = parameters[upperCameraZ];
  }
}

void CameraCalibrator::translateParameters(const CameraCalibration& cameraCalibration, const RobotPose& robotPose, vector<float>& parameters) const
{
  ASSERT(parameters.size() == numOfParameterTranslations || parameters.size() == numOfParametersLowerCamera);

  parameters[cameraTiltCorrection] = cameraCalibration.cameraTiltCorrection;
  parameters[cameraRollCorrection] = cameraCalibration.cameraRollCorrection;
  parameters[bodyTiltCorrection] = cameraCalibration.bodyTiltCorrection;
  parameters[bodyRollCorrection] = cameraCalibration.bodyRollCorrection;

  parameters[robotPoseCorrectionX] = robotPose.translation.x;
  parameters[robotPoseCorrectionY] = robotPose.translation.y;
  parameters[robotPoseCorrectionRot] = robotPose.rotation;

  if(parameters.size() == numOfParameterTranslations)
  {
    parameters[upperCameraX] = cameraCalibration.upperCameraRollCorrection;
    parameters[upperCameraY] = cameraCalibration.upperCameraTiltCorrection;
    parameters[upperCameraZ] = cameraCalibration.upperCameraPanCorrection;
  }
}

void CameraCalibrator::fetchPoint()
{
  Vector2<int> point(-1, -1);
  MODIFY("module:CameraCalibrator:point", point);

  if(point == lastFetchedPoint)
    return;

  lastFetchedPoint = point;

  // store all necessary information in the sample
  Sample sample;
  sample.pointInImage = point;
  sample.torsoMatrix = theTorsoMatrix;
  sample.headYaw = theFilteredJointData.angles[JointData::HeadYaw];
  sample.headPitch = theFilteredJointData.angles[JointData::HeadPitch];
  sample.upperCamera = theCameraInfo.camera == CameraInfo::upper;

  samples.push_back(sample);
}

bool CameraCalibrator::projectLineOnFieldIntoImage(const Geometry::Line& lineOnField, const CameraMatrix& cameraMatrix, Geometry::Line& lineInImage) const
{
  const float& f = theCameraInfo.focalLength;
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
  const bool p1Behind = p1Camera.x < theCameraInfo.focalLength;
  const bool p2Behind = p2Camera.x < theCameraInfo.focalLength;
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
    // if one point lies behind the camera and the other in front, there must be an intersection of the connective line with the image plane
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
  const Vector2<> p1Result(theCameraInfo.opticalCenter.x - p1Camera.y, theCameraInfo.opticalCenter.y - p1Camera.z);
  const Vector2<> p2Result(theCameraInfo.opticalCenter.x - p2Camera.y, theCameraInfo.opticalCenter.y - p2Camera.z);
  lineInImage.base = p1Result;
  lineInImage.direction = p2Result - p1Result;
  return true;
}

void CameraCalibrator::drawFieldLines()
{
  const Pose2D robotPoseInv = theRobotPose.invert();
  for(vector<FieldDimensions::LinesTable::Line>::const_iterator i = theFieldDimensions.fieldLines.lines.begin(); i != theFieldDimensions.fieldLines.lines.end(); ++i)
  {
    FieldDimensions::LinesTable::Line lineOnField(*i);
    lineOnField.corner = robotPoseInv + lineOnField.corner;
    Geometry::Line lineInImage;
    if(projectLineOnFieldIntoImage(Geometry::Line(lineOnField.corner, lineOnField.length), theCameraMatrix, lineInImage))
    {
      LINE("module:CameraCalibrator:drawFieldLines", lineInImage.base.x, lineInImage.base.y, (lineInImage.base + lineInImage.direction).x, (lineInImage.base + lineInImage.direction).y, 1, Drawings::ps_solid, ColorClasses::black);
    }
  }
}

MAKE_MODULE(CameraCalibrator, Cognition Infrastructure)
