/**
 * @file CameraMatrixProvider.cpp
 * This file implements a class to calculate the position of the camera.
 * @author Alexander Härtl
 * @author Colin Graf
 */

#include "CameraMatrixProvider.h"
#include "Debugging/DebugDrawings.h"
#include "Debugging/DebugDrawings3D.h"
#include "Math/RotationMatrix.h"

MAKE_MODULE(CameraMatrixProvider);

void CameraMatrixProvider::update(CameraMatrix& cameraMatrix)
{
  cameraMatrix.computeCameraMatrix(theTorsoMatrix, theRobotCameraMatrix, theCameraCalibration);
  cameraMatrix.isValid = theTorsoMatrix.isValid && theMotionInfo.isMotionStable &&
                         (theFallDownState.state == FallDownState::upright ||
                          theFallDownState.state == FallDownState::staggering ||
                          theFallDownState.state == FallDownState::squatting) &&
                         theFrameInfo.getTimeSince(theJointAngles.timestamp) < 500 &&
                         (!theGameState.isPenalized() ||
                          theGameState.playerState == GameState::penalizedMotionInSet);

  DEBUG_DRAWING("module:CameraMatrixProvider:calibrationHelper", "drawingOnImage") drawFieldLines(cameraMatrix);
  DEBUG_DRAWING3D("module:CameraMatrixProvider:cameraMatrix", "field")
  {
    const float len = 200.f;
    Vector3f ml = Vector3f(1.f, -std::tan(theCameraInfo.openingAngleWidth * 0.5f), 0) * len;
    Vector3f mt = Vector3f(1.f, 0, std::tan(theCameraInfo.openingAngleHeight * 0.5f)) * len;
    Vector3f tl = cameraMatrix * Vector3f(len, ml.y(), mt.z());
    Vector3f tr = cameraMatrix * Vector3f(len, -ml.y(), mt.z());
    Vector3f bl = cameraMatrix * Vector3f(len, ml.y(), -mt.z());
    Vector3f br = cameraMatrix * Vector3f(len, -ml.y(), -mt.z());
    const Vector3f& ce = cameraMatrix.translation;

    LINE3D("module:CameraMatrixProvider:cameraMatrix", tl.x(), tl.y(), tl.z(), tr.x(), tr.y(), tr.z(), 1, ColorRGBA(0, 255, 0));
    LINE3D("module:CameraMatrixProvider:cameraMatrix", tl.x(), tl.y(), tl.z(), bl.x(), bl.y(), bl.z(), 1, ColorRGBA(0, 255, 0));
    LINE3D("module:CameraMatrixProvider:cameraMatrix", bl.x(), bl.y(), bl.z(), br.x(), br.y(), br.z(), 1, ColorRGBA(0, 255, 0));
    LINE3D("module:CameraMatrixProvider:cameraMatrix", tr.x(), tr.y(), tr.z(), br.x(), br.y(), br.z(), 1, ColorRGBA(0, 255, 0));
    LINE3D("module:CameraMatrixProvider:cameraMatrix", tl.x(), tl.y(), tl.z(), ce.x(), ce.y(), ce.z(), 1, ColorRGBA(0, 255, 0));
    LINE3D("module:CameraMatrixProvider:cameraMatrix", tr.x(), tr.y(), tr.z(), ce.x(), ce.y(), ce.z(), 1, ColorRGBA(0, 255, 0));
    LINE3D("module:CameraMatrixProvider:cameraMatrix", bl.x(), bl.y(), bl.z(), ce.x(), ce.y(), ce.z(), 1, ColorRGBA(0, 255, 0));
    LINE3D("module:CameraMatrixProvider:cameraMatrix", br.x(), br.y(), br.z(), ce.x(), ce.y(), ce.z(), 1, ColorRGBA(0, 255, 0));
  }
}

void CameraMatrixProvider::camera2image(const Vector3f& camera, Vector2f& image) const
{
  const float& scale(theCameraInfo.focalLength / camera.x());
  image.x() = float(theCameraInfo.opticalCenter.x() - scale * camera.y());
  image.y() = float(theCameraInfo.opticalCenter.y() - scale * camera.z());
}

bool CameraMatrixProvider::intersectLineWithCullPlane(const Vector3f& lineBase, const Vector3f& lineDir,
                                                      Vector3f& point) const
{
  if(lineDir.x() == 0.f)
    return false;
  point = lineBase + lineDir * ((lineBase.x() - 200.0f) / -lineDir.x());
  return true;
}

void CameraMatrixProvider::drawFieldLines(const CameraMatrix& cameraMatrix) const
{
  Vector3f start0C, start1C, end0C, end1C;
  Vector2f start0I, start1I, end0I, end1I;

  const Pose2f& robotPoseInv(theRobotPose.inverse());
  const Pose3f& cameraMatrixInv(cameraMatrix.inverse());
  float halfFieldLinesWidth = theFieldDimensions.fieldLinesWidth / 2;

  for(unsigned int i = 0; i < theFieldDimensions.fieldLines.lines.size(); ++i)
  {
    const FieldDimensions::LinesTable::Line& line = theFieldDimensions.fieldLines.lines[i];
    Pose2f corner((line.to - line.from).angle(), line.from);
    float length = (line.to - line.from).norm();
    Pose2f relativeLine = robotPoseInv + corner;
    const Vector2f start0(Pose2f(relativeLine).translate(0, -halfFieldLinesWidth).translation);
    const Vector2f end1(Pose2f(relativeLine).translate(length, halfFieldLinesWidth).translation);

    start0C = cameraMatrixInv * Vector3f(start0.x(), start0.y(), 0.f); // field2camera
    end1C = cameraMatrixInv * Vector3f(end1.x(), end1.y(), 0.f); // field2camera

    if(start0C.x() <= 200 && end1C.x() <= 200)
      continue;

    const Vector2f& start1(Pose2f(relativeLine).translate(0, halfFieldLinesWidth).translation);
    const Vector2f& end0(Pose2f(relativeLine).translate(length, -halfFieldLinesWidth).translation);

    start1C = cameraMatrixInv * Vector3f(start1.x(), start1.y(), 0.f); // field2camera
    end0C = cameraMatrixInv * Vector3f(end0.x(), end0.y(), 0.f); // field2camera

    if(start0C.x() <= 200.f)
      intersectLineWithCullPlane(start0C, end0C - start0C, start0C);
    else if(end0C.x() <= 200.f)
      intersectLineWithCullPlane(start0C, end0C - start0C, end0C);
    if(start1C.x() <= 200.f)
      intersectLineWithCullPlane(start1C, end1C - start1C, start1C);
    else if(end1C.x() <= 200.f)
      intersectLineWithCullPlane(start1C, end1C - start1C, end1C);

    camera2image(start0C, start0I);
    camera2image(end0C, end0I);
    camera2image(start1C, start1I);
    camera2image(end1C, end1I);

    LINE("module:CameraMatrixProvider:calibrationHelper", start0I.x(), start0I.y(), end0I.x(), end0I.y(), 0, Drawings::solidPen, ColorRGBA::black);
    LINE("module:CameraMatrixProvider:calibrationHelper", start1I.x(), start1I.y(), end1I.x(), end1I.y(), 0, Drawings::solidPen, ColorRGBA::black);
  }

  start0C = cameraMatrixInv * Vector3f(100.f, -11.f, 0.f); // field2camera
  end0C = cameraMatrixInv * Vector3f(0.f, -11.f, 0.f); // field2camera
  camera2image(start0C, start0I);
  camera2image(end0C, end0I);

  LINE("module:CameraMatrixProvider:calibrationHelper", start0I.x(), start0I.y(), end0I.x(), end0I.y(), 0, Drawings::solidPen, ColorRGBA::blue);

  start0C = cameraMatrixInv * Vector3f(100.f, 11.f, 0.f); // field2camera
  end0C = cameraMatrixInv * Vector3f(0.f, 11.f, 0.f); // field2camera
  camera2image(start0C, start0I);
  camera2image(end0C, end0I);

  LINE("module:CameraMatrixProvider:calibrationHelper", start0I.x(), start0I.y(), end0I.x(), end0I.y(), 0, Drawings::solidPen, ColorRGBA::blue);

  start0C = cameraMatrixInv * Vector3f(110.f, 1000.f, 0.f); // field2camera
  end0C = cameraMatrixInv * Vector3f(110.f, -1000.f, 0.f); // field2camera
  camera2image(start0C, start0I);
  camera2image(end0C, end0I);

  LINE("module:CameraMatrixProvider:calibrationHelper", start0I.x(), start0I.y(), end0I.x(), end0I.y(), 0, Drawings::solidPen, ColorRGBA::blue);
}
