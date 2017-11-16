/**
 * @file CameraMatrixProvider.cpp
 * This file implements a class to calculate the position of the camera for the Nao.
 * @author <a href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</a>
 * @author Colin Graf
 */

#include "CameraMatrixProvider.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/RotationMatrix.h"

MAKE_MODULE(CameraMatrixProvider, perception);

void CameraMatrixProvider::update(CameraMatrix& cameraMatrix)
{
  cameraMatrix.computeCameraMatrix(theTorsoMatrix, theRobotCameraMatrix, theCameraCalibration);
  cameraMatrix.isValid = theTorsoMatrix.isValid && theMotionInfo.isMotionStable &&
                         (theFallDownState.state == FallDownState::upright ||
                          theFallDownState.state == FallDownState::squatting) &&
                         theFrameInfo.getTimeSince(theJointSensorData.timestamp) < 500 &&
                         (theRobotInfo.penalty == PENALTY_NONE || theRobotInfo.penalty == PENALTY_SPL_ILLEGAL_MOTION_IN_SET);

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

  DEBUG_DRAWING("module:CameraMatrixProvider:robotParts", "drawingOnImage")
  {
    if(p.footPoints.size() == 0)
    {
      InMapFile stream("modelPoints.cfg");
      if(stream.exists())
        stream >> p;
    }
    drawRobotParts();
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
  if(lineDir.x() == 0.)
    return false;
  point = lineBase + lineDir * ((lineBase.x() - 200.0f) / -lineDir.x());
  return true;
}

void CameraMatrixProvider::drawFieldLines(const CameraMatrix& cameraMatrix) const
{
  Vector3f start0C, start1C, end0C, end1C;
  Vector2f start0I, start1I, end0I, end1I;

  const Pose2f& robotPoseInv(theRobotPose.inversePose);
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

    start0C = cameraMatrixInv * Vector3f(start0.x(), start0.y(), 0.); // field2camera
    end1C = cameraMatrixInv * Vector3f(end1.x(), end1.y(), 0.); // field2camera

    if(start0C.x() <= 200 && end1C.x() <= 200)
      continue;

    const Vector2f& start1(Pose2f(relativeLine).translate(0, halfFieldLinesWidth).translation);
    const Vector2f& end0(Pose2f(relativeLine).translate(length, -halfFieldLinesWidth).translation);

    start1C = cameraMatrixInv * Vector3f(start1.x(), start1.y(), 0.); // field2camera
    end0C = cameraMatrixInv * Vector3f(end0.x(), end0.y(), 0.); // field2camera

    if(start0C.x() <= 200)
      intersectLineWithCullPlane(start0C, end0C - start0C, start0C);
    else if(end0C.x() <= 200)
      intersectLineWithCullPlane(start0C, end0C - start0C, end0C);
    if(start1C.x() <= 200)
      intersectLineWithCullPlane(start1C, end1C - start1C, start1C);
    else if(end1C.x() <= 200)
      intersectLineWithCullPlane(start1C, end1C - start1C, end1C);

    camera2image(start0C, start0I);
    camera2image(end0C, end0I);
    camera2image(start1C, start1I);
    camera2image(end1C, end1I);

    LINE("module:CameraMatrixProvider:calibrationHelper", start0I.x(), start0I.y(), end0I.x(), end0I.y(), 0, Drawings::solidPen, ColorRGBA::black);
    LINE("module:CameraMatrixProvider:calibrationHelper", start1I.x(), start1I.y(), end1I.x(), end1I.y(), 0, Drawings::solidPen, ColorRGBA::black);
  }

  start0C = cameraMatrixInv * Vector3f(100, -11, 0.); // field2camera
  end0C = cameraMatrixInv * Vector3f(0, -11, 0.); // field2camera
  camera2image(start0C, start0I);
  camera2image(end0C, end0I);

  LINE("module:CameraMatrixProvider:calibrationHelper", start0I.x(), start0I.y(), end0I.x(), end0I.y(), 0, Drawings::solidPen, ColorRGBA::blue);

  start0C = cameraMatrixInv * Vector3f(100, 11, 0.); // field2camera
  end0C = cameraMatrixInv * Vector3f(0, 11, 0.); // field2camera
  camera2image(start0C, start0I);
  camera2image(end0C, end0I);

  LINE("module:CameraMatrixProvider:calibrationHelper", start0I.x(), start0I.y(), end0I.x(), end0I.y(), 0, Drawings::solidPen, ColorRGBA::blue);

  start0C = cameraMatrixInv * Vector3f(110, 1000, 0.); // field2camera
  end0C = cameraMatrixInv * Vector3f(110, -1000, 0.); // field2camera
  camera2image(start0C, start0I);
  camera2image(end0C, end0I);

  LINE("module:CameraMatrixProvider:calibrationHelper", start0I.x(), start0I.y(), end0I.x(), end0I.y(), 0, Drawings::solidPen, ColorRGBA::blue);
}

void CameraMatrixProvider::drawRobotParts()
{
  for(unsigned int i = 0; i < p.thighIndex.size(); i += 3)
  {
    //for each triangle do
    Pose3f f[6] = {theRobotModel.limbs[Limbs::thighRight],
                   theRobotModel.limbs[Limbs::thighRight],
                   theRobotModel.limbs[Limbs::thighRight],
                   theRobotModel.limbs[Limbs::thighLeft],
                   theRobotModel.limbs[Limbs::thighLeft],
                   theRobotModel.limbs[Limbs::thighLeft]
                  };
    //triangle point indices
    int tIndex[3] = {p.thighIndex[i] * 3, p.thighIndex[i + 1] * 3, p.thighIndex[i + 2] * 3};

    Pose3f invertedCameraMatrix = theRobotCameraMatrix.inverse();

    Vector2f projected2Image[6];

    for(int k = 0; k < 3; ++k)
    {
      //add triangle point to ankle pose and convert from meter to mm
      f[k].translate(p.thighPoints[tIndex[k]] * 1000, p.thighPoints[tIndex[k] + 1] * 1000, p.thighPoints[tIndex[k] + 2] * 1000); //right
      f[k + 3].translate(p.thighPoints[tIndex[k]] * 1000, -p.thighPoints[tIndex[k] + 1] * 1000, p.thighPoints[tIndex[k] + 2] * 1000); //left

      //project into the image
      camera2image(invertedCameraMatrix * f[k].translation, projected2Image[k]); //right
      camera2image(invertedCameraMatrix * f[k + 3].translation, projected2Image[k + 3]); //left
    }

    // draw if it is in canvas
    for(int k = 0; k < 3; ++k)
    {
      if(k < 2)
      {
        if(projected2Image[k].x() > 0 && projected2Image[k + 1].x() > 0) //right
          LINE("module:CameraMatrixProvider:robotParts", projected2Image[k].x(), projected2Image[k].y(), projected2Image[k + 1].x(), projected2Image[k + 1].y(), 0, Drawings::solidPen, ColorRGBA::red);
        if(projected2Image[k + 3].x() > 0 && projected2Image[k + 4].x() > 0) //left
          LINE("module:CameraMatrixProvider:robotParts", projected2Image[k + 3].x(), projected2Image[k + 3].y(), projected2Image[k + 4].x(), projected2Image[k + 4].y(), 0, Drawings::solidPen, ColorRGBA::red);
      }
      else
      {
        if(projected2Image[k].x() > 0 && projected2Image[k - 2].x() > 0) //right
          LINE("module:CameraMatrixProvider:robotParts", projected2Image[k].x(), projected2Image[k].y(), projected2Image[k - 2].x(), projected2Image[k - 2].y(), 0, Drawings::solidPen, ColorRGBA::red);

        if(projected2Image[k + 3].x() > 0 && projected2Image[k + 1].x() > 0) //left
          LINE("module:CameraMatrixProvider:robotParts", projected2Image[k + 3].x(), projected2Image[k + 3].y(), projected2Image[k + 1].x(), projected2Image[k + 1].y(), 0, Drawings::solidPen, ColorRGBA::red);
      }
    }
  }

  for(unsigned int i = 0; i < p.shineIndex.size(); i += 3)
  {
    //for each triangle do
    Pose3f f[6] = {theRobotModel.limbs[Limbs::tibiaRight],
                   theRobotModel.limbs[Limbs::tibiaRight],
                   theRobotModel.limbs[Limbs::tibiaRight],
                   theRobotModel.limbs[Limbs::tibiaLeft],
                   theRobotModel.limbs[Limbs::tibiaLeft],
                   theRobotModel.limbs[Limbs::tibiaLeft]
                  };
    //triangle point indices
    int tIndex[3] = { p.shineIndex[i] * 3, p.shineIndex[i + 1] * 3, p.shineIndex[i + 2] * 3};

    Pose3f invertedCameraMatrix = theRobotCameraMatrix.inverse();

    Vector2f projected2Image[6];

    for(int k = 0; k < 3; ++k)
    {
      //add triangle point to ankle pose and convert from meter to mm
      f[k].translate(p.shinePoints[tIndex[k]] * 1000, p.shinePoints[tIndex[k] + 1] * 1000, p.shinePoints[tIndex[k] + 2] * 1000); //right
      f[k + 3].translate(p.shinePoints[tIndex[k]] * 1000, -p.shinePoints[tIndex[k] + 1] * 1000, p.shinePoints[tIndex[k] + 2] * 1000); //left

      //project into the image
      camera2image(invertedCameraMatrix * f[k].translation, projected2Image[k]); //right
      camera2image(invertedCameraMatrix * f[k + 3].translation, projected2Image[k + 3]); //left
    }
    // draw if it is in canvas
    for(int k = 0; k < 3; ++k)
    {
      if(k < 2)
      {
        if(projected2Image[k].x() > 0 && projected2Image[k + 1].x() > 0) //right
          LINE("module:CameraMatrixProvider:robotParts", projected2Image[k].x(), projected2Image[k].y(), projected2Image[k + 1].x(), projected2Image[k + 1].y(), 0, Drawings::solidPen, ColorRGBA::green);
        if(projected2Image[k + 3].x() > 0 && projected2Image[k + 4].x() > 0) //left
          LINE("module:CameraMatrixProvider:robotParts", projected2Image[k + 3].x(), projected2Image[k + 3].y(), projected2Image[k + 4].x(), projected2Image[k + 4].y(), 0, Drawings::solidPen, ColorRGBA::green);
      }
      else
      {
        if(projected2Image[k].x() > 0 && projected2Image[k - 2].x() > 0) //left
          LINE("module:CameraMatrixProvider:robotParts", projected2Image[k].x(), projected2Image[k].y(), projected2Image[k - 2].x(), projected2Image[k - 2].y(), 0, Drawings::solidPen, ColorRGBA::green);

        if(projected2Image[k + 3].x() > 0 && projected2Image[k + 1].x() > 0) //right
          LINE("module:CameraMatrixProvider:robotParts", projected2Image[k + 3].x(), projected2Image[k + 3].y(), projected2Image[k + 1].x(), projected2Image[k + 1].y(), 0, Drawings::solidPen, ColorRGBA::green);
      }
    }
  }

  for(unsigned int i = 0; i < p.footIndex.size(); i += 3)
  {
    //for each triangle of foot model do
    Pose3f f[9] = {theRobotModel.limbs[Limbs::footRight],
                   theRobotModel.limbs[Limbs::footRight],
                   theRobotModel.limbs[Limbs::footRight],
                   theRobotModel.limbs[Limbs::footLeft],
                   theRobotModel.limbs[Limbs::footLeft],
                   theRobotModel.limbs[Limbs::footLeft],
                   theRobotModel.limbs[Limbs::footRight], //for drawing of mirrored right foot
                   theRobotModel.limbs[Limbs::footRight],
                   theRobotModel.limbs[Limbs::footRight]
                  }; //draw mirrored right foot in order to compare where the left foot should be given a stand position and a right foot position
    //triangle point indices
    int tIndex[3] = { p.footIndex[i] * 3, p.footIndex[i + 1] * 3, p.footIndex[i + 2] * 3};

    Pose3f invertedCameraMatrix = theRobotCameraMatrix.inverse();

    Vector2f projected2Image[9];

    for(int k = 0; k < 3; ++k)
    {
      //mirror right
      f[k + 6].translation.y() *= -1.f;
      f[k + 6].rotateZ(-RotationMatrix(f[k + 6].rotation).getZAngle() * 2);

      //add triangle point to ankle pose and convert from meter to mm
      f[k].translate(p.footPoints[tIndex[k]] * 1000, p.footPoints[tIndex[k] + 1] * 1000, p.footPoints[tIndex[k] + 2] * 1000); //right
      f[k + 3].translate(p.footPoints[tIndex[k]] * 1000, -p.footPoints[tIndex[k] + 1] * 1000, p.footPoints[tIndex[k] + 2] * 1000); //left
      f[k + 6].translate(p.footPoints[tIndex[k]] * 1000, -p.footPoints[tIndex[k] + 1] * 1000, p.footPoints[tIndex[k] + 2] * 1000); //mirrored right

      //project into the image
      camera2image(invertedCameraMatrix * f[k].translation, projected2Image[k]); //right
      camera2image(invertedCameraMatrix * f[k + 3].translation, projected2Image[k + 3]); //left
      camera2image(invertedCameraMatrix * f[k + 6].translation, projected2Image[k + 6]); //mirrored right
    }
    // draw if it is in canvas
    for(int k = 0; k < 3; ++k)
    {
      if(k < 2)
      {
        if(projected2Image[k].x() > 0 && projected2Image[k + 1].x() > 0) //right
          LINE("module:CameraMatrixProvider:robotParts", projected2Image[k].x(), projected2Image[k].y(), projected2Image[k + 1].x(), projected2Image[k + 1].y(), 0, Drawings::solidPen, ColorRGBA::blue);
        if(projected2Image[k + 3].x() > 0 && projected2Image[k + 4].x() > 0) //left
          LINE("module:CameraMatrixProvider:robotParts", projected2Image[k + 3].x(), projected2Image[k + 3].y(), projected2Image[k + 4].x(), projected2Image[k + 4].y(), 0, Drawings::solidPen, ColorRGBA::blue);
        if(projected2Image[k + 6].x() > 0 && projected2Image[k + 7].x() > 0) //mirrored right
          LINE("module:CameraMatrixProvider:robotParts", projected2Image[k + 6].x(), projected2Image[k + 6].y(), projected2Image[k + 7].x(), projected2Image[k + 7].y(), 0, Drawings::solidPen, ColorRGBA::yellow);
      }
      else
      {
        if(projected2Image[k].x() > 0 && projected2Image[k - 2].x() > 0) //right
          LINE("module:CameraMatrixProvider:robotParts", projected2Image[k].x(), projected2Image[k].y(), projected2Image[k - 2].x(), projected2Image[k - 2].y(), 0, Drawings::solidPen, ColorRGBA::blue);
        if(projected2Image[k + 3].x() > 0 && projected2Image[k + 1].x() > 0) //left
          LINE("module:CameraMatrixProvider:robotParts", projected2Image[k + 3].x(), projected2Image[k + 3].y(), projected2Image[k + 1].x(), projected2Image[k + 1].y(), 0, Drawings::solidPen, ColorRGBA::blue);
        if(projected2Image[k + 6].x() > 0 && projected2Image[k + 4].x() > 0) //mirrored right
          LINE("module:CameraMatrixProvider:robotParts", projected2Image[k + 6].x(), projected2Image[k + 6].y(), projected2Image[k + 4].x(), projected2Image[k + 4].y(), 0, Drawings::solidPen, ColorRGBA::yellow);
      }
    }
  }
}
