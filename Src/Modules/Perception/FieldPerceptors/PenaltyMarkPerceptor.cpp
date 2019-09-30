/**
 * @file PenaltyMarkPerceptor.cpp
 *
 * This file implements a module that searches a number of regions for a penalty mark.
 *
 * @author Thomas Röfer
 */

#include "PenaltyMarkPerceptor.h"
#include "Platform/File.h"
#include "Tools/ImageProcessing/CNS/CNSTools.h"
#include "Tools/ImageProcessing/InImageSizeCalculations.h"
#include "Tools/Math/Transformation.h"

MAKE_MODULE(PenaltyMarkPerceptor, perception)

PenaltyMarkPerceptor::PenaltyMarkPerceptor()
{
  // Create mesh for penalty mark
  const float outer = theFieldDimensions.penaltyMarkSize / 2.f;
  const float inner = theFieldDimensions.fieldLinesWidth / 2.f;
  TriangleMesh mesh(
  {
    -inner, -outer, 0.f,  inner, -outer, 0.f,                                         //   0 1
    -outer, -inner, 0.f, -inner, -inner, 0.f, inner, -inner, 0.f, outer, -inner, 0.f, // 2 3 4 5
    -outer,  inner, 0.f, -inner,  inner, 0.f, inner,  inner, 0.f, outer,  inner, 0.f, // 6 7 8 9
    -inner,  outer, 0.f,  inner,  outer, 0.f,                                         //   A B
    0.f,    0.f,  -1.f,                                                              //    C
  },
  {
    0, 3, 1,
    1, 3, 4,
    2, 6, 3,
    3, 6, 7,
    3, 7, 4,
    4, 7, 8,
    4, 8, 5,
    5, 8, 9,
    7, 10, 8,
    8, 10, 11,
    0, 1, 12,
    1, 4, 12,
    4, 5, 12,
    5, 9, 12,
    9, 8, 12,
    8, 11, 12,
    11, 10, 12,
    10, 7, 12,
    7, 6, 12,
    6, 2, 12,
    2, 3, 12,
    3, 0, 12,
  });

  // Create or load contour table
  detector.create(mesh,
                  CameraModelOpenCV(),
                  Eigen::AlignedBox3d(Vector3d(-maxTableRadius, -maxTableRadius, minTableHeight),
                                      Vector3d(maxTableRadius, maxTableRadius, maxTableHeight)),
                  spacing,
                  SearchSpecification(),
                  ParametricLaplacian(),
                  (std::string(File::getBHDir()) + "/Config/CNS/penaltyMarkPerceptor.dat").c_str());

  samplePoints =
  {
    Vector3d(0.5 * -inner, -0.75 * outer, 0), Vector3d(0.5 * inner, -0.75 * outer, 0),
    Vector3d(0.5 * -inner, -0.25 * outer, 0), Vector3d(0.5 * inner, -0.25 * outer, 0),
    Vector3d(0.75 * -outer, -0.5 * inner, 0), Vector3d(0.75 * outer, -0.5 * inner, 0),
    Vector3d(0.75 * -outer,  0.5 * inner, 0), Vector3d(0.75 * outer,  0.5 * inner, 0),
    Vector3d(0.25 * -outer, -0.5 * inner, 0), Vector3d(0.25 * outer, -0.5 * inner, 0),
    Vector3d(0.25 * -outer,  0.5 * inner, 0), Vector3d(0.25 * outer,  0.5 * inner, 0),
    Vector3d(0.5 * -inner,  0.25 * outer, 0), Vector3d(0.5 * inner,  0.25 * outer, 0),
    Vector3d(0.5 * -inner,  0.75 * outer, 0), Vector3d(0.5 * inner,  0.75 * outer, 0),
  };
}

void PenaltyMarkPerceptor::update(PenaltyMarkPercept& thePenaltyMarkPercept)
{
  DECLARE_DEBUG_DRAWING("module:PenaltyMarkPerceptor:around", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:PenaltyMarkPerceptor:candidates", "drawingOnImage");

  thePenaltyMarkPercept.wasSeen = false;
  detector.setCamera(CNS::toCameraModelOpenCV(theCameraInfo, theCameraIntrinsics));

  if(theCameraMatrix.isValid)
  {
    updateSearchSpace();

    ObjectCNSStereoDetector::IsometryWithResponses objects;

    for(const Boundaryi& region : thePenaltyMarkRegions.regions)
    {
      spec.blockX = region.x.getSize();
      spec.blockY = region.y.getSize();
      detector.setSearchSpecification(spec);
      ObjectCNSStereoDetector::IsometryWithResponses newObjects;
      detector.searchBlockAllPoses(newObjects, theCNSImage, region.x.min, region.y.min);

      if(spec.nRefineIterations > 0)
        for(IsometryWithResponse& object : newObjects)
        {
          detector.refine(theCNSImage, object, spec.nRefineIterations);
        }

      for(const IsometryWithResponse& object : newObjects)
      {
        if(object.response >= minResponse)
          objects.emplace_back(object);
        COMPLEX_DRAWING("module:PenaltyMarkPerceptor:candidates")
        {
          drawContourViaLutRasterizer(detector.lr, object, detector.camera, object.response >= minResponse ? ColorRGBA::green : ColorRGBA::yellow);
          double x, y;
          detector.camera.camera2Image(Vector3d(object(0, 3), object(1, 3), object(2, 3)), x, y);
          TIP("module:PenaltyMarkPerceptor:candidates", x, y, 20.f, object.response);
        }
      }
    }

    std::sort(objects.begin(), objects.end(), MoreOnResponse());
    for(const IsometryWithResponse& object : objects)
    {
      int nonWhitePoints = 0;
      double x, y;
      for(const Vector3d& samplePoint : samplePoints)
      {
        detector.camera.camera2Image(object * samplePoint, x, y);
        if(theECImage.colored[static_cast<int>(y)][static_cast<int>(x)] != FieldColors::white
           && ++nonWhitePoints > maxNonWhiteRatio * samplePoints.size())
          goto nextObject;
      }

      detector.camera.camera2Image(Vector3d(object(0, 3), object(1, 3), object(2, 3)), x, y);
      thePenaltyMarkPercept.positionInImage = Vector2i(static_cast<int>(x), static_cast<int>(y));
      float expectedWidth;
      float expectedHeight;
      if(IISC::calculateImagePenaltyMeasurementsByCenter(Vector2f(static_cast<float>(x), static_cast<float>(y)),
                                                         expectedWidth, expectedHeight, theCameraInfo, theCameraMatrix,
                                                         theFieldDimensions)
         && countGreen(thePenaltyMarkPercept.positionInImage, expectedWidth / 2.f, expectedHeight / expectedWidth)
         >= numberOfGreenChecks * greenCheckRadiusRatios.size() * minAroundGreenRatio
         && Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(Vector2f(static_cast<float>(x), static_cast<float>(y))),
                                         theCameraMatrix, theCameraInfo, thePenaltyMarkPercept.positionOnField)
         && thePenaltyMarkPercept.positionOnField.norm() <= maxTableRadius)
      {
        Matrix4d imageInCamera;
        imageInCamera << 0,  0,  1, 0,
                      -1,  0,  0, 0,
                      0, -1,  0, 0,
                      0,  0,  0, 1;
        Pose3f objectInRobot = theCameraMatrix * CNS::toPose3f(imageInCamera * object.matrix());
        thePenaltyMarkPercept.quarterRotationOnField = Angle::normalize(std::atan2(objectInRobot.rotation(1, 0),
                                                       objectInRobot.rotation(0, 0)) * 4.f) / 4.f;
        thePenaltyMarkPercept.wasSeen = true;
        break;
      }
    nextObject:
      ;
    }
  }

  draw();
}

void PenaltyMarkPerceptor::updateSearchSpace()
{
  Matrix4d cameraInImage;
  cameraInImage << 0, -1,  0, 0,
                0,  0, -1, 0,
                1,  0,  0, 0,
                0,  0,  0, 1;
  Matrix4d cameraInRingCenter = CNS::toMatrix4d(theCameraMatrix);
  Matrix4d ringCenterInImage = cameraInImage * cameraInRingCenter.inverse();
  Pose3f p = CNS::toPose3f(ringCenterInImage);
  spec.positionSpace = CylinderRing(Eigen::Isometry3d(ringCenterInImage),
                                    0, maxTableRadius, -1.f, 1.f);
  spec.nRefineIterations = refineIterations;
  spec.stepInPixelDuringRefinement = refineStepSize;
  spec.object2WorldOrientation.clear();
  Vector3f up = theCameraMatrix.rotation.inverse() * Vector3f::UnitZ();
  for(int i = 0; i < numOfRotations; ++i)
    spec.object2WorldOrientation.push_back(fromTo(Vector3d::UnitZ(), Vector3d(-up.y(), -up.z(), up.x()))
                                           * Eigen::Isometry3d(Eigen::AngleAxisd(90_deg * i / numOfRotations, Vector3d::UnitZ())));
}

int PenaltyMarkPerceptor::countGreen(const Vector2i& center, float radius, float yRatio) const
{
  int greenCount = 0;
  for(float radiusRatio : greenCheckRadiusRatios)
    greenCount += countGreenAtRadius(center, radius * radiusRatio, yRatio);
  return greenCount;
}

int PenaltyMarkPerceptor::countGreenAtRadius(const Vector2i& center, float radius, float yRatio) const
{
  std::function<bool(int, int)> isInside;
  if(center.x() - radius > 0 && center.x() + radius + 1 < theECImage.colored.width &&
     center.y() - radius > 0 && center.y() + radius + 1 < theECImage.colored.height)
    isInside = [&](int, int) -> bool {return true;};
  else
    isInside = [&](int x, int y) -> bool
  {
    return x >= 0 && x < static_cast<int>(theECImage.colored.width) &&
    y >= 0 && y < static_cast<int>(theECImage.colored.height);
  };
  const float epsilon = pi2 / static_cast<float>(numberOfGreenChecks);
  float dx = -radius;
  float dy = 0.f;
  int greenCount = 0;
  for(int i = 0; i < numberOfGreenChecks; ++i)
  {
    int x = center.x() + static_cast<int>(std::round(dx));
    int y = center.y() + static_cast<int>(std::round(dy * yRatio));
    if(isInside(x, y))
    {
      const FieldColors::Color color = theECImage.colored[y][x];
      if(color == FieldColors::field)
      {
        ++greenCount;
        DOT("module:PenaltyMarkPerceptor:around", x, y, ColorRGBA::green, ColorRGBA::green);
      }
      else
        DOT("module:PenaltyMarkPerceptor:around", x, y, ColorRGBA::yellow, ColorRGBA::yellow);
    }
    dx -= dy * epsilon;
    dy += dx * epsilon;
  }
  return greenCount;
}

void PenaltyMarkPerceptor::draw()
{
  spec.blockX = 64;
  spec.blockY = 48;
  detector.setSearchSpecification(spec);

  DEBUG_DRAWING("module:PenaltyMarkPerceptor:ring", "drawingOnImage")
    drawCylinderRing(detector.spec.positionSpace, detector.camera);

  DEBUG_DRAWING("module:PenaltyMarkPerceptor:searchSpace", "drawingOnImage")
  {
    Contour ctAll;
    detector.renderSearchSpace(ctAll, detector.camera, 1);
    drawRasteredContour(ctAll, ColorRGBA::green);
  }
}

void PenaltyMarkPerceptor::drawContourViaLutRasterizer(const LutRasterizer& lr, const Eigen::Isometry3d& object2World, const CameraModelOpenCV& camera, const ColorRGBA& color) const
{
  CodedContour contour;
  lr.rasterize(contour, object2World, camera);
  for(unsigned i = 0; i < contour.size(); ++i)
  {
    int x = xOfCCP(contour[i]) + contour.referenceX,
        y = yOfCCP(contour[i]) + contour.referenceY;
    double alpha = angleOfCCP(contour[i]);
    double c = cos(alpha),
           s = sin(alpha);
    LINE("module:PenaltyMarkPerceptor:candidates", static_cast<int>(round(x - 2 * s)), static_cast<int>(round(y + 2 * c)),
         static_cast<int>(round(x + 2 * s)), static_cast<int>(round(y - 2 * c)), 1, Drawings::solidPen, color);
  }
}

void PenaltyMarkPerceptor::drawRasteredContour(const Contour& contour, const ColorRGBA& color) const
{
  for(unsigned i = 0; i < contour.size(); ++i)
  {
    int x = contour[i].x + contour.referenceX,
        y = contour[i].y + contour.referenceY;
    double alpha = contour[i].angle;
    double c = cos(alpha),
           s = sin(alpha);
    LINE("module:PenaltyMarkPerceptor:searchSpace", round(x - 2 * s), round(y + 2 * c),
         round(x + 2 * s), round(y - 2 * c), 1, Drawings::solidPen, color);
  }
}

void PenaltyMarkPerceptor::drawCylinderRing(const CylinderRing& cylinderRing, const CameraModelOpenCV& camera) const
{
  ColorRGBA colors[4] =
  {
    ColorRGBA(255, 255, 255),
    ColorRGBA(255, 0, 0),
    ColorRGBA(0, 255, 0),
    ColorRGBA(0, 0, 255)
  };
  for(int i = 0; i < 4; ++i)
  {
    ColorRGBA& color = colors[i];
    for(double alpha = 0; alpha < 2 * M_PI; alpha += 0.02)
    {
      double x, y;
      if(camera.world2ImageClipped(cylinderRing.sampleEdgePoint(alpha, i), x, y))
        DOT("module:PenaltyMarkPerceptor:ring", x, y, color, color);
    }
  }
}
