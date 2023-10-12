/**
 * @file CNSBallSpotsProvider.cpp
 * This file implements a module that detects balls in CNS images.
 * @author Thomas Röfer
 * @author Udo Frese
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "CNSBallSpotsProvider.h"
#include "Platform/File.h"
#include "Debugging/DebugDrawings.h"
#include "Tools/CNS/CNSTools.h"
#include "ImageProcessing/CNS/LutRasterizer.h"
#include "Tools/Math/Transformation.h"

MAKE_MODULE(CNSBallSpotsProvider);

CNSBallSpotsProvider::CNSBallSpotsProvider()
{
  // Create or load contour table
  Eigen::AlignedBox3d viewpointRange(Vector3d(-maxTableRadius,
                                              -maxTableRadius,
                                              minTableHeight),
                                     Vector3d(maxTableRadius,
                                              maxTableRadius,
                                              maxTableHeight));
  const TriangleMesh& mesh = TriangleMesh::sphere(theBallSpecification.radius, contourSubDivisions);
  detector.create(mesh,
                  CameraModelOpenCV(),
                  viewpointRange,
                  spacing,
                  SearchSpecification(),
                  ParametricLaplacian(),
                  (std::string(File::getBHDir()) + "/Config/CNS/ballPerceptor.dat").c_str());
}

void CNSBallSpotsProvider::update(BallSpots& ballSpots)
{
  detector.setCamera(CNS::toCameraModelOpenCV(theCameraInfo));
  objects.clear();
  ballSpots.ballSpots.clear();
  ballSpots.firstSpotIsPredicted = false;

  if(usePrediction)
  {
    Vector2f predictionInImage;
    if(theFrameInfo.getTimeSince(theWorldModelPrediction.timeWhenBallLastSeen) < 100 &&
       Transformation::robotToImage(Vector3f(theWorldModelPrediction.ballPosition.x(), theWorldModelPrediction.ballPosition.y(), theBallSpecification.radius), theCameraMatrix, theCameraInfo, predictionInImage))
    {
      predictionInImage = theImageCoordinateSystem.fromCorrected(predictionInImage);
      const int x = static_cast<int>(std::round(predictionInImage.x())), y = static_cast<int>(std::round(predictionInImage.y()));

      if(x >= 0 && x < theCameraInfo.width && y >= 0 && y < theCameraInfo.height)
      {
        ballSpots.firstSpotIsPredicted = true;
        ballSpots.addBallSpot(x, y);
      }
    }
  }

  if(theCameraMatrix.isValid)
  {
    updateSearchSpace();

    for(const Boundaryi& region : theBallRegions.regions)
    {
      spec.blockX = region.x.getSize();
      spec.blockY = region.y.getSize();
      detector.setSearchSpecification(spec);
      ObjectCNSStereoDetector::IsometryWithResponses newObjects;
      detector.searchBlockAllPoses(newObjects, theCNSImage, region.x.min, region.y.min);

      if(spec.nRefineIterations > 0)
        for(IsometryWithResponse& object : newObjects)
          detector.refine(theCNSImage, object, spec.nRefineIterations);

      for(const IsometryWithResponse& object : newObjects)
        if(object.response >= minResponse)
          objects.emplace_back(object);
    }

    std::sort(objects.begin(), objects.end(), MoreOnResponse());
    for(IsometryWithResponse& ballSpot : objects)
    {
      double x, y;
      detector.camera.camera2Image(Vector3d(ballSpot(0, 3), ballSpot(1, 3), ballSpot(2, 3)), x, y);
      ballSpots.ballSpots.emplace_back(static_cast<int>(x), static_cast<int>(y));
    }
  }

  draw();
}

void CNSBallSpotsProvider::updateSearchSpace()
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
                                    0,
                                    maxTableRadius,
                                    theBallSpecification.radius - 1.f,
                                    theBallSpecification.radius + 1.f);
  spec.nRefineIterations = refineIterations;
  spec.stepInPixelDuringRefinement = refineStepSize;
  spec.object2WorldOrientation.clear();
  Vector3f up = theCameraMatrix.rotation.inverse() * Vector3f::UnitZ();
  spec.object2WorldOrientation.push_back(fromTo(Vector3d::UnitZ(), Vector3d(-up.y(), -up.z(), up.x())));
}

void CNSBallSpotsProvider::draw()
{
  spec.blockX = 64;
  spec.blockY = 48;
  detector.setSearchSpecification(spec);
  DEBUG_DRAWING("module:CNSBallSpotsProvider:ring", "drawingOnImage")
  {
    drawCylinderRing(detector.spec.positionSpace, detector.camera);
  }
  DEBUG_DRAWING("module:CNSBallSpotsProvider:searchSpace", "drawingOnImage")
  {
    Contour ctAll;
    detector.renderSearchSpace(ctAll, detector.camera, 1);
    drawRasteredContour(ctAll, ColorRGBA::green);
  }
}

void CNSBallSpotsProvider::drawRasteredContour(const Contour& contour, [[maybe_unused]] const ColorRGBA& color) const
{
  for(unsigned i = 0; i < contour.size(); ++i)
  {
    int x = contour[i].x + contour.referenceX,
        y = contour[i].y + contour.referenceY;
    double alpha = contour[i].angle;
    double c = cos(alpha),
           s = sin(alpha);
    LINE("module:CNSBallSpotsProvider:searchSpace", round(x - 2 * s), round(y + 2 * c),
         round(x + 2 * s), round(y - 2 * c), 1, Drawings::solidPen, color);
  }
}

void CNSBallSpotsProvider::drawCylinderRing(const CylinderRing& cylinderRing, const CameraModelOpenCV& camera) const
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
        DOT("module:CNSBallSpotsProvider:ring", x, y, color, color);
    }
  }
}
