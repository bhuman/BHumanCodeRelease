/**
 * @file KeypointsProvider.cpp
 *
 * This file implements a module that subsamples a centered patch from the
 * camera image and uses MoveNet to detect keypoints in it. The keypoints
 * correspond to different body parts of a single person.
 *
 * @author Thomas Röfer
 */

#include "KeypointsProvider.h"
#include "Debugging/DebugDrawings.h"
#include "Debugging/DebugImages.h"
#include "ImageProcessing/ColorModelConversions.h"
#include "Platform/File.h"
#include "Streaming/Global.h"
#include <CompiledNN2ONNX/Model.h>

MAKE_MODULE(KeypointsProvider);

KeypointsProvider::KeypointsProvider()
  : detector(&Global::getAsmjitRuntime())
{
#ifdef TARGET_ROBOT
  compileNetwork();
#endif
}

void KeypointsProvider::update(Keypoints& theKeypoints)
{
  DECLARE_DEBUG_DRAWING("module:KeypointsProvider:patch", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:KeypointsProvider:mask", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:KeypointsProvider:mask:rows", "drawingOnImage");
  DECLARE_DEBUG_RESPONSE("debug images:module:KeypointsProvider:patch");

  if(!theOptionalCameraImage.image.has_value())
  {
    FOREACH_ENUM(Keypoints::Keypoint, keypoint)
      theKeypoints.points[keypoint].valid = false;
    return;
  }

  if(mask.empty())
  {
#ifndef TARGET_ROBOT
    compileNetwork();
#endif
  }

  const CameraImage& theCameraImage = theOptionalCameraImage.image.value();
  const unsigned centerX = theCameraImage.width;
  const unsigned centerY = patchAtTop ? patchSize / 2 : theCameraImage.height / 2;

  if(mask.empty() || (theRobotPose.translation - lastRobotPosition).norm() > maskRecomputeThreshold)
  {
    createMask(centerX, centerY, patchSize, patchSize, detector.input(0).dims(0), detector.input(0).dims(1));
    lastRobotPosition = theRobotPose.translation;
  }

  ASSERT(detector.input(0).dims(0) == detector.input(0).dims(1));
  ASSERT(detector.input(0).dims(2) == 3);

  // The only patch size supported.
  const int height = patchSize;
  const int width = patchSize;

  theKeypoints.patchBoundary = Boundaryi(Rangei(centerX - width / 2, centerX + width / 2),
                                         Rangei(centerY - height / 2, centerY + height / 2));

  STOPWATCH("module:KeypointsProvider:extractPatch")
  {
    // Extract image data from a centered patch and copy it to network input.
    // Use different ways to extract the patch based on the input size.
    if(detector.input(0).dims(0) == 192 && patchSize == 384)
      extractPatch2to1(centerX, centerY, height, width, detector.input(0).data());
    else if(detector.input(0).dims(0) == 256 && patchSize == 384)
      extractPatch3to2(centerX, centerY, height, width, detector.input(0).data());
    else if(detector.input(0).dims(0) == static_cast<unsigned>(patchSize))
      extractPatch1to1(centerX, centerY, height, width, detector.input(0).data());
    else
      FAIL("Unsupported network input size");
  }

  // Apply the mask to remove input that cannot be a person standing in the middle.
  STOPWATCH("module:KeypointsProvider:applyMask") applyMask(detector.input(0).dims(0), detector.input(0).dims(1), detector.input(0).data());

  // Create image of patch with applied mask on demand.
  COMPLEX_IMAGE("module:KeypointsProvider:patch")
  {
    Image<PixelTypes::RGBPixel> patch(detector.input(0).dims(1), detector.input(0).dims(0));
    float* src = detector.input(0).data();
    for(PixelTypes::RGBPixel* dest = &patch(0, 0), * destEnd = dest + patch.width * patch.height; dest < destEnd; ++dest)
    {
      dest->r = static_cast<unsigned char>(*src++);
      dest->g = static_cast<unsigned char>(*src++);
      dest->b = static_cast<unsigned char>(*src++);
    }
    SEND_DEBUG_IMAGE("module:KeypointsProvider:patch", patch);
  }

  // Run network.
  STOPWATCH("module:KeypointsProvider:apply") detector.apply();

  // Use detected Keypoints to find referee gesture
  const Output* outputs = reinterpret_cast<Output*>(detector.output(0).data());

  // Scale the keypoints and write them into the result.
  FOREACH_ENUM(Keypoints::Keypoint, keypoint)
  {
    const Output& output = outputs[keypoint];
    Keypoints::Point& point = theKeypoints.points[keypoint];
    point.position = Vector2f(centerX + (output.x - 0.5f) * width,
                              centerY + (output.y - 0.5f) * height);
    point.valid = output.confidence >= minConfidence;
  }

  // The image mask.
  COMPLEX_DRAWING("module:KeypointsProvider:mask")
  {
    const Vector2f refereeOnField(theFieldDimensions.xPosHalfwayLine,
                                  (theFieldDimensions.yPosLeftTouchline + theFieldDimensions.yPosLeftFieldBorder) / 2.f
                                  * (theGameState.leftHandTeam ? 1 : -1));
    const Vector2f refereeOffset = refereeOnField - theRobotPose.translation;
    const float yScale = 3000.f / refereeOffset.norm();
    const float xScale = yScale * std::cos(std::abs(refereeOffset.angle()) - 90_deg);
    ELLIPSE("module:KeypointsProvider:mask", Vector2f(centerX, centerY + (maskCenterY - theCameraImage.height / 2) * yScale),
            maskRadius * xScale, maskRadius * yScale, 0, 1, Drawings::solidPen, ColorRGBA::red, Drawings::noBrush, ColorRGBA::red);
    RECTANGLE("module:KeypointsProvider:mask", centerX - maskWidth / 2 * xScale, centerY - height / 2,
              centerX + maskWidth / 2 * xScale, centerY + height / 2, 1, Drawings::solidPen, ColorRGBA::red);
  }

  // The image mask as individual rows.
  COMPLEX_DRAWING("module:KeypointsProvider:mask:rows")
  {
    const int patchSize = detector.input(0).dims(0);
    for(int y = 0; y < patchSize; ++y)
    {
      const int row = centerY + (y - patchSize / 2) * height / patchSize;
      LINE("module:KeypointsProvider:mask:rows",
           centerX + (mask[y].min - patchSize / 2) * width / patchSize, row,
           centerX + (mask[y].max - patchSize / 2) * width / patchSize, row,
           1, Drawings::solidPen, ColorRGBA(255, 255, 255, 128));
    }
  }
}

void KeypointsProvider::compileNetwork()
{
  NeuralNetworkONNX::CompilationSettings settings;
  settings.useCoreML = true;
  settings.numOfThreads = 2;
  detector.compile(NeuralNetworkONNX::Model(std::string(File::getBHDir()) + "/" + filename),
                   settings);
}

void KeypointsProvider::createMask(const unsigned centerX, const unsigned centerY,
                                   const int height, const int width, const int patchHeight, const int patchWidth)
{
  const CameraImage& theCameraImage = theOptionalCameraImage.image.value();
  const Vector2f refereeOnField(theFieldDimensions.xPosHalfwayLine,
                                (theFieldDimensions.yPosLeftTouchline + theFieldDimensions.yPosLeftFieldBorder) / 2.f
                                * (theGameState.leftHandTeam ? 1 : -1));
  const Vector2f refereeOffset = refereeOnField - theRobotPose.translation;
  const float yScale = 3000.f / refereeOffset.norm();
  const float xScale = std::cos(std::abs(refereeOffset.angle()) - 90_deg);

  mask.clear();
  const Geometry::Circle circle({static_cast<float>(centerX),
    static_cast<float>(centerY) + (maskCenterY - static_cast<float>(theCameraImage.height / 2)) * yScale}, maskRadius * yScale);
  const int maskWidth = std::min(width, static_cast<int>(this->maskWidth * xScale * yScale));
  for(int patchY = 0; patchY < patchHeight; ++patchY)
  {
    const int y = centerY - height / 2 + patchY * height / patchHeight;
    const Geometry::Line row(Vector2i(0, y), Vector2i(1, 0));
    Vector2f inter1, inter2;
    Rangei range(centerX - maskWidth / 2, centerX + maskWidth / 2);
    if(Geometry::getIntersectionOfLineAndCircle(row, circle, inter1, inter2) == 2)
    {
      range.min = std::max(static_cast<int>(centerX) - width / 2,
                           std::min(range.min, static_cast<int>(centerX) + static_cast<int>((std::min(inter1.x(), inter2.x()) - static_cast<float>(centerX)) * xScale)));
      range.max = std::min(static_cast<int>(centerX) + width / 2,
                           std::max(range.max, static_cast<int>(centerX) + static_cast<int>((std::max(inter1.x(), inter2.x()) -
                               static_cast<float>(centerX)) * xScale) + 1));
    }
    mask.emplace_back((range.min - centerX + width / 2) * patchWidth / width,
                      (range.max - centerX + width / 2) * patchWidth / width);
  }
}

void KeypointsProvider::applyMask(const int patchHeight, const int patchWidth, float* data) const
{
  for(int y = 0; y < patchHeight; ++y)
  {
    const Rangei& range = mask[y];
    float* row = data + y * patchWidth * 3;
    for(float* p = row, *pEnd = row + range.min * 3; p < pEnd;)
    {
      *p++ = 255.f;
      *p++ = 0.f;
      *p++ = 255.f;
    }
    for(float* p = row + range.max * 3, *pEnd = row + patchWidth * 3; p < pEnd;)
    {
      *p++ = 255.f;
      *p++ = 0.f;
      *p++ = 255.f;
    }
  }
}

void KeypointsProvider::extractPatch1to1(const unsigned centerX, const unsigned centerY,
                                         const int height, const int width, float* channel) const
{
  const CameraImage& theCameraImage = theOptionalCameraImage.image.value();
  for(unsigned y = centerY - height / 2; y < centerY + height / 2; ++y)
    for(const CameraImage::PixelType* pixel = &theCameraImage[y][centerX / 2 - width / 4],
        *pixelEnd = pixel + width / 2; pixel < pixelEnd; ++pixel)
    {
      const CameraImage::PixelType yuyv = *pixel;
      unsigned char r, g, b;
      ColorModelConversions::fromYUVToRGB(yuyv.y0, yuyv.u, yuyv.v, r, g, b);
      *channel++ = r;
      *channel++ = g;
      *channel++ = b;
      ColorModelConversions::fromYUVToRGB(yuyv.y1, yuyv.u, yuyv.v, r, g, b);
      *channel++ = r;
      *channel++ = g;
      *channel++ = b;
    }
}

void KeypointsProvider::extractPatch2to1(const unsigned centerX, const unsigned centerY,
                                         const int height, const int width, float* channel) const
{
  const CameraImage& theCameraImage = theOptionalCameraImage.image.value();
  for(unsigned y = centerY - height / 2; y < centerY + height / 2; y += 2)
    for(const CameraImage::PixelType* pixel = &theCameraImage[y][centerX / 2 - width / 4],
        *pixelEnd = pixel + width / 2; pixel < pixelEnd; ++pixel)
    {
      unsigned char r, g, b;
      ColorModelConversions::fromYUVToRGB(pixel->y1, pixel->u, pixel->v, r, g, b);
      *channel++ = r;
      *channel++ = g;
      *channel++ = b;
    }
}

void KeypointsProvider::extractPatch3to2(const unsigned centerX, const unsigned centerY,
                                         const int height, const int width, float* channel) const
{
  const CameraImage& theCameraImage = theOptionalCameraImage.image.value();
  for(unsigned y = centerY - height / 2; y < centerY + height / 2; y += 3)
  {
    for(const CameraImage::PixelType* pixel = &theCameraImage[y][centerX / 2 - width / 4],
                                    * pixelEnd = pixel + width / 2; pixel < pixelEnd;)
    {
      unsigned char r, g, b;
      ColorModelConversions::fromYUVToRGB(pixel->y0, pixel->u, pixel->v, r, g, b);
      *channel++ = r;
      *channel++ = g;
      *channel++ = b;
      ColorModelConversions::fromYUVToRGB(static_cast<unsigned char>((static_cast<unsigned short>(pixel->y1) + pixel[1].y0) >> 1),
                                          static_cast<unsigned char>((static_cast<unsigned short>(pixel->u) + pixel[1].u) >> 1),
                                          static_cast<unsigned char>((static_cast<unsigned short>(pixel->v) + pixel[1].v) >> 1),
                                          r, g, b);
      *channel++ = r;
      *channel++ = g;
      *channel++ = b;
      ++pixel;
      ColorModelConversions::fromYUVToRGB(pixel->y1, pixel->u, pixel->v, r, g, b);
      *channel++ = r;
      *channel++ = g;
      *channel++ = b;
      ++pixel;
      ColorModelConversions::fromYUVToRGB(static_cast<unsigned char>((static_cast<unsigned short>(pixel->y0) + pixel->y1) >> 1),
                                          pixel->u, pixel->v, r, g, b);
      *channel++ = r;
      *channel++ = g;
      *channel++ = b;
      ++pixel;
    }
    for(const CameraImage::PixelType* pixel = &theCameraImage[y + 1][centerX / 2 - width / 4],
                                    * pixel2 =  &theCameraImage[y + 2][centerX / 2 - width / 4],
                                    * pixelEnd = pixel + width / 2; pixel < pixelEnd;)
    {
      unsigned char r, g, b;
      ColorModelConversions::fromYUVToRGB(static_cast<unsigned char>((static_cast<unsigned short>(pixel->y0) + pixel2->y0) >> 1),
                                          static_cast<unsigned char>((static_cast<unsigned short>(pixel->u) + pixel2->u) >> 1),
                                          static_cast<unsigned char>((static_cast<unsigned short>(pixel->v) + pixel2->v) >> 1),
                                          r, g, b);
      *channel++ = r;
      *channel++ = g;
      *channel++ = b;
      ColorModelConversions::fromYUVToRGB(static_cast<unsigned char>((static_cast<unsigned short>(pixel->y1) + pixel[1].y0 + pixel2->y1 + pixel2[1].y0) >> 2),
                                          static_cast<unsigned char>((static_cast<unsigned short>(pixel->u) + pixel[1].u + pixel2->u + pixel2[1].u) >> 2),
                                          static_cast<unsigned char>((static_cast<unsigned short>(pixel->v) + pixel[1].v + pixel2->v + pixel2[1].v) >> 2),
                                          r, g, b);
      *channel++ = r;
      *channel++ = g;
      *channel++ = b;
      ++pixel;
      ++pixel2;
      ColorModelConversions::fromYUVToRGB(static_cast<unsigned char>((static_cast<unsigned short>(pixel->y1) + pixel2->y1) >> 1),
                                          static_cast<unsigned char>((static_cast<unsigned short>(pixel->u) + pixel2->u) >> 1),
                                          static_cast<unsigned char>((static_cast<unsigned short>(pixel->v) + pixel2->v) >> 1),
                                          r, g, b);
      *channel++ = r;
      *channel++ = g;
      *channel++ = b;
      ++pixel;
      ++pixel2;
      ColorModelConversions::fromYUVToRGB(static_cast<unsigned char>((static_cast<unsigned short>(pixel->y0) + pixel->y1 + pixel2->y0 + pixel2->y1) >> 2),
                                          static_cast<unsigned char>((static_cast<unsigned short>(pixel->u) + pixel2->u) >> 1),
                                          static_cast<unsigned char>((static_cast<unsigned short>(pixel->v) + pixel2->v) >> 1),
                                          r, g, b);
      *channel++ = r;
      *channel++ = g;
      *channel++ = b;
      ++pixel;
      ++pixel2;
    }
  }
}
