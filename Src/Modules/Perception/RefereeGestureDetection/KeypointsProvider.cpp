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
#include "ImageProcessing/ColorModelConversions.h"
#include "Platform/File.h"
#include "Streaming/Global.h"
#include <CompiledNN2ONNX/Model.h>

MAKE_MODULE(KeypointsProvider, perception);

KeypointsProvider::KeypointsProvider()
  : detector(&Global::getAsmjitRuntime())
{
  NeuralNetworkONNX::CompilationSettings settings;
  settings.useCoreML = true;
  detector.compile(NeuralNetworkONNX::Model(std::string(File::getBHDir()) + "/" + filename),
                   settings);
  createMask(384, 384, detector.input(0).dims(0), detector.input(0).dims(1));
}

void KeypointsProvider::update(Keypoints& theKeypoints)
{
  ASSERT(detector.input(0).dims(0) == detector.input(0).dims(1));
  ASSERT(detector.input(0).dims(2) == 3);

  // The only patch size supported.
  const int height = 384;
  const int width = 384;

  STOPWATCH("module:KeypointsProvider:extractPatch")
  {
    // Extract image data from a centered patch and copy it to network input.
    // Use different ways to extract the patch based on the input size.
    if(detector.input(0).dims(0) == 192)
      extractPatch2to1(height, width, detector.input(0).data());
    else if(detector.input(0).dims(0) == 256)
      extractPatch3to2(height, width, detector.input(0).data());
    else
      FAIL("Unsupported network input size");
  }

  // Apply the mask to remove input that cannot be a person standing in the middle.
  STOPWATCH("module:KeypointsProvider:applyMask")
    applyMask(detector.input(0).dims(0), detector.input(0).dims(1), detector.input(0).data());

  // Run network.
  STOPWATCH("module:KeypointsProvider:apply") detector.apply();

  // Use detected Keypoints to find referee gesture
  const Output* outputs = reinterpret_cast<Output*>(detector.output(0).data());

  // Scale the keypoints and write them into the result.
  FOREACH_ENUM(Keypoints::Keypoint, keypoint)
  {
    const Output& output = outputs[keypoint];
    Keypoints::Point& point = theKeypoints.points[keypoint];
    point.position = Vector2f(theCameraInfo.width / 2 + (output.x - 0.5f) * width,
                              theCameraInfo.height / 2 + (output.y - 0.5f) * height);
    point.valid = output.confidence >= minConfidence;
  }

  // The detection area.
  DEBUG_DRAWING("module:KeypointsProvider:patch", "drawingOnImage")
  {
    const Vector2f p1((theCameraInfo.width - width) / 2, (theCameraInfo.height - height) / 2);
    const Vector2f p2((theCameraInfo.width + width) / 2, (theCameraInfo.height + height) / 2);
    RECTANGLE("module:KeypointsProvider:patch", p1.x(), p1.y(), p2.x(), p2.y(), 1, Drawings::solidPen, ColorRGBA::red);
  }

  // The image mask.
  DEBUG_DRAWING("module:KeypointsProvider:mask", "drawingOnImage")
  {
    CIRCLE("module:KeypointsProvider:mask", theCameraInfo.width / 2, maskCenterY, maskRadius, 1,
           Drawings::solidPen, ColorRGBA::red, Drawings::noBrush, ColorRGBA::red);
    RECTANGLE("module:KeypointsProvider:mask", (theCameraInfo.width - maskWidth) / 2, (theCameraInfo.height - height) / 2,
              (theCameraInfo.width + maskWidth) / 2, (theCameraInfo.height + height) / 2, 1, Drawings::solidPen, ColorRGBA::red);
  }
}

void KeypointsProvider::createMask(const int height, const int width, const int patchHeight, const int patchWidth)
{
  const Geometry::Circle circle({static_cast<float>(theCameraInfo.width / 2), maskCenterY}, maskRadius);
  for(int patchY = 0; patchY < patchHeight; ++patchY)
  {
    const int y = (theCameraInfo.height - height) / 2 + patchY * height / patchHeight;
    const Geometry::Line row(Vector2i(0, y), Vector2i(1, 0));
    Vector2f inter1, inter2;
    const int maskWidth = std::min(width, this->maskWidth);
    Rangei range((theCameraInfo.width - maskWidth) / 2, (theCameraInfo.width + maskWidth) / 2);
    if(Geometry::getIntersectionOfLineAndCircle(row, circle, inter1, inter2) == 2)
    {
      range.min = std::max((theCameraInfo.width - width) / 2, std::min(range.min, static_cast<int>(std::min(inter1.x(), inter2.x()))));
      range.max = std::min((theCameraInfo.width + width) / 2, std::max(range.max, static_cast<int>(std::max(inter1.x(), inter2.x())) + 1));
    }
    mask.emplace_back((range.min - (theCameraInfo.width - width) / 2) * patchWidth / width,
                      (range.max - (theCameraInfo.width - width) / 2) * patchWidth / width);
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

void KeypointsProvider::extractPatch2to1(const int height, const int width, float* channel) const
{
  for(int y = (theCameraInfo.height - height) / 2; y < (theCameraInfo.height + height) / 2; y += 2)
    for(const CameraImage::PixelType* pixel = &theCameraImage[y][(theCameraInfo.width - width) / 4],
        *pixelEnd = pixel + width / 2; pixel < pixelEnd; ++pixel)
    {
      unsigned char r, g, b;
      ColorModelConversions::fromYUVToRGB(pixel->y1, pixel->u, pixel->v, r, g, b);
      *channel++ = r;
      *channel++ = g;
      *channel++ = b;
    }
}

void KeypointsProvider::extractPatch3to2(const int height, const int width, float* channel) const
{
  for(int y = (theCameraInfo.height - height) / 2; y < (theCameraInfo.height + height) / 2; y += 3)
  {
    for(const CameraImage::PixelType* pixel = &theCameraImage[y][theCameraInfo.width / 4 - width / 4],
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
    for(const CameraImage::PixelType* pixel = &theCameraImage[y + 1][theCameraInfo.width / 4 - width / 4],
                                    * pixel2 =  &theCameraImage[y + 2][theCameraInfo.width / 4 - width / 4],
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
