/**
 * @file PatchUtilities.cpp
 * This file implements helper functions for processing image patches
 * @author Bernd Poppinga
 */

#include "PatchUtilities.h"
#include "ImageProcessing/ImageTransform.h"
#include <iostream>
#include <cmath>

Matrix3f PatchUtilities::calcInverseTransformation(const Vector2i& center, const Vector2i& inSize, const Vector2i& outSize)
{
  const Vector2i upperLeft = (center.array() - inSize.array() / 2).matrix();
  const Vector2f transformationFactor = (inSize.cast<float>().array() / outSize.cast<float>().array()).matrix();
  // float sin = std::sin(rotation);
  // float cos = std::cos(rotation);

  Matrix3f inverseTransformation; // TODO check rotation
  inverseTransformation(0, 0) = transformationFactor.x();// *cos;
  inverseTransformation(0, 1) = 0; // transformationFactor.x() * (-sin);
  inverseTransformation(1, 0) = 0; // transformationFactor.y() * sin;
  inverseTransformation(1, 1) = transformationFactor.y();// *cos;

  inverseTransformation(0, 2) = static_cast<float>(upperLeft.x());// +outSize.x() * (1 - cos + sin) / 2 * transformationFactor.x();
  inverseTransformation(1, 2) = static_cast<float>(upperLeft.y());// +outSize.y() * (1 - sin - cos) / 2 * transformationFactor.y();
  return inverseTransformation;
}


template<typename OutType>
void PatchUtilities::getInterpolatedImageSection(const Vector2i& center, const Vector2i& inSize, const Vector2i& outSize, const GrayscaledImage& src, OutType* output)
{
  Matrix3f inverseTransformation = calcInverseTransformation(center, inSize, outSize);

  if(std::is_same<OutType, float>::value)
    ImageTransform::transform(src, reinterpret_cast<float*>(output), outSize.x(), outSize.y(), inverseTransformation, Vector2f(0.f, 0.f), 128.f);
  else
  {
    MatrixXf dest(outSize.x(), outSize.y());
    ImageTransform::transform(src, dest.data(), outSize.x(), outSize.y(), inverseTransformation, Vector2f(0.f, 0.f), 128.f);
    Eigen::Matrix<OutType, Eigen::Dynamic, Eigen::Dynamic> dest_ = dest.cast<OutType>();
    memcpy(output, dest_.data(), outSize.x()* outSize.y() * sizeof(OutType));
  }
}

template<typename OutType, bool interpolate>
void PatchUtilities::getImageSection(const Vector2i& center, const Vector2i& inSize, const Vector2i& outSize, const GrayscaledImage& src, OutType* output)
{
  const Vector2i upperLeft = (center.array() - inSize.array() / 2).matrix();
  const Vector2f stepSize = (inSize.cast<float>().array() / outSize.cast<float>().array()).matrix();

  // Calculate Y offset and steps
  int ySteps = outSize.y();
  float yImage = static_cast<float>(upperLeft.y());
  int yPatchOffset = 0;
  if(yImage < 0.f)
  {
    const float steps = ceilf(-yImage / stepSize.y());
    yImage += steps * stepSize.y();
    yPatchOffset = static_cast<int>(steps);
    ySteps -= std::min(ySteps, yPatchOffset);
  }
  const float yOvershoot = yImage + inSize.y() - src.height + (interpolate ? 1.f : 0.f);
  if(yOvershoot > 0)
    ySteps -= std::min(ySteps, static_cast<int>(ceilf(yOvershoot / stepSize.y())));

  // Calculate X offset and steps
  int xSteps = outSize.x();
  float xImageOffset = static_cast<float>(upperLeft.x());
  int xPatchOffset = 0;
  if(xImageOffset < 0.f)
  {
    const float steps = ceilf(-xImageOffset / stepSize.x());
    xImageOffset += steps * stepSize.x();
    xPatchOffset = static_cast<int>(steps);
    xSteps -= std::min(xSteps, xPatchOffset);
  }
  const float xOvershoot = xImageOffset + inSize.x() - src.width + (interpolate ? 1.f : 0.f);
  if(xOvershoot > 0)
    xSteps -= std::min(xSteps, static_cast<int>(ceilf(xOvershoot / stepSize.x())));

  // Fill output memory with background color if the patch is partly outside of the image
  if(ySteps < outSize.y() || xSteps < outSize.x())
  {
    static constexpr unsigned char fillColor = 128;
    std::fill_n(output, outSize.x() * outSize.y(), static_cast<OutType>(fillColor));
  }

  // Copy the patch
  OutType* dest = output + yPatchOffset * outSize.x(); // Check x or y
  if(xSteps < outSize.x())
  {
    const size_t xPatchSkip = outSize.x() - xSteps - xPatchOffset;
    if(!interpolate)
    {
      for(; ySteps; yImage += stepSize.y(), --ySteps)
      {
        const PixelTypes::GrayscaledPixel* row = src[static_cast<size_t>(yImage)];

        dest += xPatchOffset;
        float xImage = xImageOffset;
        for(size_t n = xSteps; n; xImage += stepSize.x(), --n)
          * dest++ = static_cast<OutType>(row[static_cast<size_t>(xImage)]);
        dest += xPatchSkip;
      }
    }
    else
    {
      for(; ySteps; yImage += stepSize.y(), --ySteps)
      {
        const size_t yIndex = static_cast<size_t>(yImage);
        const float yWeight1 = yImage - static_cast<float>(yIndex);
        const float yWeight0 = 1 - yWeight1;
        const PixelTypes::GrayscaledPixel* row0 = src[yIndex];
        const PixelTypes::GrayscaledPixel* row1 = src[yIndex + 1];

        dest += xPatchOffset;
        float xImage = xImageOffset;
        for(size_t n = xSteps; n; xImage += stepSize.x(), --n)
        {
          const size_t xIndex = static_cast<size_t>(xImage);
          const float xWeight1 = xImage - static_cast<float>(xIndex);
          const float xWeight0 = 1 - xWeight1;

          *dest++ = static_cast<OutType>(
                      std::min(
                        255.f,
                        yWeight0 * (static_cast<float>(row0[xIndex]) * xWeight0 + static_cast<float>(row0[xIndex + 1]) * xWeight1)
                        + yWeight1 * (static_cast<float>(row1[xIndex]) * xWeight0 + static_cast<float>(row1[xIndex + 1]) * xWeight1)
                      )
                    );
        }
        dest += xPatchSkip;
      }
    }
  }
  else
  {
    if(!interpolate)
    {
      for(; ySteps; yImage += stepSize.y(), --ySteps)
      {
        const PixelTypes::GrayscaledPixel* row = src[static_cast<size_t>(yImage)];

        float xImage = xImageOffset;
        for(size_t n = xSteps; n; xImage += stepSize.x(), --n)
          * dest++ = static_cast<OutType>(row[static_cast<size_t>(xImage)]);
      }
    }
    else
    {
      for(; ySteps; yImage += stepSize.y(), --ySteps)
      {
        const size_t yIndex = static_cast<size_t>(yImage);
        const float yWeight1 = yImage - static_cast<float>(yIndex);
        const float yWeight0 = 1 - yWeight1;
        const PixelTypes::GrayscaledPixel* row0 = src[yIndex];
        const PixelTypes::GrayscaledPixel* row1 = src[yIndex + 1];

        float xImage = xImageOffset;
        for(size_t n = xSteps; n; xImage += stepSize.x(), --n)
        {
          const size_t xIndex = static_cast<size_t>(xImage);
          const float xWeight1 = xImage - static_cast<float>(xIndex);
          const float xWeight0 = 1 - xWeight1;

          *dest++ = static_cast<OutType>(
                      std::min(
                        255.f,
                        yWeight0 * (static_cast<float>(row0[xIndex]) * xWeight0 + static_cast<float>(row0[xIndex + 1]) * xWeight1)
                        + yWeight1 * (static_cast<float>(row1[xIndex]) * xWeight0 + static_cast<float>(row1[xIndex + 1]) * xWeight1)
                      )
                    );
        }
      }
    }
  }
}

void PatchUtilities::normalizeContrast(GrayscaledImage& output, const float percent)
{
  normalizeContrast(output[0], Vector2i(output.width, output.height), percent);
}

template<typename OutType>
void PatchUtilities::normalizeContrast(OutType* output, const Vector2i& size, const float percent)
{
  Eigen::Map<Eigen::Matrix<OutType, Eigen::Dynamic, Eigen::Dynamic>> patch(output, size.x(), size.y());
  Eigen::Matrix<OutType, Eigen::Dynamic, 1> sorted = Eigen::Map<Eigen::Matrix<OutType, Eigen::Dynamic, 1>>(patch.data(), size.x() * size.y());
  std::sort(sorted.data(), sorted.data() + sorted.size());

  OutType min = sorted(static_cast<int>((sorted.size() - 1) * percent));
  OutType max = sorted(static_cast<int>((sorted.size() - 1) * (1.f - percent)));
  if(max == 0)
    patch.setConstant(0);
  else
    patch.array() = ((patch.array().max(min).min(max) - min).template cast<float>() * 255.f / (static_cast<float>(max - min))).template cast<OutType>();
}

template void PatchUtilities::normalizeContrast<float>(float* output, const Vector2i& size, const float percent);
template void PatchUtilities::normalizeContrast<unsigned char>(unsigned char* output, const Vector2i& size, const float percent);

template<typename OutType>
void PatchUtilities::normalizeBrightness(OutType* output, const Vector2i& size, const float percent)
{
  Eigen::Map<Eigen::Matrix<OutType, Eigen::Dynamic, Eigen::Dynamic>> patch(output, size.x(), size.y());
  Eigen::Matrix<OutType, Eigen::Dynamic, 1> sorted = Eigen::Map<Eigen::Matrix<OutType, Eigen::Dynamic, 1>>(patch.data(), size.x() * size.y());
  if(sorted.size() == 0)
    return;

  std::partial_sort(sorted.data(), sorted.data() + static_cast<size_t>((sorted.size() - 1) * percent) + 1,
                    sorted.data() + sorted.size(), std::greater<>());

  OutType max = sorted(static_cast<int>((sorted.size() - 1) * percent));
  if(max == 0)
    patch.setConstant(0);
  else
    patch.array() = (patch.array().min(max).template cast<float>() * 255.f / static_cast<float>(max)).template cast<OutType>();
}

template void PatchUtilities::normalizeBrightness<float>(float* output, const Vector2i& size, const float percent);
template void PatchUtilities::normalizeBrightness<unsigned char>(unsigned char* output, const Vector2i& size, const float percent);

void PatchUtilities::extractPatch(const Vector2i& center, const Vector2i& inSize, const Vector2i& outSize, const GrayscaledImage& src, GrayscaledImage& dest, const ExtractionMode mode)
{
  dest.setResolution(static_cast<unsigned int>(outSize(0)), static_cast<unsigned int>(outSize(1)));
  extractPatch(center, inSize, outSize, src, dest[0], mode);
}

template<typename OutType>
void PatchUtilities::extractPatch(const Vector2i& center, const Vector2i& inSize, const Vector2i& outSize, const GrayscaledImage& src, OutType* dest, const ExtractionMode mode)
{
  switch (mode)
  {
    case fast:
      getImageSection<OutType,false>(center, inSize, outSize, src, dest);
      break;
    case fastInterpolated:
      getImageSection<OutType, true>(center, inSize, outSize, src, dest);
      break;
    case interpolated:
      getInterpolatedImageSection<OutType>(center, inSize, outSize, src, dest);
      break;
  }
}

template void PatchUtilities::extractPatch<float>(const Vector2i& center, const Vector2i& inSize, const Vector2i& outSize, const GrayscaledImage& src, float* dest, const ExtractionMode mode);
template void PatchUtilities::extractPatch<unsigned char>(const Vector2i& center, const Vector2i& inSize, const Vector2i& outSize, const GrayscaledImage& src, unsigned char* dest, const ExtractionMode mode);

template<typename OutType, bool grayscale>
void PatchUtilities::extractInput(const YUYVImage& cameraImage, const Vector2i& patchSize, OutType* input)
{
  const unsigned int yStep = cameraImage.height / patchSize(1);
  const unsigned int xStep = cameraImage.width / patchSize(0);
  for(unsigned int i = yStep / 2; i < cameraImage.height; i += yStep)
    for(unsigned int j = xStep / 2; j < cameraImage.width; j += xStep)
    {
      if constexpr(grayscale)
        *(input++) = cameraImage[i][j].y0;
      else
      {
        *input++ = cameraImage[i][j].y0;
        *input++ = cameraImage[i][j].u;
        *input++ = cameraImage[i][j].v;
      }
    }
}
template void PatchUtilities::extractInput<std::uint8_t, true>(const YUYVImage& cameraImage, const Vector2i& patchSize, std::uint8_t* input);
template void PatchUtilities::extractInput<std::uint8_t, false>(const YUYVImage& cameraImage, const Vector2i& patchSize, std::uint8_t* input);
