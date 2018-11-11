/**
 * @file NNUtilities.h
 * This file implements helper functions for neural networks
 * @author Bernd Poppinga
 */

#pragma once

#include "Tools/Math/Eigen.h"
#include "Tools/ImageProcessing/TImage.h"

namespace NNUtilities
{
  Matrix3f calcInverseTransformation(const Vector2i& center, const Vector2i& inSize, const Vector2i& outSize, const Angle rotation = 0_deg);
  
  template<typename OutType, bool interpolate = false>
  void getImageSection(const Vector2i& center, const Vector2i& inSize, const Vector2i& outSize, const GrayscaledImage& src, OutType* output);

  template<typename OutType>
  void getInterpolatedImageSection(const Vector2i& center, const Vector2i& inSize, const Vector2i& outSize, const GrayscaledImage& src, OutType* output, const Angle rotation = 0_deg);


  template<typename OutType>
  void normalizeContrast(OutType* output, const Vector2i size, const float percent=0.02f);
  void normalizeContrast(GrayscaledImage& output, const float percent = 0.02f);


  ENUM( ExtractionMode,
    { ,
     fast,
     fast_interpolated,
     interpolated,

    });
  
  template<typename OutType>
  void extractPatch(const Vector2i& center, const Vector2i& inSize, const Vector2i& outSize, const GrayscaledImage& src, OutType* dest, const ExtractionMode mode = fast);
  void extractPatch(const Vector2i& center, const Vector2i& inSize, const Vector2i& outSize, const GrayscaledImage& src, GrayscaledImage& dest, const ExtractionMode mode = fast);
}