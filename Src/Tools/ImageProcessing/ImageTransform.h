/**
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */
#pragma once

#include "Platform/BHAssert.h"
#include "Tools/Math/Eigen.h"

#include "Tools/ImageProcessing/AVX.h"
#include "Tools/ImageProcessing/Image.h"

namespace ImageTransform
{
  template<typename T>
  ALWAYSINLINE static __m128 setR2float(T, T, T, T);
  template<> ALWAYSINLINE __m128 setR2float(unsigned char v0, unsigned char v1, unsigned char v2, unsigned char v3)
  {
    return _mm_cvtepi32_ps(_mm_setr_epi32(v0, v1, v2, v3));
  }
  template<> ALWAYSINLINE __m128 setR2float(float v0, float v1, float v2, float v3)
  {
    return _mm_setr_ps(v0, v1, v2, v3);
  }

  template<typename T>
  ALWAYSINLINE static __m128 getPixel(const Image<T>& src, __m128 x_x_x_x_Dash, __m128 y_y_y_y_Dash, float defaultValue = 0.f)
  {
    int32_t xCoordinates[4];
    int32_t yCoordinates[4];

    const __m128i new_x_x_x_x_icoord = _mm_cvtps_epi32(x_x_x_x_Dash);
    const __m128i new_y_y_y_y_icoord = _mm_cvtps_epi32(y_y_y_y_Dash);

    // weight calculation
    const __m128 new_x_x_x_x_floor = _mm_cvtepi32_ps(new_x_x_x_x_icoord);
    const __m128 new_y_y_y_y_floor = _mm_cvtepi32_ps(new_y_y_y_y_icoord);

    const __m128 new_x_x_x_x_mod = _mm_sub_ps(x_x_x_x_Dash, new_x_x_x_x_floor); // = weight right pixel
    const __m128 new_y_y_y_y_mod = _mm_sub_ps(y_y_y_y_Dash, new_y_y_y_y_floor); // = weight lower pixel

    const __m128 subFrom1_new_x_x_x_x_mod = _mm_sub_ps(_mm_set1_ps(1.f), new_x_x_x_x_mod); // = weight left pixel
    const __m128 subFrom1_new_y_y_y_y_mod = _mm_sub_ps(_mm_set1_ps(1.f), new_y_y_y_y_mod); // = weight upper pixel

    //in image check
    const __m128i xInRange = _mm_and_si128(_mm_cmpgt_epi32(new_x_x_x_x_icoord, _mm_set1_epi32(-1)),
                                           _mm_cmpgt_epi32(_mm_set1_epi32(src.width - 1), new_x_x_x_x_icoord));

    const __m128i yInRange = _mm_and_si128(_mm_cmpgt_epi32(new_y_y_y_y_icoord, _mm_set1_epi32(-1)),
                                           _mm_cmpgt_epi32(_mm_set1_epi32(src.height - 1), new_y_y_y_y_icoord));

    const __m128i coordInRange = _mm_and_si128(xInRange, yInRange);

    const __m128 defaultValues = _mm_set1_ps(defaultValue);

    _mm_store_si128(reinterpret_cast<__m128i*>(xCoordinates), _mm_and_si128(coordInRange, new_x_x_x_x_icoord));
    _mm_store_si128(reinterpret_cast<__m128i*>(yCoordinates), _mm_and_si128(coordInRange, new_y_y_y_y_icoord));

    __m128 halfWay[2];
    for(int i = 0; i < 4; i += 2)
    {
      //TODO make this not so shitty
      const __m128 upperValues = setR2float(
                                   src[yCoordinates[i]][xCoordinates[i]], src[yCoordinates[i]][xCoordinates[i] + 1],
                                   src[yCoordinates[i + 1]][xCoordinates[i + 1]], src[yCoordinates[i + 1]][xCoordinates[i + 1] + 1]);

      const __m128 lowerValues = setR2float(
                                   src[yCoordinates[i] + 1][xCoordinates[i]], src[yCoordinates[i] + 1][xCoordinates[i] + 1],
                                   src[yCoordinates[i + 1] + 1][xCoordinates[i + 1]], src[yCoordinates[i + 1] + 1][xCoordinates[i + 1] + 1]);

      __m128 xWeights, yWeightUpper, yWeightLower;

      if(!i)
      {
        xWeights = _mm_unpacklo_ps(subFrom1_new_x_x_x_x_mod, new_x_x_x_x_mod);
        yWeightUpper = _mm_unpacklo_ps(subFrom1_new_y_y_y_y_mod, subFrom1_new_y_y_y_y_mod);
        yWeightLower = _mm_unpacklo_ps(new_y_y_y_y_mod, new_y_y_y_y_mod);
      }
      else
      {
        xWeights = _mm_unpackhi_ps(subFrom1_new_x_x_x_x_mod, new_x_x_x_x_mod);
        yWeightUpper = _mm_unpackhi_ps(subFrom1_new_y_y_y_y_mod, subFrom1_new_y_y_y_y_mod);
        yWeightLower = _mm_unpackhi_ps(new_y_y_y_y_mod, new_y_y_y_y_mod);
      }

      halfWay[i / 2] = _mm_mul_ps(xWeights, _mm_add_ps(_mm_mul_ps(yWeightUpper, upperValues), _mm_mul_ps(yWeightLower, lowerValues)));
    }

    const __m128 result = _mm_min_ps(_mm_and_ps(_mm_castsi128_ps(coordInRange), _mm_hadd_ps(halfWay[0], halfWay[1])), _mm_set1_ps(255.f));

    if(defaultValue == 0.f)
      return result;
    else
      return _mm_or_ps(_mm_andnot_ps(_mm_castsi128_ps(coordInRange), defaultValues), result);
  }

  // writes the result of the affine transformation into dest, which has to be a buffer with a sufficient large size
  void transform(const Image<PixelTypes::GrayscaledPixel>& src, float* destP, unsigned int dest_width, unsigned int dest_height, const Matrix3f& inverseTransformation, const Vector2f& relativTransformationCenter = Vector2f(0.5f, 0.5f), const float defaultValue = 0.f)
  {
    const __m128 a0 = _mm_set1_ps(inverseTransformation(0, 0));
    const __m128 a1 = _mm_set1_ps(inverseTransformation(0, 1));
    const __m128 a2 = _mm_set1_ps(inverseTransformation(0, 2));
    const __m128 a3 = _mm_set1_ps(inverseTransformation(1, 0));
    const __m128 a4 = _mm_set1_ps(inverseTransformation(1, 1));
    const __m128 a5 = _mm_set1_ps(inverseTransformation(1, 2));

    const __m128 _0123 = _mm_setr_ps(0, 1.f, 2.f, 3.f);

    const Vector2f transformationCenter = Vector2f(dest_width, dest_height).cwiseProduct(relativTransformationCenter);
    //const Vector2i transformationCenter_i = transformationCenter.cast<int>();
    const float startX = -transformationCenter.x();
    //const float endX = dest.width - transformationCenter.x();
    const float startY = -transformationCenter.y();
    //const float cendY = dest.height - transformationCenter.y();

    //set the 4 times a things
    const __m128 a0x4 = _mm_mul_ps(a0, _mm_set1_ps(4.f));
    const __m128 a3x4 = _mm_mul_ps(a3, _mm_set1_ps(4.f));
    //calculate the 0,0 transformation; which allows us to only accumulate afterwards
    __m128 preCalculated_x_x_x_x_Dash = _mm_add_ps(_mm_set1_ps(src.width * relativTransformationCenter.x()), _mm_add_ps(_mm_add_ps(a2, _mm_mul_ps(_mm_set1_ps(startY), a1)), _mm_mul_ps(_mm_add_ps(_mm_set1_ps(startX), _0123), a0)));
    __m128 preCalculated_y_y_y_y_Dash = _mm_add_ps(_mm_set1_ps(src.height * relativTransformationCenter.y()), _mm_add_ps(_mm_add_ps(a5, _mm_mul_ps(_mm_set1_ps(startY), a4)), _mm_mul_ps(_mm_add_ps(_mm_set1_ps(startX), _0123), a3)));

    ASSERT(dest_width % 4 == 0);
    for(unsigned y = 0; y < dest_height; ++y)
    {
      //calculate "const" y part of transform
      preCalculated_x_x_x_x_Dash = _mm_add_ps(preCalculated_x_x_x_x_Dash, a1);
      preCalculated_y_y_y_y_Dash = _mm_add_ps(preCalculated_y_y_y_y_Dash, a4);
      // copy so we dont change the precalculation of next line while adding the x part of the transform
      __m128 x_x_x_x_Dash = preCalculated_x_x_x_x_Dash;
      __m128 y_y_y_y_Dash = preCalculated_y_y_y_y_Dash;

      for(unsigned x = 0; x < dest_width; x += 4, destP += 4,

          // finish transformation for the next iteration
          x_x_x_x_Dash = _mm_add_ps(x_x_x_x_Dash, a0x4),
          y_y_y_y_Dash = _mm_add_ps(y_y_y_y_Dash, a3x4))
      {
        _mm_storeu_ps(destP, getPixel(src, x_x_x_x_Dash, y_y_y_y_Dash, defaultValue));
      }
    }
  }

  // just affine right now
  void transform(const Image<unsigned char>& src, Image<float>& dest, const Matrix3f& inverseTransformation, const Vector2f& relativTransformationCenter = Vector2f(0.5f, 0.5f))
  {
    const __m128 a0 = _mm_set1_ps(inverseTransformation(0, 0));
    const __m128 a1 = _mm_set1_ps(inverseTransformation(0, 1));
    const __m128 a2 = _mm_set1_ps(inverseTransformation(0, 2));
    const __m128 a3 = _mm_set1_ps(inverseTransformation(1, 0));
    const __m128 a4 = _mm_set1_ps(inverseTransformation(1, 1));
    const __m128 a5 = _mm_set1_ps(inverseTransformation(1, 2));

    const __m128 _0123 = _mm_setr_ps(0, 1.f, 2.f, 3.f);

    const Vector2f transformationCenter = Vector2f(dest.width, dest.height).cwiseProduct(relativTransformationCenter);
    //const Vector2i transformationCenter_i = transformationCenter.cast<int>();
    const float startX = -transformationCenter.x();
    //const float endX = dest.width - transformationCenter.x();
    const float startY = -transformationCenter.y();
    //const float cendY = dest.height - transformationCenter.y();

    float* destP = dest[0];

    //set the 4 times a things
    const __m128 a0x4 = _mm_mul_ps(a0, _mm_set1_ps(4.f));
    const __m128 a3x4 = _mm_mul_ps(a3, _mm_set1_ps(4.f));
    //calculate the 0,0 transformation; which allows us to only accumulate afterwards
    __m128 preCalculated_x_x_x_x_Dash = _mm_add_ps(_mm_set1_ps(src.width * relativTransformationCenter.x()), _mm_add_ps(_mm_add_ps(a2, _mm_mul_ps(_mm_set1_ps(startY), a1)), _mm_mul_ps(_mm_add_ps(_mm_set1_ps(startX), _0123), a0)));
    __m128 preCalculated_y_y_y_y_Dash = _mm_add_ps(_mm_set1_ps(src.height * relativTransformationCenter.y()), _mm_add_ps(_mm_add_ps(a5, _mm_mul_ps(_mm_set1_ps(startY), a4)), _mm_mul_ps(_mm_add_ps(_mm_set1_ps(startX), _0123), a3)));

    ASSERT(dest.width % 4 == 0);
    for(unsigned y = 0; y < dest.height; ++y)
    {
      //calculate "const" y part of transform
      preCalculated_x_x_x_x_Dash = _mm_add_ps(preCalculated_x_x_x_x_Dash, a1);
      preCalculated_y_y_y_y_Dash = _mm_add_ps(preCalculated_y_y_y_y_Dash, a4);
      // copy so we dont change the precalculation of next line while adding the x part of the transform
      __m128 x_x_x_x_Dash = preCalculated_x_x_x_x_Dash;
      __m128 y_y_y_y_Dash = preCalculated_y_y_y_y_Dash;

      for(unsigned x = 0; x < dest.width; x += 4, destP += 4,

          // finish transformation for the next iteration
          x_x_x_x_Dash = _mm_add_ps(x_x_x_x_Dash, a0x4),
          y_y_y_y_Dash = _mm_add_ps(y_y_y_y_Dash, a3x4))
      {
        _mm_storeu_ps(destP, getPixel(src, x_x_x_x_Dash, y_y_y_y_Dash));
      }
    }
  }

  void transform(const Image<float>& src, Image<unsigned char>& dest)
  {
    dest.setResolution(src.width, src.height);

    const float* const srcSSEend = src[0] + (src.width * src.height) / 16u * 16u;
    //const float*const srcEnd = &src[src.height - 1][src.width];

    __m128i* destP = reinterpret_cast<__m128i*>(dest[0]) - 1;

    const float* srcP = src[0] - 4;
    while(srcP < srcSSEend)
    {
      const __m128i scr0 = _mm_cvtps_epi32(_mm_load_ps(srcP += 4));
      const __m128i scr1 = _mm_cvtps_epi32(_mm_load_ps(srcP += 4));
      const __m128i scr2 = _mm_cvtps_epi32(_mm_load_ps(srcP += 4));
      const __m128i scr3 = _mm_cvtps_epi32(_mm_load_ps(srcP += 4));

      _mm_store_si128(++destP, _mm_packus_epi16(_mm_packs_epi32(scr0, scr1), _mm_packs_epi32(scr2, scr3)));
    }
  }

  ////////
  // TODO opimize below

  void polarTransform(const Image<float>& src, Image<float>& dest)
  {
    // for normal images first
    const float angelDiff = 2.f * pi / dest.width;
    const float rDiff = src.height / 2.f / dest.height;

    std::vector<float> precalcXDiff;
    std::vector<float> precalcYDiff;

    for(unsigned i = 0; i < dest.width; ++i)
    {
      const float angle = i * angelDiff;
      precalcXDiff.emplace_back(std::cos(angle) * rDiff);
      precalcYDiff.emplace_back(std::sin(angle) * rDiff);
    }

    ASSERT(dest.width % 4 == 0);
    for(unsigned x = 0; x < dest.width; x += 4)
    {
      __m128 x_x_x_x = _mm_set1_ps(src.width / 2.f);
      __m128 y_y_y_y = _mm_set1_ps(src.height / 2.f);

      const __m128 xDiff = _mm_loadu_ps(&precalcXDiff[x]);
      const __m128 yDiff = _mm_loadu_ps(&precalcYDiff[x]);

      for(unsigned y = 0; y < dest.height; ++y)
      {
        x_x_x_x = _mm_add_ps(x_x_x_x, xDiff);
        y_y_y_y = _mm_add_ps(y_y_y_y, yDiff);
        _mm_storeu_ps(&dest[y][x], getPixel(src, x_x_x_x, y_y_y_y));
      }
    }
  }

  void logPolarTransform(const Image<float>& src, Image<float>& dest)
  {
    // for normal images first
    const float angelDiff = 2.f * pi / dest.width;
    const float rDiff = std::log2(src.height / 2.f) / dest.height;

    std::vector<float> precalcXDirection;
    std::vector<float> precalcYDirection;

    for(unsigned i = 0; i < dest.width; ++i)
    {
      const float angle = i * angelDiff;
      precalcXDirection.emplace_back(std::cos(angle));
      precalcYDirection.emplace_back(std::sin(angle));
    }

    ASSERT(dest.width % 4 == 0);
    for(unsigned x = 0; x < dest.width; x += 4)
    {
      const __m128 center_x = _mm_set1_ps(src.width / 2.f);
      const __m128 center_y = _mm_set1_ps(src.height / 2.f);

      const __m128 xDir = _mm_loadu_ps(&precalcXDirection[x]);
      const __m128 yDir = _mm_loadu_ps(&precalcYDirection[x]);

      for(unsigned y = 0; y < dest.height; ++y)
      {
        const __m128 x_x_x_x = _mm_add_ps(center_x, _mm_mul_ps(xDir, _mm_set1_ps(std::pow(2.f, y * rDiff))));
        const __m128 y_y_y_y = _mm_add_ps(center_y, _mm_mul_ps(yDir, _mm_set1_ps(std::pow(2.f, y * rDiff))));
        _mm_storeu_ps(&dest[y][x], getPixel(src, x_x_x_x, y_y_y_y));
      }
    }
  }

  //todo make just one method
  void halfImagePolarTransform(const Image<float>& src, Image<float>& dest)
  {
    const float angelDiff = pi / dest.width;
    const float rDiff = std::min(src.height / 2.f, 1.f * src.width) / dest.height;

    std::vector<float> precalcXDiff;
    std::vector<float> precalcYDiff;

    for(int i = -static_cast<int>(dest.width) / 2; i < static_cast<int>(dest.width / 2); ++i)
    {
      const float angle = i * angelDiff;
      precalcXDiff.emplace_back(std::cos(angle) * rDiff);
      precalcYDiff.emplace_back(std::sin(angle) * rDiff);
    }

    ASSERT(dest.width % 4 == 0);
    for(unsigned x = 0; x < dest.width; x += 4)
    {
      __m128 x_x_x_x = _mm_setzero_ps();
      __m128 y_y_y_y = _mm_set1_ps(src.height / 2.f);

      const __m128 xDiff = _mm_loadu_ps(&precalcXDiff[x]);
      const __m128 yDiff = _mm_loadu_ps(&precalcYDiff[x]);

      for(unsigned y = 0; y < dest.height; ++y)
      {
        x_x_x_x = _mm_add_ps(x_x_x_x, xDiff);
        y_y_y_y = _mm_add_ps(y_y_y_y, yDiff);
        _mm_storeu_ps(&dest[y][x], getPixel(src, x_x_x_x, y_y_y_y));
      }
    }
  }
}
