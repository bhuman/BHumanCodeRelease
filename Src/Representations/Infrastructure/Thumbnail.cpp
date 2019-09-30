/**
 * @file Thumbnail.cpp
 *
 * Contains functionality for the thumbnail image.
 *
 * @author Felix Thielke
 */

#include "Thumbnail.h"
#include "Tools/ImageProcessing/SIMD.h"

void Thumbnail::toYUYV(Image<PixelTypes::YUYVPixel>& dest) const
{
  dest.setResolution(imageY.width / 2, imageY.height);

  const __m128i* srcY = reinterpret_cast<const __m128i*>(imageY[0]);
  __m128i* pDest = reinterpret_cast<__m128i*>(dest[0]);

  if(mode != Thumbnail::yuv)
  {
    if(mode == Thumbnail::grayscaleWithColorClasses)
    {
      for(size_t n = imageY.width * imageY.height / 16; n; --n)
      {
        const __m128i y = _mm_and_si128(_mm_load_si128(srcY++), _mm_set1_epi8(char(0xFC)));
        _mm_store_si128(pDest, _mm_unpacklo_epi8(y, _mm_set1_epi16(0x7F7F)));
        _mm_store_si128(pDest + 1, _mm_unpackhi_epi8(y, _mm_set1_epi16(0x7F7F)));
        pDest += 2;
      }
    }
    else
    {
      for(size_t n = imageY.width * imageY.height / 16; n; --n)
      {
        const __m128i y = _mm_load_si128(srcY++);
        _mm_store_si128(pDest, _mm_unpacklo_epi8(y, _mm_set1_epi16(0x7F7F)));
        _mm_store_si128(pDest + 1, _mm_unpackhi_epi8(y, _mm_set1_epi16(0x7F7F)));
        pDest += 2;
      }
    }
  }
  else
  {
    const __m128i* srcUV = reinterpret_cast<const __m128i*>(imageUV[0]);
    if(size_t overshoot = imageUV.width % 8)
    {
      overshoot = 8 - overshoot;
      for(size_t nY = imageUV.height; nY; --nY)
      {
        const __m128i* const baseSrcUV = srcUV;
        for(size_t i = 2; i; --i)
        {
          for(size_t nX = imageUV.width / 8 + 1; nX; --nX)
          {
            const __m128i y = _mm_loadu_si128(srcY);
            const __m128i uv = _mm_loadu_si128(srcUV);
            srcY++;
            srcUV++;

            _mm_storeu_si128(pDest, _mm_unpacklo_epi8(y, uv));
            _mm_storeu_si128(pDest + 1, _mm_unpackhi_epi8(y, uv));
            pDest += 2;
          }

          srcY = reinterpret_cast<const __m128i*>(reinterpret_cast<const short*>(srcY) - overshoot);
          pDest = reinterpret_cast<__m128i*>(reinterpret_cast<unsigned int*>(pDest) - overshoot);
          srcUV = baseSrcUV;
        }

        srcUV = reinterpret_cast<const __m128i*>(reinterpret_cast<const short*>(srcUV) + imageUV.width);
      }
    }
    else
    {
      for(size_t nY = imageUV.height; nY; --nY)
      {
        const __m128i* const baseSrcUV = srcUV;
        for(size_t i = 2; i; --i)
        {
          for(size_t nX = imageUV.width / 8; nX; --nX)
          {
            const __m128i y = _mm_load_si128(srcY++);
            const __m128i uv = _mm_load_si128(srcUV++);

            _mm_store_si128(pDest, _mm_unpacklo_epi8(y, uv));
            _mm_store_si128(pDest + 1, _mm_unpackhi_epi8(y, uv));
            pDest += 2;
          }

          srcUV = baseSrcUV;
        }

        srcUV += imageUV.width / 8;
      }
    }
  }
}

void Thumbnail::toECImage(ECImage& dest) const
{
  dest.grayscaled.setResolution(imageY.width * scale, imageY.height * scale);
  dest.colored.setResolution(imageY.width * scale, imageY.height * scale);
  dest.hued.setResolution(imageY.width * scale, imageY.height * scale);
  dest.saturated.setResolution(imageY.width * scale, imageY.height * scale);

  std::fill_n(dest.hued[0], dest.hued.width * dest.hued.height, PixelTypes::HuePixel(0));
  std::fill_n(dest.saturated[0], dest.saturated.width * dest.saturated.height, 0);

  if(mode != Thumbnail::grayscaleWithColorClasses)
    std::fill_n(dest.colored[0], dest.colored.width * dest.colored.height, FieldColors::Color::none);

  // Expand horizontally
  const __m128i* srcY = reinterpret_cast<const __m128i*>(imageY[0]);
  __m128i* pDestY = reinterpret_cast<__m128i*>(dest.grayscaled[0]);
  if(mode == Thumbnail::grayscaleWithColorClasses)
  {
    __m128i* pDestC = reinterpret_cast<__m128i*>(dest.colored[0]);
    for(size_t n = imageY.width * imageY.height / 16; n; --n)
    {
      const std::function<void(const __m128i, const size_t)> expand = [&expand, &pDestY, &pDestC](const __m128i p, const size_t scale)
      {
        if(scale > 1)
        {
          expand(_mm_unpacklo_epi8(p, p), scale >> 1);
          expand(_mm_unpackhi_epi8(p, p), scale >> 1);
        }
        else
        {
          _mm_store_si128(pDestY++, _mm_and_si128(p, _mm_set1_epi8(char(0xFC))));
          _mm_store_si128(pDestC++, _mm_and_si128(p, _mm_set1_epi8(3)));
        }
      };
      expand(_mm_load_si128(srcY++), scale);
    }
  }
  else
  {
    for(size_t n = imageY.width * imageY.height / 16; n; --n)
    {
      const std::function<void(const __m128i, const size_t)> expand = [&expand, &pDestY](const __m128i p, const size_t scale)
      {
        if(scale > 1)
        {
          expand(_mm_unpacklo_epi8(p, p), scale >> 1);
          expand(_mm_unpackhi_epi8(p, p), scale >> 1);
        }
        else
          _mm_store_si128(pDestY++, _mm_and_si128(p, _mm_set1_epi8(char(0xFC))));
      };
      expand(_mm_load_si128(srcY++), scale);
    }
  }

  // Expand vertically
  for(int y = imageY.height - 1; y >= 0; --y)
    for(unsigned int i = 0; i < scale; i++)
      memcpy(dest.grayscaled[0] + dest.grayscaled.width * y * scale + i * dest.grayscaled.width, dest.grayscaled[0] + dest.grayscaled.width * y, dest.grayscaled.width);
  if(mode == Thumbnail::grayscaleWithColorClasses)
  {
    for(int y = imageY.height - 1; y >= 0; --y)
      for(unsigned int i = 0; i < scale; i++)
        memcpy(dest.colored[0] + dest.colored.width * y * scale + i * dest.colored.width, dest.colored[0] + dest.colored.width * y, dest.colored.width);
  }
}

void Thumbnail::toCameraImage(CameraImage& dest) const
{
  dest.setResolution(imageY.width * scale / 2, imageY.height * scale);

  // Expand horizontally
  if(mode == grayscale)
  {
    const __m128i* srcY = reinterpret_cast<const __m128i*>(imageY[0]);
    __m128i* pDest = reinterpret_cast<__m128i*>(dest[0]);
    for(size_t n = imageY.width * imageY.height / 16; n; --n)
    {
      const std::function<void(const __m128i, const size_t)> expand = [&expand, &pDest](const __m128i p, const size_t scale)
      {
        if(scale > 1)
        {
          expand(_mm_unpacklo_epi8(p, p), scale >> 1);
          expand(_mm_unpackhi_epi8(p, p), scale >> 1);
        }
        else
        {
          _mm_storeu_si128(pDest++, _mm_unpacklo_epi8(p, _mm_set1_epi16(0x7F7F)));
          _mm_storeu_si128(pDest++, _mm_unpackhi_epi8(p, _mm_set1_epi16(0x7F7F)));
        }
      };
      expand(_mm_load_si128(srcY++), scale);
    }
  }
  else if(mode == grayscaleWithColorClasses)
  {
    const __m128i* srcY = reinterpret_cast<const __m128i*>(imageY[0]);
    __m128i* pDest = reinterpret_cast<__m128i*>(dest[0]);
    for(size_t n = imageY.width * imageY.height / 16; n; --n)
    {
      const std::function<void(const __m128i, const size_t)> expand = [&expand, &pDest](const __m128i p, const size_t scale)
      {
        if(scale > 1)
        {
          expand(_mm_unpacklo_epi8(p, p), scale >> 1);
          expand(_mm_unpackhi_epi8(p, p), scale >> 1);
        }
        else
        {
          _mm_storeu_si128(pDest++, _mm_unpacklo_epi8(p, _mm_set1_epi16(0x7F7F)));
          _mm_storeu_si128(pDest++, _mm_unpackhi_epi8(p, _mm_set1_epi16(0x7F7F)));
        }
      };
      expand(_mm_and_si128(_mm_load_si128(srcY++), _mm_set1_epi8(char(0xFC))), scale);
    }
  }
  else
  {
    const __m128i* srcY = reinterpret_cast<const __m128i*>(imageY[0]);
    const __m128i* srcUV = reinterpret_cast<const __m128i*>(imageUV[0]);
    __m128i* pDest = reinterpret_cast<__m128i*>(dest[0]);

    if(size_t overshoot = imageUV.width % 8)
    {
      overshoot = 8 - overshoot;
      for(size_t nY = imageUV.height; nY; --nY)
      {
        const __m128i* const baseSrcUV = srcUV;
        for(size_t i = 2; i; --i)
        {
          for(size_t nX = imageUV.width / 8 + 1; nX; --nX)
          {
            const std::function<void(const __m128i, const __m128i, const size_t)> expand = [&expand, &pDest](const __m128i y, const __m128i uv, const size_t scale)
            {
              if(scale > 1)
              {
                expand(_mm_unpacklo_epi8(y, y), _mm_unpacklo_epi16(uv, uv), scale >> 1);
                expand(_mm_unpackhi_epi8(y, y), _mm_unpackhi_epi16(uv, uv), scale >> 1);
              }
              else
              {
                _mm_storeu_si128(pDest, _mm_unpacklo_epi8(y, uv));
                _mm_storeu_si128(pDest + 1, _mm_unpackhi_epi8(y, uv));
                pDest += 2;
              }
            };
            expand(_mm_loadu_si128(srcY++), _mm_loadu_si128(srcUV++), scale);
          }

          srcY = reinterpret_cast<const __m128i*>(reinterpret_cast<const short*>(srcY) - overshoot);
          pDest = reinterpret_cast<__m128i*>(reinterpret_cast<unsigned int*>(pDest) - scale * overshoot);
          srcUV = baseSrcUV;
        }

        srcUV = reinterpret_cast<const __m128i*>(reinterpret_cast<const short*>(srcUV) + imageUV.width);
      }
    }
    else
    {
      for(size_t nY = imageUV.height; nY; --nY)
      {
        const __m128i* const baseSrcUV = srcUV;
        for(size_t i = 2; i; --i)
        {
          for(size_t nX = imageUV.width / 8; nX; --nX)
          {
            const std::function<void(const __m128i, const __m128i, const size_t)> expand = [&expand, &pDest](const __m128i y, const __m128i uv, const size_t scale)
            {
              if(scale > 1)
              {
                expand(_mm_unpacklo_epi8(y, y), _mm_unpacklo_epi16(uv, uv), scale >> 1);
                expand(_mm_unpackhi_epi8(y, y), _mm_unpackhi_epi16(uv, uv), scale >> 1);
              }
              else
              {
                _mm_storeu_si128(pDest, _mm_unpacklo_epi8(y, uv));
                _mm_storeu_si128(pDest + 1, _mm_unpackhi_epi8(y, uv));
                pDest += 2;
              }
            };
            expand(_mm_load_si128(srcY++), _mm_load_si128(srcUV++), scale);
          }

          srcUV = baseSrcUV;
        }

        srcUV += imageUV.width / 8;
      }
    }
  }

  // Expand vertically
  for(int y = imageY.height - 1; y >= 0; --y)
    for(unsigned int i = 0; i < scale; i++)
      memcpy(dest[0] + dest.width * y * scale + i * dest.width, dest[0] + dest.width * y, dest.width * 4);
}
