/**
 * @file Tools/Debugging/DebugImages.cpp
 *
 * @author Felix Thielke
 */

#include "DebugImages.h"
#include "Tools/ImageProcessing/AVX.h"
#include "Tools/ImageProcessing/ColorModelConversions.h"

template<bool avx, bool aligned> void rgbToBGRA(const __m_auto_i* const src, const __m_auto_i* const srcEnd, __m_auto_i* const dest)
{
  static const __m_auto_i shuffleMask = _mmauto_setr128_epi8(2, 1, 0, char(0xFF), 6, 5, 4, char(0xFF), 10, 9, 8, char(0xFF), 14, 13, 12, char(0xFF));
  static const __m_auto_i alpha = _mmauto_set1_epi32(0xFF000000);
  __m_auto_i* pDest = dest;
  for(const __m_auto_i* pSrc = src; pSrc < srcEnd; pSrc++)
  {
    _mmauto_storet_si_all<aligned>(pDest++,
                                   _mmauto_or_si_all(
                                     _mmauto_shuffle_epi8(
                                       _mmauto_loadt_si_all<true>(pSrc),
                                       shuffleMask
                                     ),
                                     alpha)
                                  );
  }
}

void rgbToBGRA(const PixelTypes::RGBPixel* const src, const PixelTypes::RGBPixel* const srcEnd, PixelTypes::BGRAPixel* const dest)
{
  PixelTypes::BGRAPixel* pDest = dest;
  for(const PixelTypes::RGBPixel* pSrc = src; pSrc < srcEnd; pSrc++, pDest++)
  {
    pDest->b = pSrc->b;
    pDest->g = pSrc->g;
    pDest->r = pSrc->r;
    pDest->a = 0xFF;
  }
}

template<bool avx, bool aligned> void yuyvToBGRA(const __m_auto_i* const src, const __m_auto_i* const srcEnd, __m_auto_i* const dest)
{
  static const __m_auto_i channelMask = _mmauto_set1_epi16(0x00FF);
  static const __m_auto_i c_128 = _mmauto_set1_epi16(128);
  static const __m_auto_i c_0 = _mmauto_setzero_si_all();
  static const __m_auto_i alpha = _mmauto_set1_epi8(static_cast<unsigned char>(0xFF));
  static const __m_auto_i scaledInvUCoeff = _mmauto_set1_epi16(static_cast<short>(static_cast<float>(1 << 14) / ColorModelConversions::uCoeff));
  static const __m_auto_i scaledInvVCoeff = _mmauto_set1_epi16(static_cast<short>(static_cast<float>(1 << 14) / ColorModelConversions::vCoeff));
  static const __m_auto_i scaledGCoeffU = _mmauto_set1_epi16(static_cast<short>(ColorModelConversions::yCoeffB / (ColorModelConversions::yCoeffG * ColorModelConversions::uCoeff) * static_cast<float>(1 << 15)));
  static const __m_auto_i scaledGCoeffV = _mmauto_set1_epi16(static_cast<short>(ColorModelConversions::yCoeffR / (ColorModelConversions::yCoeffG * ColorModelConversions::vCoeff) * static_cast<float>(1 << 15)));
  __m_auto_i* pDest = dest;
  for(const __m_auto_i* pSrc = src; pSrc < srcEnd; pSrc++)
  {
    const __m_auto_i p = _mmauto_loadt_si_all<true>(pSrc);

    const __m_auto_i y = _mmauto_and_si_all(p, channelMask);
    const __m_auto_i uv = _mmauto_sub_epi16(_mmauto_and_si_all(_mmauto_srli_si_all(p, 1), channelMask), c_128);
    __m_auto_i u = _mmauto_packs_epi32(_mmauto_srai_epi32(_mmauto_slli_epi32(uv, 16), 16), c_0);
    u = _mmauto_unpacklo_epi16(u, u);
    __m_auto_i v = _mmauto_packs_epi32(_mmauto_srai_epi32(uv, 16), c_0);
    v = _mmauto_unpacklo_epi16(v, v);

    const __m_auto_i b = _mmauto_correct_256op(_mmauto_packus_epi16(_mmauto_add_epi16(y, _mmauto_slli_epi16(_mmauto_mulhrs_epi16(u, scaledInvUCoeff), 1)), c_0));
    const __m_auto_i g = _mmauto_correct_256op(_mmauto_packus_epi16(_mmauto_sub_epi16(y, _mmauto_add_epi16(_mmauto_mulhrs_epi16(u, scaledGCoeffU), _mmauto_mulhrs_epi16(v, scaledGCoeffV))), c_0));
    const __m_auto_i r = _mmauto_correct_256op(_mmauto_packus_epi16(_mmauto_add_epi16(y, _mmauto_slli_epi32(_mmauto_mulhrs_epi16(v, scaledInvVCoeff), 1)), c_0));

    if(avx)
    {
      __m_auto_i bgra0 = b;
      __m_auto_i tmp = g;
      _mmauto_unpacklohi_epi8(bgra0, tmp);
      __m_auto_i bgra1 = r;
      tmp = alpha;
      _mmauto_unpacklohi_epi8(bgra1, tmp);
      _mmauto_unpacklohi_epi16(bgra0, bgra1);
      _mmauto_storet_si_all<aligned>(pDest++, bgra0);
      _mmauto_storet_si_all<aligned>(pDest++, bgra1);
    }
    else
    {
      const __m_auto_i bg = _mmauto_unpacklo_epi8(b, g);
      const __m_auto_i ra = _mmauto_unpacklo_epi8(r, alpha);
      _mmauto_storet_si_all<aligned>(pDest++, _mmauto_unpacklo_epi16(bg, ra));
      _mmauto_storet_si_all<aligned>(pDest++, _mmauto_unpackhi_epi16(bg, ra));
    }
  }
}

void yuyvToBGRA(const PixelTypes::YUYVPixel* const src, const PixelTypes::YUYVPixel* const srcEnd, PixelTypes::BGRAPixel* const dest)
{
  PixelTypes::BGRAPixel* pDest = dest;
  for(const PixelTypes::YUYVPixel* pSrc = src; pSrc < srcEnd; pSrc++, pDest++)
  {
    ColorModelConversions::fromYUVToRGB(pSrc->y0, pSrc->u, pSrc->v, pDest->r, pDest->g, pDest->b);
    pDest->a = 0xFF;
    pDest++;
    ColorModelConversions::fromYUVToRGB(pSrc->y1, pSrc->u, pSrc->v, pDest->r, pDest->g, pDest->b);
    pDest->a = 0xFF;
  }
}

template<bool avx, bool aligned> void yuvToBGRA(const __m_auto_i* const src, const __m_auto_i* const srcEnd, __m_auto_i* const dest)
{
  static const __m_auto_i yMask = _mmauto_set1_epi32(0x000000FF);
  static const __m_auto_i channelMask = _mmauto_set1_epi16(0x00FF);
  static const __m_auto_i c_128 = _mmauto_set1_epi16(128);
  static const __m_auto_i c_0 = _mmauto_setzero_si_all();
  static const __m_auto_i alpha = _mmauto_set1_epi8(static_cast<unsigned char>(0xFF));
  static const __m_auto_i scaledInvUCoeff = _mmauto_set1_epi16(static_cast<short>(static_cast<float>(1 << 14) / ColorModelConversions::uCoeff));
  static const __m_auto_i scaledInvVCoeff = _mmauto_set1_epi16(static_cast<short>(static_cast<float>(1 << 14) / ColorModelConversions::vCoeff));
  static const __m_auto_i scaledGCoeffU = _mmauto_set1_epi16(static_cast<short>(ColorModelConversions::yCoeffB / (ColorModelConversions::yCoeffG * ColorModelConversions::uCoeff) * static_cast<float>(1 << 15)));
  static const __m_auto_i scaledGCoeffV = _mmauto_set1_epi16(static_cast<short>(ColorModelConversions::yCoeffR / (ColorModelConversions::yCoeffG * ColorModelConversions::vCoeff) * static_cast<float>(1 << 15)));
  __m_auto_i* pDest = dest;
  for(const __m_auto_i* pSrc = src; pSrc < srcEnd;)
  {
    const __m_auto_i p0 = _mmauto_loadt_si_all<true>(pSrc++);
    const __m_auto_i p1 = _mmauto_loadt_si_all<true>(pSrc++);

    const __m_auto_i y = _mmauto_correct_256op(_mmauto_packs_epi32(_mmauto_and_si_all(_mmauto_srli_epi32(p0, 16), yMask), _mmauto_and_si_all(_mmauto_srli_epi32(p1, 16), yMask)));

    const __m_auto_i uv0 = _mmauto_sub_epi16(_mmauto_and_si_all(_mmauto_srli_si_all(p0, 1), channelMask), c_128);
    const __m_auto_i uv1 = _mmauto_sub_epi16(_mmauto_and_si_all(_mmauto_srli_si_all(p1, 1), channelMask), c_128);
    const __m_auto_i u = _mmauto_correct_256op(_mmauto_packs_epi32(_mmauto_srai_epi32(_mmauto_slli_epi32(uv0, 16), 16), _mmauto_srai_epi32(_mmauto_slli_epi32(uv1, 16), 16)));
    const __m_auto_i v = _mmauto_correct_256op(_mmauto_packs_epi32(_mmauto_srai_epi32(uv0, 16), _mmauto_srai_epi32(uv1, 16)));

    const __m_auto_i b = _mmauto_correct_256op(_mmauto_packus_epi16(_mmauto_add_epi16(y, _mmauto_slli_epi16(_mmauto_mulhrs_epi16(u, scaledInvUCoeff), 1)), c_0));
    const __m_auto_i g = _mmauto_correct_256op(_mmauto_packus_epi16(_mmauto_sub_epi16(y, _mmauto_add_epi16(_mmauto_mulhrs_epi16(u, scaledGCoeffU), _mmauto_mulhrs_epi16(v, scaledGCoeffV))), c_0));
    const __m_auto_i r = _mmauto_correct_256op(_mmauto_packus_epi16(_mmauto_add_epi16(y, _mmauto_slli_epi32(_mmauto_mulhrs_epi16(v, scaledInvVCoeff), 1)), c_0));

    if(avx)
    {
      __m_auto_i bgra0 = b;
      __m_auto_i tmp = g;
      _mmauto_unpacklohi_epi8(bgra0, tmp);
      __m_auto_i bgra1 = r;
      tmp = alpha;
      _mmauto_unpacklohi_epi8(bgra1, tmp);
      _mmauto_unpacklohi_epi16(bgra0, bgra1);
      _mmauto_storet_si_all<aligned>(pDest++, bgra0);
      _mmauto_storet_si_all<aligned>(pDest++, bgra1);
    }
    else
    {
      const __m_auto_i bg = _mmauto_unpacklo_epi8(b, g);
      const __m_auto_i ra = _mmauto_unpacklo_epi8(r, alpha);
      _mmauto_storet_si_all<aligned>(pDest++, _mmauto_unpacklo_epi16(bg, ra));
      _mmauto_storet_si_all<aligned>(pDest++, _mmauto_unpackhi_epi16(bg, ra));
    }
  }
}

void yuvToBGRA(const PixelTypes::YUVPixel* const src, const PixelTypes::YUVPixel* const srcEnd, PixelTypes::BGRAPixel* const dest)
{
  PixelTypes::BGRAPixel* pDest = dest;
  for(const PixelTypes::YUVPixel* pSrc = src; pSrc < srcEnd; pSrc++, pDest++)
  {
    ColorModelConversions::fromYUVToRGB(pSrc->y, pSrc->u, pSrc->v, pDest->r, pDest->g, pDest->b);
    pDest->a = 0xFF;
  }
}

template<bool avx, bool aligned> void ALWAYSINLINE storeColors(__m_auto_i* dest, const __m_auto_i p)
{
  static const __m_auto_i classNone = _mmauto_set1_epi32(FieldColors::Color::none);
  static const __m_auto_i classWhite = _mmauto_set1_epi32(FieldColors::Color::white);
  static const __m_auto_i classBlack = _mmauto_set1_epi32(FieldColors::Color::black);
  static const __m_auto_i classGreen = _mmauto_set1_epi32(FieldColors::Color::green);
  static const __m_auto_i colorNone = _mmauto_set1_epi32(0xFF7F7F7F);
  static const __m_auto_i colorWhite = _mmauto_set1_epi32(0xFFFFFFFF);
  static const __m_auto_i colorBlack = _mmauto_set1_epi32(0xFF000000);
  static const __m_auto_i colorGreen = _mmauto_set1_epi32(0xFF00FF00);
  _mmauto_storet_si_all<aligned>(dest,
                                 _mmauto_or_si_all(
                                   _mmauto_or_si_all(
                                     _mmauto_or_si_all(
                                       _mmauto_and_si_all(_mmauto_cmpeq_epi32(p, classNone), colorNone),
                                       _mmauto_and_si_all(_mmauto_cmpeq_epi32(p, classWhite), colorWhite)
                                     ),
                                     _mmauto_and_si_all(_mmauto_cmpeq_epi32(p, classBlack), colorBlack)
                                   ),
                                   _mmauto_and_si_all(_mmauto_cmpeq_epi32(p, classGreen), colorGreen)
                                 )
                                );
}

template<bool avx, bool aligned> void coloredToBGRA(const __m_auto_i* const src, const __m_auto_i* const srcEnd, __m_auto_i* const dest)
{
  static const __m_auto_i c_0 = _mmauto_setzero_si_all();
  __m_auto_i* pDest = dest;
  for(const __m_auto_i* pSrc = src; pSrc < srcEnd; pSrc++)
  {
    const __m_auto_i p = _mmauto_loadt_si_all<true>(pSrc);

    __m_auto_i pLo = p;
    __m_auto_i pHi = c_0;
    _mmauto_unpacklohi_epi8(pLo, pHi);

    __m_auto_i pLo2 = pLo;
    __m_auto_i pHi2 = c_0;
    _mmauto_unpacklohi_epi8(pLo2, pHi2);
    storeColors<avx, aligned>(pDest++, pLo2);
    storeColors<avx, aligned>(pDest++, pHi2);

    pLo2 = pHi;
    pHi2 = c_0;
    _mmauto_unpacklohi_epi8(pLo2, pHi2);
    storeColors<avx, aligned>(pDest++, pLo2);
    storeColors<avx, aligned>(pDest++, pHi2);
  }
}

void coloredToBGRA(const PixelTypes::ColoredPixel* const src, const PixelTypes::ColoredPixel* const srcEnd, PixelTypes::BGRAPixel* const dest)
{
  static unsigned int colors[4] =
  {
    0xFF7F7F7F,
    0xFFFFFFFF,
    0xFF000000,
    0xFF00FF00
  };

  PixelTypes::BGRAPixel* pDest = dest;
  for(const PixelTypes::ColoredPixel* pSrc = src; pSrc < srcEnd; pSrc++, pDest++)
    pDest->color = colors[*pSrc];
}

template<bool avx, bool aligned> void grayscaledToBGRA(const __m_auto_i* const src, const __m_auto_i* const srcEnd, __m_auto_i* const dest)
{
  static const __m_auto_i alpha = _mmauto_set1_epi32(0xFF000000);
  __m_auto_i* pDest = dest;
  for(const __m_auto_i* pSrc = src; pSrc < srcEnd; pSrc++)
  {
    const __m_auto_i p = _mmauto_loadt_si_all<true>(pSrc);

    __m_auto_i pLo = p;
    __m_auto_i pHi = p;
    _mmauto_unpacklohi_epi8(pLo, pHi);

    __m_auto_i pLo2 = pLo;
    __m_auto_i pHi2 = pLo;
    _mmauto_unpacklohi_epi8(pLo2, pHi2);
    _mmauto_storet_si_all<aligned>(pDest++, _mmauto_or_si_all(pLo2, alpha));
    _mmauto_storet_si_all<aligned>(pDest++, _mmauto_or_si_all(pHi2, alpha));

    pLo2 = pHi;
    pHi2 = pHi;
    _mmauto_unpacklohi_epi8(pLo2, pHi2);
    _mmauto_storet_si_all<aligned>(pDest++, _mmauto_or_si_all(pLo2, alpha));
    _mmauto_storet_si_all<aligned>(pDest++, _mmauto_or_si_all(pHi2, alpha));
  }
}

void grayscaledToBGRA(const PixelTypes::GrayscaledPixel* const src, const PixelTypes::GrayscaledPixel* const srcEnd, PixelTypes::BGRAPixel* const dest)
{
  PixelTypes::BGRAPixel* pDest = dest;
  for(const PixelTypes::GrayscaledPixel* pSrc = src; pSrc < srcEnd; pSrc++, pDest++)
  {
    pDest->r = pDest->g = pDest->b = *pSrc;
    pDest->a = 0xFF;
  }
}

template<bool avx, bool aligned> void typeToBGRA(const void* const src, void* const dest, const PixelTypes::PixelType type, const size_t size)
{
  const __m_auto_i* const pSrc = reinterpret_cast<const __m_auto_i*>(src);
  const __m_auto_i* const srcEnd = reinterpret_cast<const __m_auto_i*>(src) + (size * PixelTypes::pixelSize(type)) / sizeof(__m_auto_i);
  __m_auto_i* pDest = reinterpret_cast<__m_auto_i*>(dest);

  switch(type)
  {
    case PixelTypes::PixelType::RGB:
      rgbToBGRA<avx, aligned>(pSrc, srcEnd, pDest);
      break;
    case PixelTypes::PixelType::YUV:
      yuvToBGRA<avx, aligned>(pSrc, srcEnd, pDest);
      break;
    case PixelTypes::PixelType::YUYV:
      yuyvToBGRA<avx, aligned>(pSrc, srcEnd, pDest);
      break;
    case PixelTypes::PixelType::Colored:
      coloredToBGRA<avx, aligned>(pSrc, srcEnd, pDest);
      break;
    case PixelTypes::PixelType::Grayscale:
      grayscaledToBGRA<avx, aligned>(pSrc, srcEnd, pDest);
      break;
    default:
      ASSERT(false);
  }

  size_t rest = (size * PixelTypes::pixelSize(type)) % sizeof(__m_auto_i);
  if(rest != 0)
  {
    PixelTypes::BGRAPixel* const ppDest = reinterpret_cast<PixelTypes::BGRAPixel*>(dest) + size - rest;
    switch(type)
    {
      case PixelTypes::PixelType::RGB:
        rgbToBGRA(reinterpret_cast<const PixelTypes::RGBPixel*>(src) + size - rest, reinterpret_cast<const PixelTypes::RGBPixel*>(src) + size, ppDest);
        break;
      case PixelTypes::PixelType::YUV:
        yuvToBGRA(reinterpret_cast<const PixelTypes::YUVPixel*>(src) + size - rest, reinterpret_cast<const PixelTypes::YUVPixel*>(src) + size, ppDest);
        break;
      case PixelTypes::PixelType::YUYV:
        yuyvToBGRA(reinterpret_cast<const PixelTypes::YUYVPixel*>(src) + size - rest, reinterpret_cast<const PixelTypes::YUYVPixel*>(src) + size, ppDest);
        break;
      case PixelTypes::PixelType::Colored:
        coloredToBGRA(reinterpret_cast<const PixelTypes::ColoredPixel*>(src) + size - rest, reinterpret_cast<const PixelTypes::ColoredPixel*>(src) + size, ppDest);
        break;
      case PixelTypes::PixelType::Grayscale:
        grayscaledToBGRA(reinterpret_cast<const PixelTypes::GrayscaledPixel*>(src) + size - rest, reinterpret_cast<const PixelTypes::GrayscaledPixel*>(src) + size, ppDest);
        break;
      default:
        ASSERT(false);
    }
  }
}

void DebugImage::convertToBGRA(void* dest) const
{
  if(type == PixelTypes::BGRA)
    memcpy(dest, data, width * height * PixelTypes::pixelSize(PixelTypes::BGRA));
  else
  {
    if(_supportsAVX2)
    {
      if(simdAligned<true>(dest))
        typeToBGRA<true, true>(data, dest, type, width * height);
      else
        typeToBGRA<true, false>(data, dest, type, width * height);
    }
    else
    {
      if(simdAligned<false>(dest))
        typeToBGRA<false, true>(data, dest, type, width * height);
      else
        typeToBGRA<false, false>(data, dest, type, width * height);
    }
  }
}
