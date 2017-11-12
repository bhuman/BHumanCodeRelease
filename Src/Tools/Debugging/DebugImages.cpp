/**
 * @file Tools/Debugging/DebugImages.cpp
 *
 * @author Felix Thielke
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "DebugImages.h"
#include "Tools/ImageProcessing/AVX.h"
#include "Tools/ImageProcessing/ColorModelConversions.h"

template<bool avx> void rgbToBGRA(const __m_auto_i* const src, const __m_auto_i* const srcEnd, __m_auto_i* const dest)
{
  static const __m_auto_i shuffleMask = _mmauto_setr128_epi8(2, 1, 0, char(0xFF), 6, 5, 4, char(0xFF), 10, 9, 8, char(0xFF), 14, 13, 12, char(0xFF));
  static const __m_auto_i alpha = _mmauto_set1_epi32(0xFF000000);
  __m_auto_i* pDest = dest;
  for(const __m_auto_i* pSrc = src; pSrc < srcEnd; pSrc++)
  {
    _mmauto_storet_si_all<true>(pDest++,
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

template<bool avx> void yuyvToBGRA(const __m_auto_i* const src, const __m_auto_i* const srcEnd, __m_auto_i* const dest)
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
      _mmauto_storet_si_all<true>(pDest++, bgra0);
      _mmauto_storet_si_all<true>(pDest++, bgra1);
    }
    else
    {
      const __m_auto_i bg = _mmauto_unpacklo_epi8(b, g);
      const __m_auto_i ra = _mmauto_unpacklo_epi8(r, alpha);
      _mmauto_storet_si_all<true>(pDest++, _mmauto_unpacklo_epi16(bg, ra));
      _mmauto_storet_si_all<true>(pDest++, _mmauto_unpackhi_epi16(bg, ra));
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

template<bool avx> void yuvToBGRA(const __m_auto_i* const src, const __m_auto_i* const srcEnd, __m_auto_i* const dest)
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
      _mmauto_storet_si_all<true>(pDest++, bgra0);
      _mmauto_storet_si_all<true>(pDest++, bgra1);
    }
    else
    {
      const __m_auto_i bg = _mmauto_unpacklo_epi8(b, g);
      const __m_auto_i ra = _mmauto_unpacklo_epi8(r, alpha);
      _mmauto_storet_si_all<true>(pDest++, _mmauto_unpacklo_epi16(bg, ra));
      _mmauto_storet_si_all<true>(pDest++, _mmauto_unpackhi_epi16(bg, ra));
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

template<bool avx> void ALWAYSINLINE storeColors(__m_auto_i* dest, const __m_auto_i p)
{
  alignas(avx ? 32 : 16)static __m_auto_i colors[FieldColors::numOfColors] =
  {
    _mmauto_set1_epi32(0xff7f7f7f), //none
    _mmauto_set1_epi32(0xffffffff), //white
    _mmauto_set1_epi32(0xff000000), //black
    _mmauto_set1_epi32(0xff00ff00), //green
    _mmauto_set1_epi32(0xff0000ff), //own team color
    _mmauto_set1_epi32(0xffff0000)  //opponent team color
  };

  __m_auto_i result = _mmauto_setzero_si_all();

  FOREACH_ENUM((FieldColors) Color, i)
    result = _mmauto_or_si_all(result, _mmauto_and_si_all(_mmauto_cmpeq_epi32(p, _mmauto_set1_epi32(i)), colors[i]));

  _mmauto_storet_si_all<true>(dest, result);
}

template<bool avx> void coloredToBGRA(const __m_auto_i* const src, const __m_auto_i* const srcEnd, __m_auto_i* const dest)
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
    storeColors<avx>(pDest++, pLo2);
    storeColors<avx>(pDest++, pHi2);

    pLo2 = pHi;
    pHi2 = c_0;
    _mmauto_unpacklohi_epi8(pLo2, pHi2);
    storeColors<avx>(pDest++, pLo2);
    storeColors<avx>(pDest++, pHi2);
  }
}

void coloredToBGRA(const PixelTypes::ColoredPixel* const src, const PixelTypes::ColoredPixel* const srcEnd, PixelTypes::BGRAPixel* const dest)
{
  static unsigned int colors[FieldColors::numOfColors] =
  {
    0xff7f7f7f, //none
    0xffffffff, //white
    0xff000000, //black
    0xff00ff00, //green
    0xff0000ff, //own team color
    0xffff0000  //opponent team color
  };

  PixelTypes::BGRAPixel* pDest = dest;
  for(const PixelTypes::ColoredPixel* pSrc = src; pSrc < srcEnd; pSrc++, pDest++)
    pDest->color = colors[*pSrc];
}

template<bool avx> void grayscaledToBGRA(const __m_auto_i* const src, const __m_auto_i* const srcEnd, __m_auto_i* const dest)
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
    _mmauto_storet_si_all<true>(pDest++, _mmauto_or_si_all(pLo2, alpha));
    _mmauto_storet_si_all<true>(pDest++, _mmauto_or_si_all(pHi2, alpha));

    pLo2 = pHi;
    pHi2 = pHi;
    _mmauto_unpacklohi_epi8(pLo2, pHi2);
    _mmauto_storet_si_all<true>(pDest++, _mmauto_or_si_all(pLo2, alpha));
    _mmauto_storet_si_all<true>(pDest++, _mmauto_or_si_all(pHi2, alpha));
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

template<bool avx> void binaryToBGRA(const __m_auto_i* const src, const __m_auto_i* const srcEnd, __m_auto_i* const dest)
{
  static const __m_auto_i alpha = _mmauto_set1_epi32(0xFF000000);
  __m_auto_i* pDest = dest;
  for(const __m_auto_i* pSrc = src; pSrc < srcEnd; pSrc++)
  {
    const __m_auto_i p = _mmauto_cmpgt_epi8(_mmauto_loadt_si_all<true>(pSrc), _mmauto_setzero_si_all());

    __m_auto_i pLo = p;
    __m_auto_i pHi = p;
    _mmauto_unpacklohi_epi8(pLo, pHi);

    __m_auto_i pLo2 = pLo;
    __m_auto_i pHi2 = pLo;
    _mmauto_unpacklohi_epi8(pLo2, pHi2);
    _mmauto_storet_si_all<true>(pDest++, _mmauto_or_si_all(pLo2, alpha));
    _mmauto_storet_si_all<true>(pDest++, _mmauto_or_si_all(pHi2, alpha));

    pLo2 = pHi;
    pHi2 = pHi;
    _mmauto_unpacklohi_epi8(pLo2, pHi2);
    _mmauto_storet_si_all<true>(pDest++, _mmauto_or_si_all(pLo2, alpha));
    _mmauto_storet_si_all<true>(pDest++, _mmauto_or_si_all(pHi2, alpha));
  }
}

void binaryToBGRA(const PixelTypes::BinaryPixel* const src, const PixelTypes::BinaryPixel* const srcEnd, PixelTypes::BGRAPixel* const dest)
{
  PixelTypes::BGRAPixel* pDest = dest;
  for(const PixelTypes::BinaryPixel* pSrc = src; pSrc < srcEnd; pSrc++, pDest++)
  {
    pDest->r = pDest->g = pDest->b = *pSrc ? 0xFF : 0;
    pDest->a = 0xFF;
  }
}

void hueToBGRA(const PixelTypes::HuePixel* const src, const PixelTypes::HuePixel* const srcEnd, PixelTypes::BGRAPixel* const dest)
{
  PixelTypes::BGRAPixel* pDest = dest;
  for(const PixelTypes::HuePixel* pSrc = src; pSrc < srcEnd; pSrc++, pDest++)
  {
    ColorModelConversions::fromYUVToRGB(128, static_cast<unsigned char>(static_cast<float>(std::cos(static_cast<float>(*pSrc) * 360_deg / 256.f)) * 128.f + 128.f), static_cast<unsigned char>(static_cast<float>(std::sin(static_cast<float>(*pSrc) * 360_deg / 256.f)) * 128.f + 128.f), pDest->r, pDest->g, pDest->b);
    pDest->a = 0xFF;
  }
}

template<bool avx> void edge2ToBGRA(const __m_auto_i* const src, const __m_auto_i* const srcEnd, __m_auto_i* const dest, const __m_auto_i* const destEnd)
{
  static const __m_auto_i offset = _mmauto_set1_epi8(char(128));
  __m_auto_i* pDest = dest;

  for(const __m_auto_i* pSrc = src; pSrc < srcEnd && pDest < destEnd; pSrc++)
  {
    const __m_auto_i p = _mmauto_loadt_si_all<true>(pSrc);
    const __m_auto_i offsetCorrected = _mmauto_abs_epi8(_mmauto_sub_epi8(p, offset));
    __m_auto_i res0 = _mmauto_slli_epi16(_mmauto_sqrt_epu16<avx>(_mmauto_slli_epi16(_mmauto_maddubs_epi16(offsetCorrected, offsetCorrected), 1)), 8);
    __m_auto_i res1 = p;
    _mmauto_unpacklohi_epi8(res0, res1);
    _mmauto_storet_si_all<true>(pDest++, res0);
    _mmauto_storet_si_all<true>(pDest++, res1);
  }

  yuvToBGRA<avx>(dest, destEnd, dest);
}

void edge2ToBGRA(const PixelTypes::Edge2Pixel* const src, const PixelTypes::Edge2Pixel* const srcEnd, PixelTypes::BGRAPixel* const dest, const PixelTypes::BGRAPixel* const destEnd)
{
  //todo
}

template<bool avx> void typeToBGRA(const void* const src, void* const dest, const PixelTypes::PixelType type, const size_t size)
{
  const __m_auto_i* const pSrc = reinterpret_cast<const __m_auto_i*>(src);
  const __m_auto_i* const srcEnd = reinterpret_cast<const __m_auto_i*>(src) + (size * PixelTypes::pixelSize(type)) / sizeof(__m_auto_i);
  __m_auto_i* pDest = reinterpret_cast<__m_auto_i*>(dest);

  switch(type)
  {
    case PixelTypes::PixelType::RGB:
      rgbToBGRA<avx>(pSrc, srcEnd, pDest);
      break;
    case PixelTypes::PixelType::YUV:
      yuvToBGRA<avx>(pSrc, srcEnd, pDest);
      break;
    case PixelTypes::PixelType::YUYV:
      yuyvToBGRA<avx>(pSrc, srcEnd, pDest);
      break;
    case PixelTypes::PixelType::Colored:
      coloredToBGRA<avx>(pSrc, srcEnd, pDest);
      break;
    case PixelTypes::PixelType::Grayscale:
      grayscaledToBGRA<avx>(pSrc, srcEnd, pDest);
      break;
    case PixelTypes::PixelType::Hue:
      hueToBGRA(reinterpret_cast<const PixelTypes::HuePixel*>(src), reinterpret_cast<const PixelTypes::HuePixel*>(srcEnd), reinterpret_cast<PixelTypes::BGRAPixel*>(dest));
      break;
    case PixelTypes::PixelType::Edge2:
      edge2ToBGRA<avx>(pSrc, srcEnd, pDest, reinterpret_cast<const __m_auto_i*>(dest) + (size * PixelTypes::pixelSize(PixelTypes::BGRA)) / sizeof(__m_auto_i));
      break;/*
    case PixelTypes::PixelType::Edge2MonoAvg:
      //edge2MonoAvgToBGRA<avx>(pSrc, srcEnd, pDest);
      break;
    case PixelTypes::PixelType::Edge2MonoAbsAvg:
      //edge2MonoAbsAvgBGRA<avx>(pSrc, srcEnd, pDest);
      break;*/
    case PixelTypes::PixelType::Binary:
      binaryToBGRA<avx>(pSrc, srcEnd, pDest);
      break;
    default:
      FAIL("Unknown pixel type.");
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
      case PixelTypes::PixelType::Hue:
        hueToBGRA(reinterpret_cast<const PixelTypes::HuePixel*>(src) + size - rest, reinterpret_cast<const PixelTypes::HuePixel*>(src) + size, ppDest);
        break;
      case PixelTypes::PixelType::Edge2:
        edge2ToBGRA(reinterpret_cast<const PixelTypes::Edge2Pixel*>(src) + size - rest, reinterpret_cast<const PixelTypes::Edge2Pixel*>(src) + size, ppDest, reinterpret_cast<PixelTypes::BGRAPixel*>(dest) + size);
        break;
      case PixelTypes::PixelType::Binary:
        binaryToBGRA(reinterpret_cast<const PixelTypes::BinaryPixel*>(src) + size - rest, reinterpret_cast<const PixelTypes::BinaryPixel*>(src) + size, ppDest);
        break;
      default:
        FAIL("Unknown pixel type.");
    }
  }
}

void DebugImage::convertToBGRA(void* dest) const
{
  if(type == PixelTypes::BGRA)
    memcpy(dest, data, width * height * PixelTypes::pixelSize(PixelTypes::BGRA));
  else
  {
    ASSERT(simdAligned<_supportsAVX2>(dest));
    typeToBGRA<_supportsAVX2>(data, dest, type, width * height);
  }
}
