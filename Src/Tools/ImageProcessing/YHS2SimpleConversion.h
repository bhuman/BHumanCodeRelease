/**
 * @file YHS2SimpleConversion.h
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * @author Felix Thielke
 */

#pragma once

#include "Representations/Configuration/FieldColors.h"

#include "AVX.h"
#include "PixelTypes.h"
#include "YHSColorConversion.h"

namespace YHS2s
{
#define PREFETCH

  using namespace PixelTypes;

  template<bool avx> ALWAYSINLINE __m_auto_i classifyColorSimple(
    const __m_auto_i isColored,
    const __m_auto_i isYWhite,
    const __m_auto_i isHueField)
  {
    static const __m_auto_i classWhite = _mmauto_set1_epi8(FieldColors::Color::white);
    static const __m_auto_i classField = _mmauto_set1_epi8(FieldColors::Color::field);
    static const __m_auto_i classBlack = _mmauto_set1_epi8(FieldColors::Color::black);
    static_assert(static_cast<int>(FieldColors::Color::none) == 0, "");

    __m_auto_i classification =
      _mmauto_or_si_all(
        _mmauto_and_si_all(
          isColored,
          _mmauto_and_si_all(isHueField, classField)
        ),
        _mmauto_andnot_si_all(
          isColored,
          _mmauto_or_si_all(
            _mmauto_and_si_all(isYWhite, classWhite),
            _mmauto_andnot_si_all(isYWhite, classBlack)
          )
        )
      );

    return classification;
  }

  template<bool aligned, bool avx, bool saveGrayscaled, bool saveSaturation, bool saveHue>
  void classifyByYHS2FieldColorSSE(const YUYVPixel* const srcImage, const int srcWidth, const int srcHeight, const FieldColors& theFieldColors,
                                   Image<PixelTypes::GrayscaledPixel>& grayscaled, Image<ColoredPixel>& colored, Image<HuePixel>& hued,
                                   Image<PixelTypes::GrayscaledPixel>& saturated)
  {
    ASSERT(srcWidth % 32 == 0);

    __m_auto_i* grayscaledDest = reinterpret_cast<__m_auto_i*>(grayscaled[0]) - 1;
    __m_auto_i* coloredDest = reinterpret_cast<__m_auto_i*>(colored[0]) - 1;
    __m_auto_i* saturatedDest = reinterpret_cast<__m_auto_i*>(saturated[0]) - 1;
    __m_auto_i* huedDest = reinterpret_cast<__m_auto_i*>(hued[0]) - 1;
    const __m_auto_i* const imageEnd = reinterpret_cast<const __m_auto_i*>(srcImage + srcWidth * srcHeight) - 1;

    const __m_auto_i fieldHFrom = _mmauto_set1_epi8(theFieldColors.fieldHue.min);
    const __m_auto_i fieldHTo = _mmauto_set1_epi8(theFieldColors.fieldHue.max);

    const __m_auto_i maxNonColorSaturation = _mmauto_set1_epi8(theFieldColors.maxNonColorSaturation);
    const __m_auto_i blackWhiteDelimiter = _mmauto_set1_epi8(theFieldColors.blackWhiteDelimiter);

    static const __m_auto_i c_128 = _mmauto_set1_epi8(char(128));
    static const __m_auto_i channelMask = _mmauto_set1_epi16(0x00FF);

#ifdef PREFETCH
    const char* prefetchSrc = reinterpret_cast<const char*>(srcImage) + (avx ? 128 : 64);
    const char* prefetchGrayscaledDest = reinterpret_cast<const char*>(grayscaled[0]) + (avx ? 64 : 32);
    const char* prefetchColoredDest = reinterpret_cast<const char*>(colored[0]) - (avx ? 64 : 32);
#endif // PREFETCH

    const __m_auto_i* src = reinterpret_cast<__m_auto_i const*>(srcImage) - 1;
    while(src < imageEnd)
    {
      const __m_auto_i p0 = _mmauto_loadt_si_all<aligned>(++src);
      const __m_auto_i p1 = _mmauto_loadt_si_all<aligned>(++src);
      const __m_auto_i p2 = _mmauto_loadt_si_all<aligned>(++src);
      const __m_auto_i p3 = _mmauto_loadt_si_all<aligned>(++src);

      // Compute luminance
      const __m_auto_i y0 = _mmauto_correct_256op(_mmauto_packus_epi16(_mmauto_and_si_all(p0, channelMask), _mmauto_and_si_all(p1, channelMask)));
      const __m_auto_i y1 = _mmauto_correct_256op(_mmauto_packus_epi16(_mmauto_and_si_all(p2, channelMask), _mmauto_and_si_all(p3, channelMask)));
      if(saveGrayscaled)
      {
        _mmauto_storet_si_all<true>(++grayscaledDest, y0);
        _mmauto_storet_si_all<true>(++grayscaledDest, y1);
      }

#ifdef PREFETCH
      _mm_prefetch(prefetchColoredDest += 32, _MM_HINT_T0);
      if(avx) _mm_prefetch(prefetchColoredDest += 32, _MM_HINT_T0);

      _mm_prefetch(prefetchSrc += 32, _MM_HINT_T0);
      _mm_prefetch(prefetchSrc += 32, _MM_HINT_T0);
      if(avx) _mm_prefetch(prefetchSrc += 32, _MM_HINT_T0);
      if(avx) _mm_prefetch(prefetchSrc += 32, _MM_HINT_T0);

      _mm_prefetch(prefetchGrayscaledDest += 32, _MM_HINT_T0);
      if(avx) _mm_prefetch(prefetchGrayscaledDest += 32, _MM_HINT_T0);
#endif // PREFETCH

      // Compute saturation
      const __m_auto_i uv0 = _mmauto_sub_epi8(_mmauto_correct_256op(_mmauto_packus_epi16(_mmauto_and_si_all(_mmauto_srli_si_all(p0, 1), channelMask), _mmauto_and_si_all(_mmauto_srli_si_all(p1, 1), channelMask))), c_128);
      const __m_auto_i uv1 = _mmauto_sub_epi8(_mmauto_correct_256op(_mmauto_packus_epi16(_mmauto_and_si_all(_mmauto_srli_si_all(p2, 1), channelMask), _mmauto_and_si_all(_mmauto_srli_si_all(p3, 1), channelMask))), c_128);

      /*
       // This is more exact, but sadly slower because of the two divisions
       __m_auto_i sat0 = YHSColorConversion::computeSaturation<avx>(uv0, uv1);
       __m_auto_i sat1 = sat0;
       _mmauto_unpacklohi_epi8(sat0, sat1);
       sat0 = _mmauto_divq8_epu8<avx>(sat0, y0);
       sat1 = _mmauto_divq8_epu8<avx>(sat1, y1);
       */
      const __m_auto_i sat0 = YHSColorConversion::computeLightingIndependentSaturation<avx>(y0, uv0);
      const __m_auto_i sat1 = YHSColorConversion::computeLightingIndependentSaturation<avx>(y1, uv1);
      if(saveSaturation)
      {
        _mmauto_streamt_si_all<true>(++saturatedDest, sat0);
        _mmauto_streamt_si_all<true>(++saturatedDest, sat1);
      }

      // Compute hue
      const __m_auto_i hue = YHSColorConversion::computeHue<avx>(uv0, uv1);
      __m_auto_i hue0 = hue;
      __m_auto_i hue1 = hue;
      _mmauto_unpacklohi_epi8(hue0, hue1);

      if(saveHue)
      {
        _mmauto_streamt_si_all<true>(++huedDest, hue0);
        _mmauto_streamt_si_all<true>(++huedDest, hue1);
      }

      // classify
      const __m_auto_i isHueField = _mmauto_cmpeq_epi8(_mmauto_subs_epu8(fieldHFrom, hue), _mmauto_subs_epu8(hue, fieldHTo));
      __m_auto_i isHueField0 = isHueField;
      __m_auto_i isHueField1 = isHueField;
      _mmauto_unpacklohi_epi8(isHueField0, isHueField1);

      const __m_auto_i isColored0 = _mmauto_cmpeq_epi8(_mmauto_subs_epu8(maxNonColorSaturation, sat0), _mmauto_setzero_si_all());
      const __m_auto_i isYWhite0 = _mmauto_cmpeq_epi8(_mmauto_subs_epu8(blackWhiteDelimiter, y0), _mmauto_setzero_si_all());

      const __m_auto_i isColored1 = _mmauto_cmpeq_epi8(_mmauto_subs_epu8(maxNonColorSaturation, sat1), _mmauto_setzero_si_all());
      const __m_auto_i isYWhite1 = _mmauto_cmpeq_epi8(_mmauto_subs_epu8(blackWhiteDelimiter, y1), _mmauto_setzero_si_all());

      _mmauto_storet_si_all<true>(++coloredDest, classifyColorSimple<avx>(isColored0, isYWhite0, isHueField0));
      _mmauto_storet_si_all<true>(++coloredDest, classifyColorSimple<avx>(isColored1, isYWhite1, isHueField1));
    }
  }

  template<bool saveGrayscaled, bool saveSaturation,  bool saveHue>
  void updateSSE(const YUYVPixel* const src, const int srcWidth, const int srcHeight, const FieldColors& theFieldColors,
                 Image<PixelTypes::GrayscaledPixel>& grayscaled, Image<ColoredPixel>& colored, Image<HuePixel>& hued, Image<PixelTypes::GrayscaledPixel>& saturated)
  {
    if(simdAligned<_supportsAVX2>(src))
      classifyByYHS2FieldColorSSE<true, _supportsAVX2, saveGrayscaled, saveSaturation, saveHue>(src, srcWidth, srcHeight, theFieldColors, grayscaled, colored, hued, saturated);
    else
      classifyByYHS2FieldColorSSE<false, _supportsAVX2, saveGrayscaled, saveSaturation, saveHue>(src, srcWidth, srcHeight, theFieldColors, grayscaled, colored, hued, saturated);
  }
}
