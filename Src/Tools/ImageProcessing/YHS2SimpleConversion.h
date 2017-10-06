/**
 * @file YHS2SimpleConversion.h
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * @author Felix Thielke
 */

#pragma once

#include "Representations/Configuration/FieldColors.h"
#include "Representations/Infrastructure/Image.h"

#include "AVX.h"
#include "PixelTypes.h"
#include "YHSColorConversion.h"

namespace YHS2s
{
#define PREFETCH

  using namespace PixelTypes;

  template<bool avx, bool classifyAllColors> ALWAYSINLINE __m_auto_i classifyColorSimple(
    const __m_auto_i isColored,
    const __m_auto_i isYWhite,
    const __m_auto_i isHueField,
    const __m_auto_i isOtherColors[FieldColors::numOfColors - FieldColors::numOfNonColors])
  {
    static const __m_auto_i classWhite = _mmauto_set1_epi8(FieldColors::Color::white);
    static const __m_auto_i classField = _mmauto_set1_epi8(FieldColors::Color::field);
    static const __m_auto_i classBlack = _mmauto_set1_epi8(FieldColors::Color::black);
    static_assert((int)FieldColors::Color::none == 0, "");

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

    if(classifyAllColors)
      for(unsigned char i = 0; i < FieldColors::numOfColors - FieldColors::numOfNonColors; ++i)
        classification =
          _mmauto_or_si_all(
            classification,
            _mmauto_and_si_all(
              _mmauto_cmpeq_epi8(classification, _mmauto_setzero_si_all()),
              _mmauto_and_si_all(
                isColored,
                _mmauto_and_si_all(isOtherColors[i], _mmauto_set1_epi8(FieldColors::numOfNonColors + i))
              )
            )
          );

    return classification;
  }

  template<bool aligned, bool avx, bool classifyAllColors, bool saveGrayscaled, bool saveColorchanneled, bool saveSaturation, bool saveHue>
  void classifyByYHS2FieldColorSSE(const YUYVPixel* const srcImage, const int srcWidth, const int srcHeight, const FieldColors& theFieldColors,
                                   TImage<PixelTypes::GrayscaledPixel>& grayscaled, TImage<PixelTypes::GrayscaledPixel>& ued, TImage<PixelTypes::GrayscaledPixel>& ved,
                                   TImage<ColoredPixel>& colored, TImage<HuePixel>& hued, TImage<PixelTypes::GrayscaledPixel>& saturated)
  {
    ASSERT(srcWidth % 32 == 0);

    __m_auto_i* grayscaledDest = reinterpret_cast<__m_auto_i*>(grayscaled[0]) - 1;
    __m_auto_i* uedDest0 = reinterpret_cast<__m_auto_i*>(ued[0]) - 1;
    __m_auto_i* vedDest1 = reinterpret_cast<__m_auto_i*>(ved[0]) - 1;
    __m_auto_i* coloredDest = reinterpret_cast<__m_auto_i*>(colored[0]) - 1;
    __m_auto_i* saturatedDest = reinterpret_cast<__m_auto_i*>(saturated[0]) - 1;
    __m_auto_i* huedDest = reinterpret_cast<__m_auto_i*>(hued[0]) - 1;
    const __m_auto_i* const imageEnd = reinterpret_cast<const __m_auto_i*>(srcImage + srcWidth * srcHeight) - 1;

    const __m_auto_i fieldHFrom = _mmauto_set1_epi8(theFieldColors.fieldHue.min);
    const __m_auto_i fieldHTo = _mmauto_set1_epi8(theFieldColors.fieldHue.max);
    alignas(avx ? 32 : 16)__m_auto_i isOtherColorsHMin[FieldColors::numOfColors - FieldColors::numOfNonColors];
    alignas(avx ? 32 : 16)__m_auto_i isOtherColorsHMax[FieldColors::numOfColors - FieldColors::numOfNonColors];
    if(classifyAllColors)
      for(int i = 0; i < FieldColors::numOfColors - FieldColors::numOfNonColors; ++i)
      {
        isOtherColorsHMin[i] = _mmauto_set1_epi8(theFieldColors.colorHues[i].min);
        isOtherColorsHMax[i] = _mmauto_set1_epi8(theFieldColors.colorHues[i].max);
      }

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

      if(saveColorchanneled)
      {
        _mmauto_storet_si_all<true>(++uedDest0, _mmauto_correct_256op(_mmauto_packus_epi16(_mmauto_and_si_all(uv0, channelMask), _mmauto_and_si_all(uv1, channelMask))));
        _mmauto_storet_si_all<true>(++vedDest1, _mmauto_correct_256op(_mmauto_packus_epi16(_mmauto_and_si_all(_mmauto_srli_si_all(uv0, 1), channelMask), _mmauto_and_si_all(_mmauto_srli_si_all(uv1, 1), channelMask))));
      }

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

      if(!classifyAllColors)
      {
        _mmauto_storet_si_all<true>(++coloredDest, classifyColorSimple<avx, classifyAllColors>(isColored0, isYWhite0, isHueField0, nullptr));
        _mmauto_storet_si_all<true>(++coloredDest, classifyColorSimple<avx, classifyAllColors>(isColored1, isYWhite1, isHueField1, nullptr));
      }
      else
      {
        alignas(avx ? 32 : 16)__m_auto_i isOtherColors0[FieldColors::numOfColors - FieldColors::numOfNonColors];
        alignas(avx ? 32 : 16)__m_auto_i isOtherColors1[FieldColors::numOfColors - FieldColors::numOfNonColors];

        for(int i = 0; i < FieldColors::numOfColors - FieldColors::numOfNonColors; ++i)
        {
          isOtherColors0[i] = isOtherColors1[i] = _mmauto_or_si_all(
              _mmauto_cmpeq_epi8(_mmauto_subs_epu8(isOtherColorsHMin[i], hue), _mmauto_subs_epu8(hue, isOtherColorsHMax[i])),
              _mmauto_cmpeq_epi8(_mmauto_subs_epu8(isOtherColorsHMax[i], hue), _mmauto_subs_epu8(hue, isOtherColorsHMin[i])));
          _mmauto_unpacklohi_epi8(isOtherColors0[i], isOtherColors1[i]);
        }

        _mmauto_storet_si_all<true>(++coloredDest, classifyColorSimple<avx, classifyAllColors>(isColored0, isYWhite0, isHueField0, isOtherColors0));
        _mmauto_storet_si_all<true>(++coloredDest, classifyColorSimple<avx, classifyAllColors>(isColored1, isYWhite1, isHueField1, isOtherColors1));
      }
    }
  }

  template<bool classifyAllColors, bool saveGrayscaled, bool saveColorchanneled, bool saveSaturation,  bool saveHue>
  void updateSSE(const YUYVPixel* const src, const int srcWidth, const int srcHeight, const FieldColors& theFieldColors,
                 TImage<PixelTypes::GrayscaledPixel>& grayscaled, TImage<PixelTypes::GrayscaledPixel>& ued, TImage<PixelTypes::GrayscaledPixel>& ved,
                 TImage<ColoredPixel>& colored, TImage<HuePixel>& hued, TImage<PixelTypes::GrayscaledPixel>& saturated)
  {
    if(simdAligned<_supportsAVX2>(src))
      classifyByYHS2FieldColorSSE<true, _supportsAVX2, classifyAllColors, saveGrayscaled, saveColorchanneled, saveSaturation, saveHue>(src, srcWidth, srcHeight, theFieldColors, grayscaled, ued, ved, colored, hued, saturated);
    else
      classifyByYHS2FieldColorSSE<false, _supportsAVX2, classifyAllColors, saveGrayscaled, saveColorchanneled, saveSaturation, saveHue>(src, srcWidth, srcHeight, theFieldColors, grayscaled, ued, ved, colored, hued, saturated);
  }
}
