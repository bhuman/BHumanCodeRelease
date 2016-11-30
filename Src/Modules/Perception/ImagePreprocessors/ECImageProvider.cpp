/**
 * @file ECImageProvider.cpp
 * @author Felix Thielke
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "ECImageProvider.h"
#include "Tools/ImageProcessing/AVX.h"
#include "Tools/ImageProcessing/YHSColorConversion.h"
#include "Tools/Math/Approx.h"
#include "Tools/Debugging/DebugImages.h"
#include "Tools/ImageProcessing/ColorModelConversions.h"
#define PREFETCH

using namespace PixelTypes;

MAKE_MODULE(ECImageProvider, perception)

void ECImageProvider::update(ECImage& ecImage)
{
  if(theImage.timeStamp > 10)
  {
    ecImage.grayscaled.setResolution(theCameraInfo.width, theCameraInfo.height);
    ecImage.saturated.setResolution(theCameraInfo.width, theCameraInfo.height);
    ecImage.hued.setResolution(theCameraInfo.width, theCameraInfo.height);
    ecImage.colored.setResolution(theCameraInfo.width, theCameraInfo.height);

    ASSERT((theCameraInfo.width * theCameraInfo.height) % 32 == 0);
    if(theImage.width == theCameraInfo.width)
    {
#ifdef TARGET_ROBOT
      ASSERT(false);
#endif // TARGET_ROBOT
      classifyByYHSFieldColor<true>(ecImage);
    }
    else
    {
      if(sseOptimized)
      {
        if(simpleClassification)
          updateSSE<true>(ecImage);
        else
          updateSSE<false>(ecImage);
      }
      else
        classifyByYHSFieldColor<false>(ecImage);
    }
  }
}

template<bool simple> void ECImageProvider::updateSSE(ECImage& ecImage)
{
  switch(mode)
  {
    case yhs:
      if(_supportsAVX2)
      {
        if(simdAligned<true>(theImage[0]))
          classifyByYHSFieldColorSSE<true, true>(ecImage);
        else
          classifyByYHSFieldColorSSE<false, true>(ecImage);
      }
      else
      {
        if(simdAligned<false>(theImage[0]))
          classifyByYHSFieldColorSSE<true, false>(ecImage);
        else
          classifyByYHSFieldColorSSE<false, false>(ecImage);
      }
      break;
    case yhs2:
      if(_supportsAVX2)
      {
        if(simdAligned<true>(theImage[0]))
          classifyByYHS2FieldColorSSE<true, true, simple>(ecImage);
        else
          classifyByYHS2FieldColorSSE<false, true, simple>(ecImage);
      }
      else
      {
        if(simdAligned<false>(theImage[0]))
          classifyByYHS2FieldColorSSE<true, false, simple>(ecImage);
        else
          classifyByYHS2FieldColorSSE<false, false, simple>(ecImage);
      }
      break;
    case hsi:
      if(_supportsAVX2)
      {
        if(simdAligned<true>(theImage[0]))
          classifyByHSIFieldColorSSE<true, true, simple>(ecImage);
        else
          classifyByHSIFieldColorSSE<false, true, simple>(ecImage);
      }
      else
      {
        if(simdAligned<false>(theImage[0]))
          classifyByHSIFieldColorSSE<true, false, simple>(ecImage);
        else
          classifyByHSIFieldColorSSE<false, false, simple>(ecImage);
      }
      break;
    default:
      ;
  }
}

template<bool isSmall>
void ECImageProvider::classifyByYHSFieldColor(ECImage& ecImage) const
{
  GrayscaledPixel* grayscaledDest = ecImage.grayscaled[0];
  ColoredPixel* coloredDest = ecImage.colored[0];
  GrayscaledPixel* saturatedDest = ecImage.saturated[0];
  GrayscaledPixel* huedDest = ecImage.hued[0];

  const Image::Pixel* const imageEnd = theImage[theImage.height];

  const int lineEndStep = isSmall ? theImage.widthStep : theImage.width;
  const int srcStep = isSmall ? theImage.width : 0;

  const Image::Pixel* src = theImage[0];
  for(const Image::Pixel* lineEnd = &src[theImage.width]; lineEnd <= imageEnd; lineEnd += lineEndStep, src += srcStep)
    for(; src < lineEnd; src++)
    {
      *grayscaledDest++ = static_cast<GrayscaledPixel>(src->yCbCrPadding);
      if(!isSmall)
        *grayscaledDest++ = static_cast<GrayscaledPixel>(src->y);

      const short u = src->cb - 128;
      const short v = src->cr - 128;
      const unsigned char hue = static_cast<unsigned char>(Approx::atan2(v, u) >> 8) - 1;
      const unsigned char sat = static_cast<unsigned char>(sqrt(static_cast<float>(static_cast<int>(u * u + v * v) << 1)));

      *saturatedDest++ = static_cast<GrayscaledPixel>(sat);
      if(!isSmall)
        *saturatedDest++ = static_cast<GrayscaledPixel>(sat);

      *huedDest++ = static_cast<GrayscaledPixel>(hue);
      if(!isSmall)
        *huedDest++ = static_cast<GrayscaledPixel>(hue);

      *coloredDest++ = (theFieldColors[FieldColors::green].y.isInside(src->yCbCrPadding) &&
                        theFieldColors[FieldColors::green].h.isInside(hue) &&
                        theFieldColors[FieldColors::green].s.isInside(sat))
                       ? FieldColors::Color::green :
                       (src->yCbCrPadding >= theFieldColors.minYWhite ? FieldColors::Color::white :
                        (src->yCbCrPadding <= theFieldColors.maxYBlack ? FieldColors::Color::black :
                         FieldColors::Color::none));
      if(!isSmall)
        *coloredDest++ = (theFieldColors[FieldColors::green].y.isInside(src->y) &&
                          theFieldColors[FieldColors::green].h.isInside(hue) &&
                          theFieldColors[FieldColors::green].s.isInside(sat))
                         ? FieldColors::Color::green :
                         (src->y >= theFieldColors.minYWhite ? FieldColors::Color::white :
                          (src->y <= theFieldColors.maxYBlack ? FieldColors::Color::black :
                           FieldColors::Color::none));
    }
}

template<bool avx> ALWAYSINLINE __m_auto_i classifyColor(const __m_auto_i y, const __m_auto_i s,
    const __m_auto_i isHueGreen,
    const __m_auto_i greenSFrom,
    const __m_auto_i greenSTo,
    const __m_auto_i greenYFrom,
    const __m_auto_i greenYTo,
    const __m_auto_i blackYTo,
    const __m_auto_i whiteYFrom)
{
  static const __m_auto_i c_0 = _mmauto_setzero_si_all();
  static const __m_auto_i classWhite = _mmauto_set1_epi8(FieldColors::Color::white);
  static const __m_auto_i classGreen = _mmauto_set1_epi8(FieldColors::Color::green);
  static const __m_auto_i classBlack = _mmauto_set1_epi8(FieldColors::Color::black);
  static_assert((int)FieldColors::Color::none == 0, "");

  const __m_auto_i isYWhite = _mmauto_cmpeq_epi8(_mmauto_subs_epu8(whiteYFrom, y), c_0);
  const __m_auto_i isYBlack = _mmauto_cmpeq_epi8(_mmauto_subs_epu8(y, blackYTo), c_0);
  const __m_auto_i isGreen = _mmauto_and_si_all(
                               isHueGreen,
                               _mmauto_and_si_all(
                                 _mmauto_cmpeq_epi8(_mmauto_subs_epu8(greenYFrom, y), _mmauto_subs_epu8(y, greenYTo)),
                                 _mmauto_cmpeq_epi8(_mmauto_subs_epu8(greenSFrom, s), _mmauto_subs_epu8(s, greenSTo))
                               )
                             );

  return _mmauto_or_si_all(
           _mmauto_and_si_all(isGreen, classGreen),
           _mmauto_or_si_all(
             _mmauto_and_si_all(isYWhite, classWhite),
             _mmauto_and_si_all(isYBlack, classBlack)
           )
         );
}

template<bool avx> ALWAYSINLINE __m_auto_i classifyColor(const __m_auto_i y, const __m_auto_i s,
    const __m_auto_i isHueGreen,
    const __m_auto_i greenSFrom,
    const __m_auto_i greenSTo,
    const __m_auto_i greenYFrom,
    const __m_auto_i greenYTo,
    const __m_auto_i maxNonColorSaturation,
    const __m_auto_i blackYTo,
    const __m_auto_i whiteYFrom)
{
  static const __m_auto_i c_0 = _mmauto_setzero_si_all();
  static const __m_auto_i classWhite = _mmauto_set1_epi8(FieldColors::Color::white);
  static const __m_auto_i classGreen = _mmauto_set1_epi8(FieldColors::Color::green);
  static const __m_auto_i classBlack = _mmauto_set1_epi8(FieldColors::Color::black);
  static_assert((int)FieldColors::Color::none == 0, "");

  const __m_auto_i isColored = _mmauto_cmpeq_epi8(_mmauto_subs_epu8(maxNonColorSaturation, s), c_0);
  const __m_auto_i isYWhite = _mmauto_cmpeq_epi8(_mmauto_subs_epu8(whiteYFrom, y), c_0);
  const __m_auto_i isYBlack = _mmauto_cmpeq_epi8(_mmauto_subs_epu8(y, blackYTo), c_0);
  const __m_auto_i isGreen = _mmauto_and_si_all(
                               isHueGreen,
                               _mmauto_and_si_all(
                                 _mmauto_cmpeq_epi8(_mmauto_subs_epu8(greenYFrom, y), _mmauto_subs_epu8(y, greenYTo)),
                                 _mmauto_cmpeq_epi8(_mmauto_subs_epu8(greenSFrom, s), _mmauto_subs_epu8(s, greenSTo))
                               )
                             );

  return _mmauto_or_si_all(
           _mmauto_and_si_all(
             isColored,
             _mmauto_and_si_all(isGreen, classGreen)
           ),
           _mmauto_andnot_si_all(
             isColored,
             _mmauto_or_si_all(
               _mmauto_and_si_all(isYWhite, classWhite),
               _mmauto_and_si_all(isYBlack, classBlack)
             )
           )
         );
}

template<bool avx> ALWAYSINLINE __m_auto_i classifyColorSimple(
  const __m_auto_i y,
  const __m_auto_i s,
  const __m_auto_i isHueGreen,
  const __m_auto_i maxNonColorSaturation,
  const __m_auto_i blackWhiteThreshold)
{
  static const __m_auto_i c_0 = _mmauto_setzero_si_all();
  static const __m_auto_i classWhite = _mmauto_set1_epi8(FieldColors::Color::white);
  static const __m_auto_i classGreen = _mmauto_set1_epi8(FieldColors::Color::green);
  static const __m_auto_i classBlack = _mmauto_set1_epi8(FieldColors::Color::black);
  static_assert((int)FieldColors::Color::none == 0, "");

  const __m_auto_i isColored = _mmauto_cmpeq_epi8(_mmauto_subs_epu8(maxNonColorSaturation, s), c_0);
  const __m_auto_i isYWhite = _mmauto_cmpeq_epi8(_mmauto_subs_epu8(blackWhiteThreshold, y), c_0);

  return _mmauto_or_si_all(
           _mmauto_and_si_all(
             isColored,
             _mmauto_and_si_all(isHueGreen, classGreen)
           ),
           _mmauto_andnot_si_all(
             isColored,
             _mmauto_or_si_all(
               _mmauto_and_si_all(isYWhite, classWhite),
               _mmauto_andnot_si_all(isYWhite, classBlack)
             )
           )
         );
}

template<bool aligned, bool avx>
void ECImageProvider::classifyByYHSFieldColorSSE(ECImage& ecImage) const
{
  ASSERT(theImage.width % 32 == 0);

  __m_auto_i* grayscaledDest = reinterpret_cast<__m_auto_i*>(ecImage.grayscaled[0]) - 1;
  __m_auto_i* coloredDest = reinterpret_cast<__m_auto_i*>(ecImage.colored[0]) - 1;
  __m_auto_i* saturatedDest = reinterpret_cast<__m_auto_i*>(ecImage.saturated[0]) - 1;
  __m_auto_i* huedDest = reinterpret_cast<__m_auto_i*>(ecImage.hued[0]) - 1;
  const __m_auto_i* const imageEnd = reinterpret_cast<const __m_auto_i*>(theImage[theImage.height]) - 1;

  const __m_auto_i greenHFrom = _mmauto_set1_epi8(theFieldColors[FieldColors::green].h.min);
  const __m_auto_i greenHTo = _mmauto_set1_epi8(theFieldColors[FieldColors::green].h.max);
  const __m_auto_i greenSFrom = _mmauto_set1_epi8(theFieldColors[FieldColors::green].s.min);
  const __m_auto_i greenSTo = _mmauto_set1_epi8(theFieldColors[FieldColors::green].s.max);
  const __m_auto_i greenYFrom = _mmauto_set1_epi8(theFieldColors[FieldColors::green].y.min);
  const __m_auto_i greenYTo = _mmauto_set1_epi8(theFieldColors[FieldColors::green].y.max);

  const __m_auto_i blackYTo = _mmauto_set1_epi8(theFieldColors.maxYBlack);
  const __m_auto_i whiteYFrom = _mmauto_set1_epi8(theFieldColors.minYWhite);

  static const __m_auto_i c_128 = _mmauto_set1_epi8(char(128));
  static const __m_auto_i channelMask = _mmauto_set1_epi16(0x00FF);

#ifdef PREFETCH
  const char* prefetchSrc = reinterpret_cast<const char*>(theImage[0]) + (avx ? 128 : 64);
  const char* prefetchGrayscaledDest = reinterpret_cast<const char*>(ecImage.grayscaled[0]) + (avx ? 64 : 32);
  const char* prefetchColoredDest = reinterpret_cast<const char*>(ecImage.colored[0]) - (avx ? 64 : 32);
#endif // PREFETCH

  const __m_auto_i* src = reinterpret_cast<__m_auto_i const*>(theImage[0]) - 1;
  while(src < imageEnd)
  {
    const __m_auto_i p0 = _mmauto_loadt_si_all<aligned>(++src);
    const __m_auto_i p1 = _mmauto_loadt_si_all<aligned>(++src);
    const __m_auto_i p2 = _mmauto_loadt_si_all<aligned>(++src);
    const __m_auto_i p3 = _mmauto_loadt_si_all<aligned>(++src);

    // Compute luminance
    const __m_auto_i y0 = _mmauto_correct_256op(_mmauto_packus_epi16(_mmauto_and_si_all(p0, channelMask), _mmauto_and_si_all(p1, channelMask)));
    const __m_auto_i y1 = _mmauto_correct_256op(_mmauto_packus_epi16(_mmauto_and_si_all(p2, channelMask), _mmauto_and_si_all(p3, channelMask)));
    _mmauto_storet_si_all<true>(++grayscaledDest, y0);
    _mmauto_storet_si_all<true>(++grayscaledDest, y1);

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

    __m_auto_i sat0 = YHSColorConversion::computeSaturation<avx>(uv0, uv1);
    __m_auto_i sat1 = sat0;
    _mmauto_unpacklohi_epi8(sat0, sat1);

    _mmauto_streamt_si_all<true>(++saturatedDest, sat0);
    _mmauto_streamt_si_all<true>(++saturatedDest, sat1);

    // Compute hue
    const __m_auto_i hue = YHSColorConversion::computeHue<avx>(uv0, uv1);

    __m_auto_i hue0 = hue;
    __m_auto_i hue1 = hue;
    _mmauto_unpacklohi_epi8(hue0, hue1);

    _mmauto_streamt_si_all<true>(++huedDest, hue0);
    _mmauto_streamt_si_all<true>(++huedDest, hue1);

    // classify
    const __m_auto_i isHueGreen = _mmauto_cmpeq_epi8(_mmauto_subs_epu8(greenHFrom, hue), _mmauto_subs_epu8(hue, greenHTo));
    __m_auto_i isHueGreen0 = isHueGreen;
    __m_auto_i isHueGreen1 = isHueGreen;
    _mmauto_unpacklohi_epi8(isHueGreen0, isHueGreen1);

    _mmauto_storet_si_all<true>(++coloredDest,
                                classifyColor<avx>(y0, sat0, isHueGreen0, greenSFrom, greenSTo, greenYFrom, greenYTo, blackYTo, whiteYFrom)
                               );
    _mmauto_storet_si_all<true>(++coloredDest,
                                classifyColor<avx>(y1, sat1, isHueGreen1, greenSFrom, greenSTo, greenYFrom, greenYTo, blackYTo, whiteYFrom)
                               );
  }
}

template<bool aligned, bool avx, bool simple>
void ECImageProvider::classifyByYHS2FieldColorSSE(ECImage& ecImage) const
{
  ASSERT(theImage.width % 32 == 0);

  __m_auto_i* grayscaledDest = reinterpret_cast<__m_auto_i*>(ecImage.grayscaled[0]) - 1;
  __m_auto_i* coloredDest = reinterpret_cast<__m_auto_i*>(ecImage.colored[0]) - 1;
  __m_auto_i* saturatedDest = reinterpret_cast<__m_auto_i*>(ecImage.saturated[0]) - 1;
  __m_auto_i* huedDest = reinterpret_cast<__m_auto_i*>(ecImage.hued[0]) - 1;
  const __m_auto_i* const imageEnd = reinterpret_cast<const __m_auto_i*>(theImage[theImage.height]) - 1;

  const __m_auto_i greenHFrom = _mmauto_set1_epi8(theFieldColors[FieldColors::green].h.min);
  const __m_auto_i greenHTo = _mmauto_set1_epi8(theFieldColors[FieldColors::green].h.max);
  const __m_auto_i greenSFrom = _mmauto_set1_epi8(theFieldColors[FieldColors::green].s.min);
  const __m_auto_i greenSTo = _mmauto_set1_epi8(theFieldColors[FieldColors::green].s.max);
  const __m_auto_i greenYFrom = _mmauto_set1_epi8(theFieldColors[FieldColors::green].y.min);
  const __m_auto_i greenYTo = _mmauto_set1_epi8(theFieldColors[FieldColors::green].y.max);

  const __m_auto_i maxNonColorSaturation = _mmauto_set1_epi8(theFieldColors.maxNonColorSaturation);
  const __m_auto_i blackYTo = _mmauto_set1_epi8(theFieldColors.maxYBlack);
  const __m_auto_i whiteYFrom = _mmauto_set1_epi8(theFieldColors.minYWhite);

  static const __m_auto_i c_128 = _mmauto_set1_epi8(char(128));
  static const __m_auto_i channelMask = _mmauto_set1_epi16(0x00FF);

#ifdef PREFETCH
  const char* prefetchSrc = reinterpret_cast<const char*>(theImage[0]) + (avx ? 128 : 64);
  const char* prefetchGrayscaledDest = reinterpret_cast<const char*>(ecImage.grayscaled[0]) + (avx ? 64 : 32);
  const char* prefetchColoredDest = reinterpret_cast<const char*>(ecImage.colored[0]) - (avx ? 64 : 32);
#endif // PREFETCH

  const __m_auto_i* src = reinterpret_cast<__m_auto_i const*>(theImage[0]) - 1;
  while(src < imageEnd)
  {
    const __m_auto_i p0 = _mmauto_loadt_si_all<aligned>(++src);
    const __m_auto_i p1 = _mmauto_loadt_si_all<aligned>(++src);
    const __m_auto_i p2 = _mmauto_loadt_si_all<aligned>(++src);
    const __m_auto_i p3 = _mmauto_loadt_si_all<aligned>(++src);

    // Compute luminance
    const __m_auto_i y0 = _mmauto_correct_256op(_mmauto_packus_epi16(_mmauto_and_si_all(p0, channelMask), _mmauto_and_si_all(p1, channelMask)));
    const __m_auto_i y1 = _mmauto_correct_256op(_mmauto_packus_epi16(_mmauto_and_si_all(p2, channelMask), _mmauto_and_si_all(p3, channelMask)));
    _mmauto_storet_si_all<true>(++grayscaledDest, y0);
    _mmauto_storet_si_all<true>(++grayscaledDest, y1);

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
    _mmauto_streamt_si_all<true>(++saturatedDest, sat0);
    _mmauto_streamt_si_all<true>(++saturatedDest, sat1);

    // Compute hue
    const __m_auto_i hue = YHSColorConversion::computeHue<avx>(uv0, uv1);
    __m_auto_i hue0 = hue;
    __m_auto_i hue1 = hue;
    _mmauto_unpacklohi_epi8(hue0, hue1);

    _mmauto_streamt_si_all<true>(++huedDest, hue0);
    _mmauto_streamt_si_all<true>(++huedDest, hue1);

    // classify
    const __m_auto_i isHueGreen = _mmauto_cmpeq_epi8(_mmauto_subs_epu8(greenHFrom, hue), _mmauto_subs_epu8(hue, greenHTo));
    __m_auto_i isHueGreen0 = isHueGreen;
    __m_auto_i isHueGreen1 = isHueGreen;
    _mmauto_unpacklohi_epi8(isHueGreen0, isHueGreen1);

    _mmauto_storet_si_all<true>(++coloredDest,
                                simple ? classifyColorSimple<avx>(y0, sat0, isHueGreen0, maxNonColorSaturation, whiteYFrom) : classifyColor<avx>(y0, sat0, isHueGreen0, greenSFrom, greenSTo, greenYFrom, greenYTo, maxNonColorSaturation, blackYTo, whiteYFrom)
                               );
    _mmauto_storet_si_all<true>(++coloredDest,
                                simple ? classifyColorSimple<avx>(y1, sat1, isHueGreen1, maxNonColorSaturation, whiteYFrom) : classifyColor<avx>(y1, sat1, isHueGreen1, greenSFrom, greenSTo, greenYFrom, greenYTo, maxNonColorSaturation, blackYTo, whiteYFrom)
                               );
  }
}

template<bool aligned, bool avx, bool simple>
void ECImageProvider::classifyByHSIFieldColorSSE(ECImage& ecImage) const
{
  ASSERT(theImage.width % 32 == 0);

  __m_auto_i* grayscaledDest = reinterpret_cast<__m_auto_i*>(ecImage.grayscaled[0]) - 1;
  __m_auto_i* coloredDest = reinterpret_cast<__m_auto_i*>(ecImage.colored[0]) - 1;
  __m_auto_i* saturatedDest = reinterpret_cast<__m_auto_i*>(ecImage.saturated[0]) - 1;
  __m_auto_i* huedDest = reinterpret_cast<__m_auto_i*>(ecImage.hued[0]) - 1;
  const __m_auto_i* const imageEnd = reinterpret_cast<const __m_auto_i*>(theImage[theImage.height]) - 1;

  const __m_auto_i greenHFrom = _mmauto_set1_epi8(theFieldColors[FieldColors::green].h.min);
  const __m_auto_i greenHTo = _mmauto_set1_epi8(theFieldColors[FieldColors::green].h.max);
  const __m_auto_i greenSFrom = _mmauto_set1_epi8(theFieldColors[FieldColors::green].s.min);
  const __m_auto_i greenSTo = _mmauto_set1_epi8(theFieldColors[FieldColors::green].s.max);
  const __m_auto_i greenYFrom = _mmauto_set1_epi8(theFieldColors[FieldColors::green].y.min);
  const __m_auto_i greenYTo = _mmauto_set1_epi8(theFieldColors[FieldColors::green].y.max);

  const __m_auto_i maxNonColorSaturation = _mmauto_set1_epi8(theFieldColors.maxNonColorSaturation);
  const __m_auto_i blackYTo = _mmauto_set1_epi8(theFieldColors.maxYBlack);
  const __m_auto_i whiteYFrom = _mmauto_set1_epi8(theFieldColors.minYWhite);

  static const __m_auto_i c_0 = _mmauto_setzero_si_all();
  static const __m_auto_i c_128 = _mmauto_set1_epi8(char(128));
  static const __m_auto_i channelMask = _mmauto_set1_epi16(0x00FF);

  static const __m_auto_i c_85 = _mmauto_set1_epi8(85);
  static const __m_auto_i c_171 = _mmauto_set1_epi8(char(171));

  static const __m_auto_i scaledInvUCoeff = _mmauto_set1_epi16(static_cast<short>(static_cast<float>(1 << 14) / ColorModelConversions::uCoeff));
  static const __m_auto_i scaledInvVCoeff = _mmauto_set1_epi16(static_cast<short>(static_cast<float>(1 << 14) / ColorModelConversions::vCoeff));
  static const __m_auto_i scaledGCoeffU = _mmauto_set1_epi16(static_cast<short>(ColorModelConversions::yCoeffB / (ColorModelConversions::yCoeffG * ColorModelConversions::uCoeff) * static_cast<float>(1 << 15)));
  static const __m_auto_i scaledGCoeffV = _mmauto_set1_epi16(static_cast<short>(ColorModelConversions::yCoeffR / (ColorModelConversions::yCoeffG * ColorModelConversions::vCoeff) * static_cast<float>(1 << 15)));

#ifdef PREFETCH
  const char* prefetchSrc = reinterpret_cast<const char*>(theImage[0]) + (avx ? 128 : 64);
  const char* prefetchGrayscaledDest = reinterpret_cast<const char*>(ecImage.grayscaled[0]) + (avx ? 64 : 32);
  const char* prefetchColoredDest = reinterpret_cast<const char*>(ecImage.colored[0]) - (avx ? 64 : 32);
#endif // PREFETCH

  const __m_auto_i* src = reinterpret_cast<__m_auto_i const*>(theImage[0]) - 1;
  while(src < imageEnd)
  {
    const __m_auto_i p0 = _mmauto_loadt_si_all<aligned>(++src);
    const __m_auto_i p1 = _mmauto_loadt_si_all<aligned>(++src);

    // Compute luminance
    const __m_auto_i y0 = _mmauto_and_si_all(p0, channelMask);
    const __m_auto_i y1 = _mmauto_and_si_all(p1, channelMask);
    const __m_auto_i y = _mmauto_correct_256op(_mmauto_packus_epi16(y0, y1));
    _mmauto_storet_si_all<true>(++grayscaledDest, y);

#ifdef PREFETCH
    _mm_prefetch(prefetchColoredDest += 32, _MM_HINT_T0);

    _mm_prefetch(prefetchSrc += 32, _MM_HINT_T0);
    if(avx) _mm_prefetch(prefetchSrc += 32, _MM_HINT_T0);

    _mm_prefetch(prefetchGrayscaledDest += 32, _MM_HINT_T0);
#endif // PREFETCH

    // Convert to RGB
    const __m_auto_i uv = _mmauto_sub_epi8(_mmauto_correct_256op(_mmauto_packus_epi16(_mmauto_and_si_all(_mmauto_srli_si_all(p0, 1), channelMask), _mmauto_and_si_all(_mmauto_srli_si_all(p1, 1), channelMask))), c_128);
    const __m_auto_i u = _mmauto_srai_epi16(_mmauto_slli_epi16(uv, 8), 8);
    const __m_auto_i v = _mmauto_srai_epi16(uv, 8);

    __m_auto_i uvB0 = _mmauto_slli_epi16(_mmauto_mulhrs_epi16(u, scaledInvUCoeff), 1);
    __m_auto_i uvB1 = uvB0;
    _mmauto_unpacklohi_epi16(uvB0, uvB1);
    __m_auto_i uvR0 = _mmauto_slli_epi16(_mmauto_mulhrs_epi16(v, scaledInvVCoeff), 1);
    __m_auto_i uvR1 = uvR0;
    _mmauto_unpacklohi_epi16(uvR0, uvR1);
    __m_auto_i uvG0 = _mmauto_add_epi16(_mmauto_mulhrs_epi16(u, scaledGCoeffU), _mmauto_mulhrs_epi16(v, scaledGCoeffV));
    __m_auto_i uvG1 = uvG0;
    _mmauto_unpacklohi_epi16(uvG0, uvG1);

    const __m_auto_i b = _mmauto_correct_256op(_mmauto_packus_epi16(
                           _mmauto_add_epi16(y0, uvB0),
                           _mmauto_add_epi16(y1, uvB1)
                         ));
    const __m_auto_i r = _mmauto_correct_256op(_mmauto_packus_epi16(
                           _mmauto_add_epi16(y0, uvR0),
                           _mmauto_add_epi16(y1, uvR1)
                         ));
    const __m_auto_i g = _mmauto_correct_256op(_mmauto_packus_epi16(
                           _mmauto_sub_epi16(y0, uvG0),
                           _mmauto_sub_epi16(y1, uvG1)
                         ));

    // Convert to HS(I)
    const __m_auto_i rgbMax = _mmauto_max_epu8(r, _mmauto_max_epu8(g, b));

    const __m_auto_i c = _mmauto_sub_epi8(rgbMax, _mmauto_min_epu8(r, _mmauto_min_epu8(g, b)));

    const __m_auto_i rMax = _mmauto_cmpeq_epi8(rgbMax, r);
    const __m_auto_i gMax = _mmauto_andnot_si_all(rMax, _mmauto_cmpeq_epi8(rgbMax, g));
    const __m_auto_i bMax = _mmauto_andnot_si_all(rMax, _mmauto_andnot_si_all(gMax, _mmauto_cmpeq_epi8(rgbMax, b)));

    const __m_auto_i minuend = _mmauto_or_si_all(
                                 _mmauto_and_si_all(rMax, g),
                                 _mmauto_or_si_all(
                                   _mmauto_and_si_all(gMax, b),
                                   _mmauto_and_si_all(bMax, r)
                                 )
                               );
    const __m_auto_i subtrahend = _mmauto_or_si_all(
                                    _mmauto_and_si_all(rMax, b),
                                    _mmauto_or_si_all(
                                      _mmauto_and_si_all(gMax, r),
                                      _mmauto_and_si_all(bMax, g)
                                    )
                                  );
    const __m_auto_i absUnshiftedH = _mmauto_divq8_epu8<avx>(_mmauto_subs_epu8(_mmauto_max_epu8(minuend, subtrahend), _mmauto_min_epu8(minuend, subtrahend)), c);
    const __m_auto_i offset = _mmauto_or_si_all(
                                _mmauto_and_si_all(gMax, c_85),
                                _mmauto_and_si_all(bMax, c_171)
                              );
    const __m_auto_i hNegative = _mmauto_cmpeq_epi8(_mmauto_subs_epu8(minuend, subtrahend), c_0);

    const __m_auto_i hue = _mmauto_andnot_si_all(
                             _mmauto_cmpeq_epi8(c, c_0),
                             _mmauto_or_si_all(
                               _mmauto_andnot_si_all(hNegative, _mmauto_add_epi8(offset, absUnshiftedH)),
                               _mmauto_and_si_all(hNegative, _mmauto_sub_epi8(offset, absUnshiftedH))
                             )
                           );
    _mmauto_streamt_si_all<true>(++huedDest, hue);

    const __m_auto_i sat = _mmauto_divq8_epu8<avx>(c, rgbMax);
    _mmauto_streamt_si_all<true>(++saturatedDest, sat);

    // classify
    const __m_auto_i isHueGreen = _mmauto_cmpeq_epi8(_mmauto_subs_epu8(greenHFrom, hue), _mmauto_subs_epu8(hue, greenHTo));
    _mmauto_storet_si_all<true>(++coloredDest,
                                simple ? classifyColorSimple<avx>(y, sat, isHueGreen, maxNonColorSaturation, whiteYFrom) : classifyColor<avx>(y, sat, isHueGreen, greenSFrom, greenSTo, greenYFrom, greenYTo, maxNonColorSaturation, blackYTo, whiteYFrom)
                               );
  }
}
