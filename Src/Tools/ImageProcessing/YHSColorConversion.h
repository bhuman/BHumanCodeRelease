/**
 * @file YHSColorConversion.h
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * @author Felix Thielke
 */

#pragma once

#include "Tools/ImageProcessing/AVX.h"
#include "Tools/Math/Approx.h"

namespace YHSColorConversion
{
  inline unsigned short isqrt(unsigned short n)
  {
    unsigned short res = 0;
    unsigned short bit = 1 << 14;

    for(size_t i = 0; i < 6; i++)
    {
      const unsigned short resPlusBit = res + bit;
      res >>= 1;
      if(n >= resPlusBit)
      {
        n -= resPlusBit;
        res += bit;
      }
      bit >>= 2;
    }

    return (res >> 1) + (n >= res + bit ? bit : 0);
  }

  inline unsigned char computeSaturation(const unsigned char u, const unsigned char v)
  {
    const short uC = u - 128;
    const short vC = v - 128;
    return static_cast<unsigned char>(isqrt((uC * uC + vC * vC) * 2));
  }

  inline unsigned char computeHue(const unsigned char u, const unsigned char v)
  {
    return static_cast<unsigned char>(Approx::atan2(static_cast<short>(v - 128), static_cast<short>(u - 128)) >> 8);
  }

  template<bool avx> ALWAYSINLINE __m_auto_i computeSaturation(const __m_auto_i uv0, const __m_auto_i uv1)
  {
    const __m_auto_i factor0 = _mmauto_abs_epi8(uv0);
    const __m_auto_i factor1 = _mmauto_abs_epi8(uv1);
    return _mmauto_sqrt16_epu8<avx>(
             _mmauto_slli_epi16(_mmauto_maddubs_epi16(factor0, factor0), 1),
             _mmauto_slli_epi16(_mmauto_maddubs_epi16(factor1, factor1), 1)
           );
  }

  template<bool avx> ALWAYSINLINE __m_auto_i computeLightingIndependentSaturation(const __m_auto_i y, const __m_auto_i uv)
  {
    static const __m_auto_i c_0 = _mmauto_setzero_si_all();
    static const __m_auto_i loMask = _mmauto_set1_epi32(0x0000FFFF);

    __m_auto_i y0 = y;
    __m_auto_i y2 = c_0;
    _mmauto_unpacklohi_epi8(y0, y2);
    const __m_auto_i y1 = _mmauto_srli_epi32(y0, 16);
    const __m_auto_i y3 = _mmauto_srli_epi32(y2, 16);
    y0 = _mmauto_and_si_all(y0, loMask);
    y2 = _mmauto_and_si_all(y2, loMask);

    const __m_auto_i absUV = _mmauto_abs_epi8(uv);
    __m_auto_i squaredNormUV0 = _mmauto_maddubs_epi16(absUV, absUV);
    __m_auto_i squaredNormUV1 = c_0;
    _mmauto_unpacklohi_epi16(squaredNormUV0, squaredNormUV1);

    const __m_auto rnormUV0 = _mmauto_rsqrt_ps(_mmauto_cvtepi32_ps(_mmauto_slli_epi32(squaredNormUV0, 17)));
    const __m_auto rnormUV1 = _mmauto_rsqrt_ps(_mmauto_cvtepi32_ps(_mmauto_slli_epi32(squaredNormUV1, 17)));

    return _mmauto_correct_256op(
             _mmauto_packus_epi16(
               _mmauto_or_si_all(
                 _mmauto_cvtps_epi32(_mmauto_rcp_ps(_mmauto_mul_ps(_mmauto_cvtepi32_ps(y0), rnormUV0))),
                 _mmauto_slli_epi32(_mmauto_cvtps_epi32(_mmauto_rcp_ps(_mmauto_mul_ps(_mmauto_cvtepi32_ps(y1), rnormUV0))), 16)
               ),
               _mmauto_or_si_all(
                 _mmauto_cvtps_epi32(_mmauto_rcp_ps(_mmauto_mul_ps(_mmauto_cvtepi32_ps(y2), rnormUV1))),
                 _mmauto_slli_epi32(_mmauto_cvtps_epi32(_mmauto_rcp_ps(_mmauto_mul_ps(_mmauto_cvtepi32_ps(y3), rnormUV1))), 16)
               )
             )
           );
  }

  template<bool avx> ALWAYSINLINE __m_auto_i _mmauto_div8_epi16(__m_auto_i min, __m_auto_i max)
  {
    __m_auto_i tally = _mmauto_set1_epi16(1 << 5);
    __m_auto_i retVal = _mmauto_setzero_si_all();
    max = _mmauto_slli_epi16(_mmauto_adds_epu16(max, _mmauto_set1_epi16(1)), 5);
    min = _mmauto_slli_epi16(min, 6);

    auto run = [&]()
    {
      const __m_auto_i temp = _mmauto_cmpgt_epi16(min, max);
      retVal = _mmauto_adds_epi16(retVal, _mmauto_and_si_all(temp, tally));
      min = _mmauto_sub_epi16(min, _mmauto_and_si_all(temp, max));
      tally = _mmauto_srli_epi16(tally, 1);
      max = _mmauto_srli_epi16(max, 1);
    };

    run();
    run();
    run();
    run();
    run();
    return retVal;
  }

  template<bool avx> ALWAYSINLINE __m_auto_i computeHue(const __m_auto_i uv0, const __m_auto_i uv1)
  {
    static const __m_auto_i c_0 = _mmauto_setzero_si_all();
    static const __m_auto_i c_64 = _mmauto_set1_epi8(64);
    static const __m_auto_i c_128 = _mmauto_set1_epi8(char(128));
    static const __m_auto_i c_129 = _mmauto_set1_epi8(char(0x81));
    static const __m_auto_i c_5695 = _mmauto_set1_epi16(5695);
    static const __m_auto_i c_11039 = _mmauto_set1_epi16(11039);
    static const __m_auto_i loMask = _mmauto_set1_epi16(0x00FF);

    const __m_auto_i u = _mmauto_correct_256op(
                           _mmauto_packus_epi16(
                             _mmauto_and_si_all(uv0, loMask),
                             _mmauto_and_si_all(uv1, loMask)
                           )
                         );
    const __m_auto_i v = _mmauto_correct_256op(
                           _mmauto_packus_epi16(
                             _mmauto_srli_epi16(uv0, 8),
                             _mmauto_srli_epi16(uv1, 8)
                           )
                         );

    const __m_auto_i absU = _mmauto_abs_epi8(u);
    const __m_auto_i absV = _mmauto_abs_epi8(v);

    const __m_auto_i min = _mmauto_min_epu8(absU, absV);

    __m_auto_i min0 = min;
    __m_auto_i min1 = c_0;
    _mmauto_unpacklohi_epi8(min0, min1);
    __m_auto_i max0 = _mmauto_max_epu8(absU, absV);
    __m_auto_i max1 = c_0;
    _mmauto_unpacklohi_epi8(max0, max1);
    const __m_auto_i quotient0 = _mmauto_div8_epi16<avx>(min0, max0);
    const __m_auto_i quotient1 = _mmauto_div8_epi16<avx>(min1, max1);

    const __m_auto_i absUnrotatedH = _mmauto_correct_256op(
                                       _mmauto_packs_epi16(
                                         _mmauto_mulhrs_epi16(_mmauto_sub_epi16(c_11039, _mmauto_mulhrs_epi16(c_5695, quotient0)), quotient0),
                                         _mmauto_mulhrs_epi16(_mmauto_sub_epi16(c_11039, _mmauto_mulhrs_epi16(c_5695, quotient1)), quotient1)
                                       )
                                     );
    const __m_auto_i uGtV = _mmauto_cmpeq_epi8(min, absV);
    return _mmauto_add_epi8(
             _mmauto_or_si_all(
               _mmauto_and_si_all(
                 uGtV,
                 _mmauto_and_si_all(u, c_128)
               ),
               _mmauto_andnot_si_all(
                 uGtV,
                 _mmauto_or_si_all(
                   c_64,
                   _mmauto_and_si_all(v, c_128)
                 )
               )
             ),
             _mmauto_sign_epi8(
               absUnrotatedH,
               _mmauto_sign_epi8(_mmauto_xor_si_all(uGtV, c_129), _mmauto_sign_epi8(u, v))
             )
           );
  }
}
