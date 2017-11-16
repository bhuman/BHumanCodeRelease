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

  inline unsigned char computeLightingIndependentSaturation(const unsigned char y, const unsigned char u, const unsigned char v)
  {
    const short uC = u - 128;
    const short vC = v - 128;
    return static_cast<unsigned char>((isqrt((uC * uC + vC * vC) * 2) << 8) / y);
  }

  inline unsigned char computeHue(const unsigned char u, const unsigned char v)
  {
    return static_cast<unsigned char>(Approx::atan2(static_cast<short>(v - 128), static_cast<short>(u - 128)) >> 8);
  }

  template<bool avx> ALWAYSINLINE __m_auto_i computeSaturation(const __m_auto_i uv0, const __m_auto_i uv1)
  {
    return _mmauto_norm8_epi8<avx>(uv0, uv1);
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

  template<bool avx> ALWAYSINLINE __m_auto_i computeHue(const __m_auto_i uv0, const __m_auto_i uv1)
  {
    static const __m_auto_i loMask = _mmauto_set1_epi16(0x00FF);
    return _mmauto_atan2_epi8<avx>(
             _mmauto_correct_256op( // V
               _mmauto_packus_epi16(
                 _mmauto_srli_epi16(uv0, 8),
                 _mmauto_srli_epi16(uv1, 8)
               )
             ),
             _mmauto_correct_256op( // U
               _mmauto_packus_epi16(
                 _mmauto_and_si_all(uv0, loMask),
                 _mmauto_and_si_all(uv1, loMask)
               )
             )
           );
  }
}
