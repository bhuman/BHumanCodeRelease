/**
 * @file AVX.h
 *
 * @author Felix Thielke
 */

#pragma once

#include "SIMD.h"

ALWAYSINLINE bool aligned32(const void* x) { return ((reinterpret_cast<size_t>(x)) & 0x1f) == 0; }
template<bool avx> ALWAYSINLINE bool simdAligned(const void* x) { return avx ? aligned32(x) : aligned16(x); }

#if defined TARGET_ROBOT || !defined __AVX2__
#define DOES_DEFINITELY_NOT_SUPPORT_AVX2
#define _supportsAVX2 false

#define __m_auto_i_using(avx) __m128i
#define __m_auto_using(avx) __m128
#define DEFINE_FUNCTION_M(name, name128, name256) \
  static ALWAYSINLINE __m128 name (const __m128 a){return name128 (a);}
#define DEFINE_FUNCTION_MM(name, name128, name256) \
  static ALWAYSINLINE __m128 name (const __m128 a, const __m128 b){return name128 (a, b);}
#define DEFINE_FUNCTIONI_M(name, name128, name256) \
  static ALWAYSINLINE __m128i name (const __m128i a){return name128 (a);}
#define DEFINE_FUNCTIONI_MM(name, name128, name256) \
  static ALWAYSINLINE __m128i name (const __m128i a, const __m128i b){return name128 (a, b);}
#define DEFINE_FUNCTIONI_MI(name, name128, name256) \
  template<int imm> static ALWAYSINLINE __m128i name (const __m128i a){return name128 (a, imm);}
#define DEFINE_FUNCTIONI_MMI(name, name128, name256) \
  template<int imm> static ALWAYSINLINE __m128i name (const __m128i a, const __m128i b){return name128 (a, b, imm);}
#else
#define _supportsAVX2 true
#define __m_auto_i_using(avx) typename std::conditional<avx,__m256i,__m128i>::type
#define __m_auto_using(avx) typename std::conditional<avx,__m256,__m128>::type
#define DEFINE_FUNCTION_M(name, name128, name256) \
  static ALWAYSINLINE __m128 name (const __m128 a){return name128 (a);} \
  static ALWAYSINLINE __m256 name (const __m256 a){return name256 (a);}
#define DEFINE_FUNCTION_MM(name, name128, name256) \
  static ALWAYSINLINE __m128 name (const __m128 a, const __m128 b){return name128 (a, b);} \
  static ALWAYSINLINE __m256 name (const __m256 a, const __m256 b){return name256 (a, b);}
#define DEFINE_FUNCTIONI_M(name, name128, name256) \
  static ALWAYSINLINE __m128i name (const __m128i a){return name128 (a);} \
  static ALWAYSINLINE __m256i name (const __m256i a){return name256 (a);}
#define DEFINE_FUNCTIONI_MM(name, name128, name256) \
  static ALWAYSINLINE __m128i name (const __m128i a, const __m128i b){return name128 (a, b);} \
  static ALWAYSINLINE __m256i name (const __m256i a, const __m256i b){return name256 (a, b);}
#define DEFINE_FUNCTIONI_MI(name, name128, name256) \
  template<int imm> static ALWAYSINLINE __m128i name (const __m128i a){return name128 (a, imm);} \
  template<int imm> static ALWAYSINLINE __m256i name (const __m256i a){return name256 (a, imm);}
#define DEFINE_FUNCTIONI_MMI(name, name128, name256) \
  template<int imm> static ALWAYSINLINE __m128i name (const __m128i a, const __m128i b){return name128 (a, b, imm);} \
  template<int imm> static ALWAYSINLINE __m256i name (const __m256i a, const __m256i b){return name256 (a, b, imm);}
#endif

ALWAYSINLINE __m128i _mm_slli_epi8(const __m128i& a, int imm)
{
  return _mm_and_si128(_mm_slli_epi16(a, imm), _mm_set1_epi8(static_cast<unsigned char>(0xFF << imm)));
}
ALWAYSINLINE __m128i _mm_srli_epi8(const __m128i a, int imm)
{
  return _mm_and_si128(_mm_srli_epi16(a, imm), _mm_set1_epi8(0xFF >> imm));
}
#ifndef DOES_DEFINITELY_NOT_SUPPORT_AVX2
ALWAYSINLINE __m256i _mm256_slli_epi8(const __m256i a, int imm)
{
  return _mm256_and_si256(_mm256_slli_epi16(a, imm), _mm256_set1_epi8(static_cast<unsigned char>(0xFF << imm)));
}
ALWAYSINLINE __m256i _mm256_srli_epi8(const __m256i a, int imm)
{
  return _mm256_and_si256(_mm256_srli_epi16(a, imm), _mm256_set1_epi8(0xFF >> imm));
}
#endif

template<typename T> static ALWAYSINLINE T _mmauto_set1_ps(const float f);
template<> ALWAYSINLINE __m128 _mmauto_set1_ps<>(const float f)
{
  return _mm_set1_ps(f);
}
template<typename T> static ALWAYSINLINE T _mmauto_set1_epi32(const int i);
template<> ALWAYSINLINE __m128i _mmauto_set1_epi32<>(const int i)
{
  return _mm_set1_epi32(i);
}
template<typename T> static ALWAYSINLINE T _mmauto_set1_epi16(const short i);
template<> ALWAYSINLINE __m128i _mmauto_set1_epi16<>(const short i)
{
  return _mm_set1_epi16(i);
}
template<typename T> static ALWAYSINLINE T _mmauto_set1_epi8(const char i);
template<> ALWAYSINLINE __m128i _mmauto_set1_epi8<>(const char i)
{
  return _mm_set1_epi8(i);
}
template<typename T> static ALWAYSINLINE T _mmauto_setzero_si_all();
template<> ALWAYSINLINE __m128i _mmauto_setzero_si_all<>()
{
  return _mm_setzero_si128();
}
template<typename T> static ALWAYSINLINE T _mmauto_setr128_epi8(const char a0, const char a1, const char a2, const char a3, const char a4, const char a5, const char a6, const char a7, const char a8, const char a9, const char a10, const char a11, const char a12, const char a13, const char a14, const char a15);
template<> ALWAYSINLINE __m128i _mmauto_setr128_epi8<>(const char a0, const char a1, const char a2, const char a3, const char a4, const char a5, const char a6, const char a7, const char a8, const char a9, const char a10, const char a11, const char a12, const char a13, const char a14, const char a15)
{
  return _mm_setr_epi8(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15);
}
template<typename T> static ALWAYSINLINE T _mmauto_set_as_posible_epi8(const char a0, const char a1, const char a2, const char a3, const char a4, const char a5, const char a6, const char a7, const char a8, const char a9, const char a10, const char a11, const char a12, const char a13, const char a14, const char a15, const char a16, const char a17, const char a18, const char a19, const char a20, const char a21, const char a22, const char a23, const char a24, const char a25, const char a26, const char a27, const char a28, const char a29, const char a30, const char a31);
template<> ALWAYSINLINE __m128i _mmauto_set_as_posible_epi8<>(const char a0, const char a1, const char a2, const char a3, const char a4, const char a5, const char a6, const char a7, const char a8, const char a9, const char a10, const char a11, const char a12, const char a13, const char a14, const char a15, const char a16, const char a17, const char a18, const char a19, const char a20, const char a21, const char a22, const char a23, const char a24, const char a25, const char a26, const char a27, const char a28, const char a29, const char a30, const char a31)
{
  return _mm_set_epi8(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15);
}

DEFINE_FUNCTIONI_MM(_mmauto_and_si_all, _mm_and_si128, _mm256_and_si256)
DEFINE_FUNCTIONI_MM(_mmauto_andnot_si_all, _mm_andnot_si128, _mm256_andnot_si256)
DEFINE_FUNCTIONI_MM(_mmauto_or_si_all, _mm_or_si128, _mm256_or_si256)
DEFINE_FUNCTIONI_MM(_mmauto_xor_si_all, _mm_xor_si128, _mm256_xor_si256)
DEFINE_FUNCTIONI_MM(_mmauto_cmpeq_epi8, _mm_cmpeq_epi8, _mm256_cmpeq_epi8)
DEFINE_FUNCTIONI_MM(_mmauto_cmpgt_epi8, _mm_cmpgt_epi8, _mm256_cmpgt_epi8)
DEFINE_FUNCTIONI_MM(_mmauto_cmpeq_epi16, _mm_cmpeq_epi16, _mm256_cmpeq_epi16)
DEFINE_FUNCTIONI_MM(_mmauto_cmpgt_epi16, _mm_cmpgt_epi16, _mm256_cmpgt_epi16)
DEFINE_FUNCTIONI_MM(_mmauto_cmpeq_epi32, _mm_cmpeq_epi32, _mm256_cmpeq_epi32)
DEFINE_FUNCTIONI_MM(_mmauto_cmpgt_epi32, _mm_cmpgt_epi32, _mm256_cmpgt_epi32)
DEFINE_FUNCTIONI_MM(_mmauto_max_epu8, _mm_max_epu8, _mm256_max_epu8)
DEFINE_FUNCTIONI_MM(_mmauto_min_epu8, _mm_min_epu8, _mm256_min_epu8)
DEFINE_FUNCTIONI_MM(_mmauto_max_epi16, _mm_max_epi16, _mm256_max_epi16)
DEFINE_FUNCTIONI_MM(_mmauto_min_epi16, _mm_min_epi16, _mm256_min_epi16)

DEFINE_FUNCTIONI_MM(_mmauto_sign_epi8, _mm_sign_epi8, _mm256_sign_epi8)
DEFINE_FUNCTIONI_MM(_mmauto_sign_epi16, _mm_sign_epi16, _mm256_sign_epi16)

DEFINE_FUNCTIONI_MI(_mmauto_slli_si_all, _mm_slli_si128, _mm256_slli_si256)
#define _mmauto_slli_si_all(a, i) _mmauto_slli_si_all<i>(a) /** ATTENTION: THIS BEHAVES DIFFERENTLY ON 128 AND 256 */
DEFINE_FUNCTIONI_MI(_mmauto_srli_si_all, _mm_srli_si128, _mm256_srli_si256)
#define _mmauto_srli_si_all(a, i) _mmauto_srli_si_all<i>(a) /** ATTENTION: THIS BEHAVES DIFFERENTLY ON 128 AND 256 */
DEFINE_FUNCTIONI_MI(_mmauto_srli_epi8, _mm_srli_epi8, _mm256_srli_epi8)
#define _mmauto_srli_epi8(a, i) _mmauto_srli_epi8<i>(a)
DEFINE_FUNCTIONI_MI(_mmauto_slli_epi8, _mm_slli_epi8, _mm256_slli_epi8)
#define _mmauto_slli_epi8(a, i) _mmauto_slli_epi8<i>(a)
DEFINE_FUNCTIONI_MI(_mmauto_srli_epi16, _mm_srli_epi16, _mm256_srli_epi16)
#define _mmauto_srli_epi16(a, i) _mmauto_srli_epi16<i>(a)
DEFINE_FUNCTIONI_MI(_mmauto_srai_epi16, _mm_srai_epi16, _mm256_srai_epi16)
#define _mmauto_srai_epi16(a, i) _mmauto_srai_epi16<i>(a)
DEFINE_FUNCTIONI_MI(_mmauto_slli_epi16, _mm_slli_epi16, _mm256_slli_epi16)
#define _mmauto_slli_epi16(a, i) _mmauto_slli_epi16<i>(a)
DEFINE_FUNCTIONI_MI(_mmauto_srli_epi32, _mm_srli_epi32, _mm256_srli_epi32)
#define _mmauto_srli_epi32(a, i) _mmauto_srli_epi32<i>(a)
DEFINE_FUNCTIONI_MI(_mmauto_slli_epi32, _mm_slli_epi32, _mm256_slli_epi32)
#define _mmauto_slli_epi32(a, i) _mmauto_slli_epi32<i>(a)
DEFINE_FUNCTIONI_MI(_mmauto_srai_epi32, _mm_srai_epi32, _mm256_srai_epi32)
#define _mmauto_srai_epi32(a, i) _mmauto_srai_epi32<i>(a)
DEFINE_FUNCTIONI_MI(_mmauto_srli_epi64, _mm_srli_epi64, _mm256_srli_epi64)
#define _mmauto_srli_epi64(a, i) _mmauto_srli_epi64<i>(a)
DEFINE_FUNCTIONI_MI(_mmauto_slli_epi64, _mm_slli_epi64, _mm256_slli_epi64)
#define _mmauto_slli_epi64(a, i) _mmauto_slli_epi64<i>(a)

DEFINE_FUNCTIONI_MM(_mmauto_add_epi8, _mm_add_epi8, _mm256_add_epi8)
DEFINE_FUNCTIONI_MM(_mmauto_adds_epu8, _mm_adds_epu8, _mm256_adds_epu8)
DEFINE_FUNCTIONI_MM(_mmauto_add_epi16, _mm_add_epi16, _mm256_add_epi16)
DEFINE_FUNCTIONI_MM(_mmauto_adds_epi16, _mm_adds_epi16, _mm256_adds_epi16)
DEFINE_FUNCTIONI_MM(_mmauto_adds_epu16, _mm_adds_epu16, _mm256_adds_epu16)
DEFINE_FUNCTIONI_MM(_mmauto_add_epi32, _mm_add_epi32, _mm256_add_epi32)
DEFINE_FUNCTIONI_MM(_mmauto_add_epi64, _mm_add_epi64, _mm256_add_epi64)
DEFINE_FUNCTIONI_MM(_mmauto_subs_epu8, _mm_subs_epu8, _mm256_subs_epu8)
DEFINE_FUNCTIONI_MM(_mmauto_sub_epi8, _mm_sub_epi8, _mm256_sub_epi8)
DEFINE_FUNCTIONI_MM(_mmauto_sub_epi16, _mm_sub_epi16, _mm256_sub_epi16)
DEFINE_FUNCTIONI_MM(_mmauto_subs_epu16, _mm_subs_epu16, _mm256_subs_epu16)
DEFINE_FUNCTIONI_MM(_mmauto_sub_epi32, _mm_sub_epi32, _mm256_sub_epi32)
DEFINE_FUNCTIONI_MM(_mmauto_mulhi_epi16, _mm_mulhi_epi16, _mm256_mulhi_epi16)
DEFINE_FUNCTIONI_MM(_mmauto_mulhi_epu16, _mm_mulhi_epu16, _mm256_mulhi_epu16)
DEFINE_FUNCTIONI_MM(_mmauto_mullo_epi16, _mm_mullo_epi16, _mm256_mullo_epi16)
DEFINE_FUNCTIONI_MM(_mmauto_madd_epi16, _mm_madd_epi16, _mm256_madd_epi16)
DEFINE_FUNCTIONI_MM(_mmauto_maddubs_epi16, _mm_maddubs_epi16, _mm256_maddubs_epi16)
DEFINE_FUNCTIONI_MM(_mmauto_mulhrs_epi16, _mm_mulhrs_epi16, _mm256_mulhrs_epi16)
DEFINE_FUNCTIONI_MM(_mmauto_mul_epu32, _mm_mul_epu32, _mm256_mul_epu32)
DEFINE_FUNCTIONI_MM(_mmauto_avg_epu8, _mm_avg_epu8, _mm256_avg_epu8)
DEFINE_FUNCTIONI_MM(_mmauto_avg_epu16, _mm_avg_epu16, _mm256_avg_epu16)

DEFINE_FUNCTIONI_M(_mmauto_abs_epi8, _mm_abs_epi8, _mm256_abs_epi8)
DEFINE_FUNCTIONI_M(_mmauto_abs_epi16, _mm_abs_epi16, _mm256_abs_epi16)
DEFINE_FUNCTIONI_M(_mmauto_abs_epi32, _mm_abs_epi32, _mm256_abs_epi32)

DEFINE_FUNCTIONI_MM(_mmauto_hadd_epi16, _mm_hadd_epi16, _mm256_hadd_epi16) /** ATTENTION: THIS BEHAVES DIFFERENTLY ON 128 AND 256 */
DEFINE_FUNCTIONI_MM(_mmauto_hadds_epi16, _mm_hadds_epi16, _mm256_hadds_epi16)
DEFINE_FUNCTIONI_MM(_mmauto_hadd_epi32, _mm_hadd_epi32, _mm256_hadd_epi32) /** ATTENTION: THIS BEHAVES DIFFERENTLY ON 128 AND 256 */

DEFINE_FUNCTIONI_MM(_mmauto_packs_epi16, _mm_packs_epi16, _mm256_packs_epi16) /** ATTENTION: THIS BEHAVES DIFFERENTLY ON 128 AND 256 */
DEFINE_FUNCTIONI_MM(_mmauto_packus_epi16, _mm_packus_epi16, _mm256_packus_epi16) /** ATTENTION: THIS BEHAVES DIFFERENTLY ON 128 AND 256 */
DEFINE_FUNCTIONI_MM(_mmauto_packs_epi32, _mm_packs_epi32, _mm256_packs_epi32) /** ATTENTION: THIS BEHAVES DIFFERENTLY ON 128 AND 256 */
DEFINE_FUNCTIONI_MM(_mmauto_unpacklo_epi8, _mm_unpacklo_epi8, _mm256_unpacklo_epi8) /** ATTENTION: THIS BEHAVES DIFFERENTLY ON 128 AND 256 */
DEFINE_FUNCTIONI_MM(_mmauto_unpackhi_epi8, _mm_unpackhi_epi8, _mm256_unpackhi_epi8) /** ATTENTION: THIS BEHAVES DIFFERENTLY ON 128 AND 256 */
DEFINE_FUNCTIONI_MM(_mmauto_unpacklo_epi16, _mm_unpacklo_epi16, _mm256_unpacklo_epi16) /** ATTENTION: THIS BEHAVES DIFFERENTLY ON 128 AND 256 */
DEFINE_FUNCTIONI_MM(_mmauto_unpackhi_epi16, _mm_unpackhi_epi16, _mm256_unpackhi_epi16) /** ATTENTION: THIS BEHAVES DIFFERENTLY ON 128 AND 256 */
DEFINE_FUNCTIONI_MM(_mmauto_unpacklo_epi32, _mm_unpacklo_epi32, _mm256_unpacklo_epi32) /** ATTENTION: THIS BEHAVES DIFFERENTLY ON 128 AND 256 */
DEFINE_FUNCTIONI_MM(_mmauto_unpackhi_epi32, _mm_unpackhi_epi32, _mm256_unpackhi_epi32) /** ATTENTION: THIS BEHAVES DIFFERENTLY ON 128 AND 256 */
DEFINE_FUNCTION_MM(_mmauto_unpacklo_ps, _mm_unpacklo_ps, _mm256_unpacklo_ps) /** ATTENTION: THIS BEHAVES DIFFERENTLY ON 128 AND 256 */
DEFINE_FUNCTION_MM(_mmauto_unpackhi_ps, _mm_unpackhi_ps, _mm256_unpackhi_ps) /** ATTENTION: THIS BEHAVES DIFFERENTLY ON 128 AND 256 */
DEFINE_FUNCTIONI_MM(_mmauto_shuffle_epi8, _mm_shuffle_epi8, _mm256_shuffle_epi8) /** ATTENTION: THIS BEHAVES DIFFERENTLY ON 128 AND 256 */
DEFINE_FUNCTIONI_MMI(_mmauto_alignr_epi8, _mm_alignr_epi8, _mm256_alignr_epi8)
#define _mmauto_alignr_epi8(a, b, i) _mmauto_alignr_epi8<i>(a, b) /** ATTENTION: THIS BEHAVES DIFFERENTLY ON 128 AND 256 */

DEFINE_FUNCTION_MM(_mmauto_add_ps, _mm_add_ps, _mm256_add_ps)
DEFINE_FUNCTION_MM(_mmauto_hadd_ps, _mm_hadd_ps, _mm256_hadd_ps) /** ATTENTION: THIS BEHAVES DIFFERENTLY ON 128 AND 256 */
DEFINE_FUNCTION_MM(_mmauto_sub_ps, _mm_sub_ps, _mm256_sub_ps)
DEFINE_FUNCTION_MM(_mmauto_mul_ps, _mm_mul_ps, _mm256_mul_ps)
DEFINE_FUNCTION_MM(_mmauto_div_ps, _mm_div_ps, _mm256_div_ps)
DEFINE_FUNCTION_M(_mmauto_rcp_ps, _mm_rcp_ps, _mm256_rcp_ps)
DEFINE_FUNCTION_M(_mmauto_sqrt_ps, _mm_sqrt_ps, _mm256_sqrt_ps)
DEFINE_FUNCTION_M(_mmauto_rsqrt_ps, _mm_rsqrt_ps, _mm256_rsqrt_ps)

static ALWAYSINLINE __m128i _mmauto_cvtps_epi32(const __m128 a) { return _mm_cvtps_epi32(a); }
static ALWAYSINLINE __m128 _mmauto_cvtepi32_ps(const __m128i a) { return _mm_cvtepi32_ps(a); }

static ALWAYSINLINE __m128i _mmauto_castsiauto_si128(const __m128i a) { return a; }

static ALWAYSINLINE __m128i _mmauto_mullo_epi32(const __m128i a, const __m128i b)
{
  return _mm_shuffle_epi32(
           _mm_castps_si128(_mm_shuffle_ps(_mm_castsi128_ps(_mm_mul_epu32(a, b)), _mm_castsi128_ps(_mm_mul_epu32(_mm_shuffle_epi32(a, 1 | (3 << 4) | (2 << 6)), _mm_shuffle_epi32(b, 1 | (3 << 4) | (2 << 6)))), (2 << 6 | 2 << 2))),
           (2 << 2) | (1 << 4) | (3 << 6)
         );
}

static ALWAYSINLINE int _mmauto_movemask_epi8(const __m128i a) { return _mm_movemask_epi8(a); }

template<int imm> static ALWAYSINLINE __m128i _mmauto_permute2x128_si256(const __m128i a, const __m128i b) { return imm == 0 ? a : (imm == 2 ? b : _mm_setzero_si128()); } /** Make sure to not willingly use this function; this is just to silence the compiler as there is no static if in C++. */

template<bool aligned> static ALWAYSINLINE __m128i _mmauto_loadt_si_all(__m128i const* src)
{
  return _mm_loadt_si128<aligned>(src);
}

template<bool aligned> static ALWAYSINLINE void _mmauto_storet_si_all(__m128i* dest, const __m128i val)
{
  _mm_storet_si128<aligned>(dest, val);
}

template<bool aligned> static ALWAYSINLINE void _mmauto_streamt_si_all(__m128i* dest, const __m128i val)
{
  if(aligned)
    _mm_stream_si128(dest, val);
  else
    _mm_storeu_si128(dest, val);
}

static ALWAYSINLINE __m128i _mmauto_correct_256op(const __m128i a)
{
  return a;
}

static ALWAYSINLINE void _mmauto_unpacklohi_epi8(__m128i& a, __m128i& b)
{
  const __m128i tmp = _mm_unpacklo_epi8(a, b);
  b = _mm_unpackhi_epi8(a, b);
  a = tmp;
}

static ALWAYSINLINE void _mmauto_unpacklohi_epi16(__m128i& a, __m128i& b)
{
  const __m128i tmp = _mm_unpacklo_epi16(a, b);
  b = _mm_unpackhi_epi16(a, b);
  a = tmp;
}

static ALWAYSINLINE void _mmauto_unpacklohi_epi32(__m128i& a, __m128i& b)
{
  const __m128i tmp = _mm_unpacklo_epi32(a, b);
  b = _mm_unpackhi_epi32(a, b);
  a = tmp;
}

static ALWAYSINLINE void _mmauto_unpacklohi_ps(__m128& a, __m128& b)
{
  const __m128 tmp = _mm_unpacklo_ps(a, b);
  b = _mm_unpackhi_ps(a, b);
  a = tmp;
}

#ifndef DOES_DEFINITELY_NOT_SUPPORT_AVX2
template<> ALWAYSINLINE __m256 _mmauto_set1_ps<>(const float f)
{
  return _mm256_set1_ps(f);
}
template<> ALWAYSINLINE __m256i _mmauto_set1_epi32<>(const int i)
{
  return _mm256_set1_epi32(i);
}
template<> ALWAYSINLINE __m256i _mmauto_set1_epi16<>(const short i)
{
  return _mm256_set1_epi16(i);
}
template<> ALWAYSINLINE __m256i _mmauto_set1_epi8<>(const char i)
{
  return _mm256_set1_epi8(i);
}
template<> ALWAYSINLINE __m256i _mmauto_setzero_si_all<>()
{
  return _mm256_setzero_si256();
}
template<> ALWAYSINLINE __m256i _mmauto_setr128_epi8<>(const char a0, const char a1, const char a2, const char a3, const char a4, const char a5, const char a6, const char a7, const char a8, const char a9, const char a10, const char a11, const char a12, const char a13, const char a14, const char a15)
{
  return _mm256_setr_epi8(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15);
}
template<> ALWAYSINLINE __m256i _mmauto_set_as_posible_epi8<>(const char a0, const char a1, const char a2, const char a3, const char a4, const char a5, const char a6, const char a7, const char a8, const char a9, const char a10, const char a11, const char a12, const char a13, const char a14, const char a15, const char a16, const char a17, const char a18, const char a19, const char a20, const char a21, const char a22, const char a23, const char a24, const char a25, const char a26, const char a27, const char a28, const char a29, const char a30, const char a31)
{
  return _mm256_set_epi8(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31);
}

static ALWAYSINLINE __m256i _mmauto_cvtps_epi32(const __m256 a) { return _mm256_cvtps_epi32(a); }
static ALWAYSINLINE __m256 _mmauto_cvtepi32_ps(const __m256i a) { return _mm256_cvtepi32_ps(a); }

static ALWAYSINLINE __m128i _mmauto_castsiauto_si128(const __m256i a) { return _mm256_castsi256_si128(a); }

template<bool aligned> static ALWAYSINLINE __m256i _mmauto_loadt_si_all(__m256i const* src)
{
  return aligned ? _mm256_load_si256(src) : _mm256_loadu_si256(src);
}

template<bool aligned> static ALWAYSINLINE void _mmauto_storet_si_all(__m256i* dest, const __m256i val)
{
  if(aligned)
    _mm256_store_si256(dest, val);
  else
    _mm256_storeu_si256(dest, val);
}

template<bool aligned> static ALWAYSINLINE void _mmauto_streamt_si_all(__m256i* dest, const __m256i val)
{
  if(aligned)
    _mm256_stream_si256(dest, val);
  else
    _mm256_storeu_si256(dest, val);
}

static ALWAYSINLINE __m256i _mmauto_correct_256op(const __m256i a)
{
  return _mm256_permute4x64_epi64(a, 0xD8);
}

static ALWAYSINLINE __m256i _mmauto_mullo_epi32(const __m256i a, const __m256i b)
{
  return _mm256_mullo_epi32(a, b);
}

static ALWAYSINLINE int _mmauto_movemask_epi8(const __m256i a) { return _mm256_movemask_epi8(a); }

template<int imm> static ALWAYSINLINE __m256i _mmauto_permute2x128_si256(const __m256i a, const __m256i b) { return _mm256_permute2x128_si256(a, b, imm); }

static ALWAYSINLINE void _mmauto_unpacklohi_epi8(__m256i& a, __m256i& b)
{
  const __m256i lo = _mm256_unpacklo_epi8(a, b);
  const __m256i hi = _mm256_unpackhi_epi8(a, b);
  a = _mm256_permute2x128_si256(lo, hi, 2 << 4);
  b = _mm256_permute2x128_si256(lo, hi, 1 | (3 << 4));
}

static ALWAYSINLINE void _mmauto_unpacklohi_epi16(__m256i& a, __m256i& b)
{
  const __m256i lo = _mm256_unpacklo_epi16(a, b);
  const __m256i hi = _mm256_unpackhi_epi16(a, b);
  a = _mm256_permute2x128_si256(lo, hi, 2 << 4);
  b = _mm256_permute2x128_si256(lo, hi, 1 | (3 << 4));
}

static ALWAYSINLINE void _mmauto_unpacklohi_epi32(__m256i& a, __m256i& b)
{
  const __m256i lo = _mm256_unpacklo_epi32(a, b);
  const __m256i hi = _mm256_unpackhi_epi32(a, b);
  a = _mm256_permute2x128_si256(lo, hi, 2 << 4);
  b = _mm256_permute2x128_si256(lo, hi, 1 | (3 << 4));
}

static ALWAYSINLINE void _mmauto_unpacklohi_ps(__m256& a, __m256& b)
{
  const __m256 lo = _mm256_unpacklo_ps(a, b);
  const __m256 hi = _mm256_unpackhi_ps(a, b);
  a = _mm256_permute2f128_ps(lo, hi, 2 << 4);
  b = _mm256_permute2f128_ps(lo, hi, 1 | (3 << 4));
}
#endif

#define _mmauto_set1_ps_using(avx, f) _mmauto_set1_ps<__m_auto_using(avx)>(f)
#define _mmauto_set1_epi32_using(avx, i) _mmauto_set1_epi32<__m_auto_i_using(avx)>(i)
#define _mmauto_set1_epi16_using(avx, i) _mmauto_set1_epi16<__m_auto_i_using(avx)>(i)
#define _mmauto_set1_epi8_using(avx, i) _mmauto_set1_epi8<__m_auto_i_using(avx)>(i)
#define _mmauto_setzero_si_all_using(avx) _mmauto_setzero_si_all<__m_auto_i_using(avx)>()
#define _mmauto_setr128_epi8_using(avx, a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15) _mmauto_setr128_epi8<__m_auto_i_using(avx)>(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15)
#define _mmauto_set_as_posible_epi8_using(avx, a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31) _mmauto_set_as_posible_epi8<__m_auto_i_using(avx)>(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31)

#define _mmauto_permute2x128_si256(a, b, i) _mmauto_permute2x128_si256<i>(a, b)

/**
 * Shortcuts that ONLY work if there is a _compile-time_ constant (most likely
 * a template parameter) named avx in the current scope which is true iff the
 * system supports AVX2 instructions.
 */
#define __m_auto_i __m_auto_i_using(avx)
#define __m_auto __m_auto_using(avx)
#define _mmauto_set1_ps(f) _mmauto_set1_ps_using(avx, f)
#define _mmauto_set1_epi32(i) _mmauto_set1_epi32_using(avx, i)
#define _mmauto_set1_epi16(i) _mmauto_set1_epi16_using(avx, i)
#define _mmauto_set1_epi8(i) _mmauto_set1_epi8_using(avx, i)
#define _mmauto_setzero_si_all() _mmauto_setzero_si_all_using(avx)
#define _mmauto_setr128_epi8(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15) _mmauto_setr128_epi8_using(avx, a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15)
#define _mmauto_set_as_posible_epi8(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31) _mmauto_set_as_posible_epi8_using(avx, a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31)

/**
 * Utility function for dividing unsigned 8-bit values.
 * (inspired by https://en.wikipedia.org/wiki/Division_algorithm#Integer_division_.28unsigned.29_with_remainder)
 *
 * @param n nominator in range [0..255]
 * @param d denominator in range [n..255]
 * @return (n << 8) / d
 */
template<bool avx> static ALWAYSINLINE __m_auto_i _mmauto_divq8_epu8(const __m_auto_i n, const __m_auto_i d)
{
  static const __m_auto_i c_0 = _mmauto_setzero_si_all();
  __m_auto_i q = c_0;
  __m_auto_i r = n;
  __m_auto_i tally = _mmauto_set1_epi8(char(1 << 7));

  for(size_t i = 0; i < 7; i++)
  {
    const __m_auto_i rGtD = _mmauto_cmpeq_epi8(c_0, _mmauto_subs_epu8(d, _mmauto_adds_epu8(r, r)));
    q = _mmauto_or_si_all(q, _mmauto_and_si_all(rGtD, tally));
    r = _mmauto_sub_epi8(_mmauto_add_epi8(r, r), _mmauto_and_si_all(rGtD, d));
    tally = _mmauto_avg_epu8(tally, c_0);
  }

  return _mmauto_andnot_si_all(
           _mmauto_cmpeq_epi8(d, c_0), // Division by zero yields zero
           _mmauto_or_si_all(
             _mmauto_cmpeq_epi8(_mmauto_subs_epu8(d, n), c_0), // Saturate to 255 if n >= d
             _mmauto_or_si_all(q, _mmauto_and_si_all(_mmauto_cmpeq_epi8(c_0, _mmauto_subs_epu8(d, _mmauto_adds_epu8(r, r))), tally))
           )
         );
}

/**
 * Utility function for approximately dividing unsigned 8-bit values in 16-bit registers.
 *
 * @param min nominator in range [0..255]
 * @param max denominator in range [n..255]
 * @return (n << 15) / d
 */
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

/**
 * Utility function for computing the square roots of unsigned 8-bit values.
 * (SIMD version of https://en.wikipedia.org/wiki/Methods_of_computing_square_roots#Binary_numeral_system_.28base_2.29)
 *
 * @param num value
 * @return isqrt(num)
 */
template<bool avx> static ALWAYSINLINE __m_auto_i _mmauto_sqrt_epu8(const __m_auto_i num)
{
  static const __m_auto_i c_0 = _mmauto_setzero_si_all();
  __m_auto_i rest = num;
  __m_auto_i res = c_0;
  __m_auto_i bit = _mmauto_set1_epi8(1 << 6);

  for(size_t i = 0; i < 2; i++)
  {
    const __m_auto_i resPlusBit = _mmauto_add_epi8(res, bit);
    const __m_auto_i numGeResPlusBit = _mmauto_cmpeq_epi8(c_0, _mmauto_subs_epu8(resPlusBit, rest));
    rest = _mmauto_sub_epi8(rest, _mmauto_and_si_all(numGeResPlusBit, resPlusBit));
    res = _mmauto_add_epi8(_mmauto_avg_epu8(res, c_0), _mmauto_and_si_all(numGeResPlusBit, bit));
    bit = _mmauto_avg_epu8(_mmauto_avg_epu8(bit, c_0), c_0);
  }

  return _mmauto_add_epi8(_mmauto_avg_epu8(res, c_0), _mmauto_and_si_all(_mmauto_cmpeq_epi8(c_0, _mmauto_subs_epu8(_mmauto_add_epi8(res, bit), rest)), bit));
}

/**
 * Utility function for computing the square roots of unsigned 16-bit values.
 * (SIMD version of https://en.wikipedia.org/wiki/Methods_of_computing_square_roots#Binary_numeral_system_.28base_2.29)
 *
 * @param num value
 * @return isqrt(num)
 */
template<bool avx> static ALWAYSINLINE __m_auto_i _mmauto_sqrt_epu16(const __m_auto_i num)
{
  static const __m_auto_i c_0 = _mmauto_setzero_si_all();
  __m_auto_i rest = num;
  __m_auto_i res = c_0;
  __m_auto_i bit = _mmauto_set1_epi16(1 << 14);

  for(size_t i = 0; i < 6; i++)
  {
    const __m_auto_i resPlusBit = _mmauto_add_epi16(res, bit);
    const __m_auto_i numGeResPlusBit = _mmauto_cmpeq_epi16(c_0, _mmauto_subs_epu16(resPlusBit, rest));
    rest = _mmauto_sub_epi16(rest, _mmauto_and_si_all(numGeResPlusBit, resPlusBit));
    res = _mmauto_add_epi16(_mmauto_srli_epi16(res, 1), _mmauto_and_si_all(numGeResPlusBit, bit));
    bit = _mmauto_srli_epi16(bit, 2);
  }

  return _mmauto_add_epi16(_mmauto_srli_epi16(res, 1), _mmauto_and_si_all(_mmauto_cmpeq_epi16(c_0, _mmauto_subs_epu16(_mmauto_add_epi16(res, bit), rest)), bit));
}

/**
 * Utility function for computing the square roots of unsigned 16-bit values and storing them as 8-bit values.
 * (SIMD version of https://en.wikipedia.org/wiki/Methods_of_computing_square_roots#Binary_numeral_system_.28base_2.29)
 *
 * @param num0 first value
 * @param num1 second value
 * @return pack(isqrt(num0), isqrt(num1))
 */
template<bool avx> static ALWAYSINLINE __m_auto_i _mmauto_sqrt16_epu8(const __m_auto_i num0, const __m_auto_i num1)
{
  static const __m_auto_i c_0 = _mmauto_setzero_si_all();
  __m_auto_i rest0 = num0;
  __m_auto_i rest1 = num1;
  __m_auto_i res0 = c_0;
  __m_auto_i res1 = c_0;
  __m_auto_i bit = _mmauto_set1_epi16(1 << 14);

  for(size_t i = 0; i < 6; i++)
  {
    const __m_auto_i resPlusBit0 = _mmauto_add_epi16(res0, bit);
    const __m_auto_i numGeResPlusBit0 = _mmauto_cmpeq_epi16(c_0, _mmauto_subs_epu16(resPlusBit0, rest0));
    rest0 = _mmauto_sub_epi16(rest0, _mmauto_and_si_all(numGeResPlusBit0, resPlusBit0));
    res0 = _mmauto_add_epi16(_mmauto_srli_epi16(res0, 1), _mmauto_and_si_all(numGeResPlusBit0, bit));

    const __m_auto_i resPlusBit1 = _mmauto_add_epi16(res1, bit);
    const __m_auto_i numGeResPlusBit1 = _mmauto_cmpeq_epi16(c_0, _mmauto_subs_epu16(resPlusBit1, rest1));
    rest1 = _mmauto_sub_epi16(rest1, _mmauto_and_si_all(numGeResPlusBit1, resPlusBit1));
    res1 = _mmauto_add_epi16(_mmauto_srli_epi16(res1, 1), _mmauto_and_si_all(numGeResPlusBit1, bit));

    bit = _mmauto_srli_epi16(bit, 2);
  }

  return _mmauto_correct_256op(
           _mmauto_adds_epu8(
             _mmauto_packus_epi16(_mmauto_srli_epi16(res0, 1), _mmauto_srli_epi16(res1, 1)),
             _mmauto_and_si_all(_mmauto_packus_epi16(bit, bit), _mmauto_cmpeq_epi8(c_0, _mmauto_packus_epi16(_mmauto_subs_epu16(_mmauto_add_epi16(res0, bit), rest0), _mmauto_subs_epu16(_mmauto_add_epi16(res1, bit), rest1))))
           )
         );
}

/**
 * Utility function for computing the norms of pairs of signed 8-bit values,
 * scaling them to range [0..255] and storing them as unsigned 8-bit values.
 *
 * @param num0 first packed pairs of values
 * @param num1 second packed pairs of values
 * @return pack(isqrt(2*hadd(num0*num0)), isqrt(2*hadd(num1+num1)))
 */
template<bool avx> static ALWAYSINLINE __m_auto_i _mmauto_norm8_epi8(const __m_auto_i num0, const __m_auto_i num1)
{
  const __m_auto_i factor0 = _mmauto_abs_epi8(num0);
  const __m_auto_i factor1 = _mmauto_abs_epi8(num1);
  return _mmauto_sqrt16_epu8<avx>(
           _mmauto_slli_epi16(_mmauto_maddubs_epi16(factor0, factor0), 1),
           _mmauto_slli_epi16(_mmauto_maddubs_epi16(factor1, factor1), 1)
         );
}

/**
 * Utility function for computing the atan2 of signed 8-bit values.
 * Based on the Q15-implementation from http://geekshavefeelings.com/posts/fixed-point-atan2.
 *
 * @param y vector of y values
 * @param x vector of x values
 * @param absY absolute of y
 * @param absX absolute of x
 * @return atan2(y,x) in range [0,255]
 */
template<bool avx> static ALWAYSINLINE __m_auto_i _mmauto_atan2_epi8(const __m_auto_i y, const __m_auto_i x, const __m_auto_i absY, const __m_auto_i absX)
{
  static const __m_auto_i c_0 = _mmauto_setzero_si_all();
  static const __m_auto_i c_64 = _mmauto_set1_epi8(64);
  static const __m_auto_i c_128 = _mmauto_set1_epi8(char(128));
  static const __m_auto_i c_129 = _mmauto_set1_epi8(char(0x81));
  static const __m_auto_i c_5695 = _mmauto_set1_epi16(5695);
  static const __m_auto_i c_11039 = _mmauto_set1_epi16(11039);

  const __m_auto_i min = _mmauto_min_epu8(absX, absY);

  __m_auto_i min0 = min;
  __m_auto_i min1 = c_0;
  _mmauto_unpacklohi_epi8(min0, min1);
  __m_auto_i max0 = _mmauto_max_epu8(absX, absY);
  __m_auto_i max1 = c_0;
  _mmauto_unpacklohi_epi8(max0, max1);
  const __m_auto_i quotient0 = _mmauto_div8_epi16<avx>(min0, max0);
  const __m_auto_i quotient1 = _mmauto_div8_epi16<avx>(min1, max1);

  const __m_auto_i absUnrotatedAtan2 = _mmauto_correct_256op(
                                         _mmauto_packs_epi16(
                                             _mmauto_mulhrs_epi16(_mmauto_sub_epi16(c_11039, _mmauto_mulhrs_epi16(c_5695, quotient0)), quotient0),
                                             _mmauto_mulhrs_epi16(_mmauto_sub_epi16(c_11039, _mmauto_mulhrs_epi16(c_5695, quotient1)), quotient1)
                                         )
                                       );
  const __m_auto_i xGtY = _mmauto_cmpeq_epi8(min, absY);
  return _mmauto_add_epi8(
           _mmauto_or_si_all(
             _mmauto_and_si_all(
               xGtY,
               _mmauto_and_si_all(x, c_128)
             ),
             _mmauto_andnot_si_all(
               xGtY,
               _mmauto_or_si_all(
                 c_64,
                 _mmauto_and_si_all(y, c_128)
               )
             )
           ),
           _mmauto_sign_epi8(
             absUnrotatedAtan2,
             _mmauto_sign_epi8(_mmauto_xor_si_all(xGtY, c_129), _mmauto_sign_epi8(x, y))
           )
         );
}

/**
 * Utility function for computing the atan2 of signed 8-bit values.
 * Based on the Q15-implementation from http://geekshavefeelings.com/posts/fixed-point-atan2.
 *
 * @param y vector of y values
 * @param x vector of x values
 * @return atan2(y,x) in range [0,255]
 */
template<bool avx> static ALWAYSINLINE __m_auto_i _mmauto_atan2_epi8(const __m_auto_i y, const __m_auto_i x)
{
  return _mmauto_atan2_epi8<avx>(y, x, _mmauto_abs_epi8(y), _mmauto_abs_epi8(x));
}
