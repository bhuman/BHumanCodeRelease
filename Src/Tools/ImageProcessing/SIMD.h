/**
 * @file SIMD.h
 *
 * This file includes SSE intrinsics up to SSSE3 and provides some macros to emulate
 * SSSE3 intrinsics on systems that do not support them natively.
 *
 * @author Alexis Tsogias
 * @author Felix Thielke
 */

#pragma once

#ifdef _MSC_VER
#include <intrin.h>
#define ALWAYSINLINE __forceinline
#else
#include <x86intrin.h>
#define ALWAYSINLINE inline __attribute((always_inline))
#endif

/**
 * Workarounds for clang-3.7.
 */
#if defined __clang__ && defined __has_builtin
#if __has_builtin(__builtin_shufflevector) && !__has_builtin(__builtin_ia32_pslldqi128)
#undef _mm_slli_si128
#define _mm_slli_si128(a, imm) __extension__ ({                         \
    (__m128i)__builtin_shufflevector((__v16qi)_mm_setzero_si128(),        \
                                     (__v16qi)(__m128i)(a),               \
                                     ((imm)&0xF0) ? 0 : 16 - ((imm)&0xF), \
                                     ((imm)&0xF0) ? 0 : 17 - ((imm)&0xF), \
                                     ((imm)&0xF0) ? 0 : 18 - ((imm)&0xF), \
                                     ((imm)&0xF0) ? 0 : 19 - ((imm)&0xF), \
                                     ((imm)&0xF0) ? 0 : 20 - ((imm)&0xF), \
                                     ((imm)&0xF0) ? 0 : 21 - ((imm)&0xF), \
                                     ((imm)&0xF0) ? 0 : 22 - ((imm)&0xF), \
                                     ((imm)&0xF0) ? 0 : 23 - ((imm)&0xF), \
                                     ((imm)&0xF0) ? 0 : 24 - ((imm)&0xF), \
                                     ((imm)&0xF0) ? 0 : 25 - ((imm)&0xF), \
                                     ((imm)&0xF0) ? 0 : 26 - ((imm)&0xF), \
                                     ((imm)&0xF0) ? 0 : 27 - ((imm)&0xF), \
                                     ((imm)&0xF0) ? 0 : 28 - ((imm)&0xF), \
                                     ((imm)&0xF0) ? 0 : 29 - ((imm)&0xF), \
                                     ((imm)&0xF0) ? 0 : 30 - ((imm)&0xF), \
                                     ((imm)&0xF0) ? 0 : 31 - ((imm)&0xF)); })
#endif

#if __has_builtin(__builtin_shufflevector) && !__has_builtin(__builtin_ia32_psrldqi128)
#undef _mm_srli_si128
#define _mm_srli_si128(a, imm) __extension__ ({                          \
    (__m128i)__builtin_shufflevector((__v16qi)(__m128i)(a),                \
                                     (__v16qi)_mm_setzero_si128(),         \
                                     ((imm)&0xF0) ? 16 : ((imm)&0xF) + 0,  \
                                     ((imm)&0xF0) ? 16 : ((imm)&0xF) + 1,  \
                                     ((imm)&0xF0) ? 16 : ((imm)&0xF) + 2,  \
                                     ((imm)&0xF0) ? 16 : ((imm)&0xF) + 3,  \
                                     ((imm)&0xF0) ? 16 : ((imm)&0xF) + 4,  \
                                     ((imm)&0xF0) ? 16 : ((imm)&0xF) + 5,  \
                                     ((imm)&0xF0) ? 16 : ((imm)&0xF) + 6,  \
                                     ((imm)&0xF0) ? 16 : ((imm)&0xF) + 7,  \
                                     ((imm)&0xF0) ? 16 : ((imm)&0xF) + 8,  \
                                     ((imm)&0xF0) ? 16 : ((imm)&0xF) + 9,  \
                                     ((imm)&0xF0) ? 16 : ((imm)&0xF) + 10, \
                                     ((imm)&0xF0) ? 16 : ((imm)&0xF) + 11, \
                                     ((imm)&0xF0) ? 16 : ((imm)&0xF) + 12, \
                                     ((imm)&0xF0) ? 16 : ((imm)&0xF) + 13, \
                                     ((imm)&0xF0) ? 16 : ((imm)&0xF) + 14, \
                                     ((imm)&0xF0) ? 16 : ((imm)&0xF) + 15); })
#endif
#endif

/**
 * Workarounds for platforms not supporting SSSE3.
 */
#if !defined TARGET_ROBOT && !defined MACOS
bool _checkSSSE3();
const bool _supportsSSSE3 = _checkSSSE3();

ALWAYSINLINE __m128i my_mm_shuffle_epi8(__m128i a, __m128i m)
{
  __m128i r;
#ifdef _MSC_VER
  const char* ac = a.m128i_i8;
  const char* mc = m.m128i_i8;
  char* rc = r.m128i_i8;
#else
  const char* ac = reinterpret_cast<const char*>(&a);
  const char* mc = reinterpret_cast<const char*>(&m);
  char* rc = reinterpret_cast<char*>(&r);
#endif
  for(size_t i = 0; i < 16; ++i)
  {
    if(mc[i] & 0x80)
    {
      rc[i] = 0;
    }
    else
    {
      rc[i] = ac[mc[i] & 0x0F];
    }
  }
  return r;
}

ALWAYSINLINE __m128i my_mm_abs_epi16(const __m128i a)
{
  return _mm_add_epi16(_mm_xor_si128(a, _mm_srai_epi16(a, 15)), _mm_srli_epi16(a, 15));
}

ALWAYSINLINE __m128i my_mm_hadd_epi16(const __m128i a, const __m128i b)
{
  static const __m128i ones = _mm_set1_epi16(1);
  return _mm_packs_epi32(
           _mm_madd_epi16(a, ones),
           _mm_madd_epi16(b, ones)
         );
}

ALWAYSINLINE __m128i my_mm_hadd_epi32(const __m128i a, const __m128i b)
{
  return _mm_unpacklo_epi64(
           _mm_add_epi32(_mm_shuffle_epi32(a, 2 << 2), _mm_shuffle_epi32(a, 1 | (3 << 2))),
           _mm_add_epi32(_mm_shuffle_epi32(b, 2 << 2), _mm_shuffle_epi32(b, 1 | (3 << 2)))
         );
}

ALWAYSINLINE __m128i my_mm_mulhrs_epi16(__m128i a, __m128i b)
{
  __m128i r;
#ifdef _MSC_VER
  const short* ac = a.m128i_i16;
  const short* bc = b.m128i_i16;
  short* rc = r.m128i_i16;
#else
  const short* ac = reinterpret_cast<const short*>(&a);
  const short* bc = reinterpret_cast<const short*>(&b);
  short* rc = reinterpret_cast<short*>(&r);
#endif
  for(size_t i = 0; i < 8; i++)
  {
    rc[i] = ((ac[i] * bc[i]) + 0x4000) >> 15;
  }
  return r;
}

ALWAYSINLINE __m128i my_mm_maddubs_epi16(const __m128i a, const __m128i b)
{
  return _mm_packs_epi32(
           _mm_madd_epi16(
             _mm_unpacklo_epi8(a, _mm_setzero_si128()),
             _mm_unpacklo_epi8(b, _mm_setzero_si128())
           ),
           _mm_madd_epi16(
             _mm_unpackhi_epi8(a, _mm_setzero_si128()),
             _mm_unpackhi_epi8(b, _mm_setzero_si128())
           )
         );
}

#define my_mm_alignr_epi8(a, b, count) \
  _mm_or_si128(_mm_srli_si128(b, count), _mm_slli_si128(a, 16 - count))

// if _mm_alignr_epi8 is defined as a macro, we cannot simply replace it.
#ifdef _mm_alignr_epi8
#define my_mm_alignr_epi8_org(a, b, n) __extension__ ({ \
    __m128i __a = (a); \
    __m128i __b = (b); \
    (__m128i)__builtin_ia32_palignr128((__v16qi)__a, (__v16qi)__b, (n)); })
#undef _mm_alignr_epi8
#define _mm_alignr_epi8(a, b, count) (_supportsSSSE3 ? my_mm_alignr_epi8_org(a, b, count) : my_mm_alignr_epi8(a, b, count))
#else
#define _mm_alignr_epi8(a, b, count) (_supportsSSSE3 ? _mm_alignr_epi8(a, b, count) : my_mm_alignr_epi8(a, b, count))
#endif

#define _mm_abs_epi16(a) (_supportsSSSE3 ? _mm_abs_epi16(a) : my_mm_abs_epi16(a))
#define _mm_hadd_epi16(a, b) (_supportsSSSE3 ? _mm_hadd_epi16(a, b) : my_mm_hadd_epi16(a, b))
#define _mm_hadd_epi32(a, b) (_supportsSSSE3 ? _mm_hadd_epi32(a, b) : my_mm_hadd_epi32(a, b))
#define _mm_maddubs_epi16(a, b) (_supportsSSSE3 ? _mm_maddubs_epi16(a, b) : my_mm_maddubs_epi16(a, b))
#define _mm_mulhrs_epi16(a, b) (_supportsSSSE3 ? _mm_mulhrs_epi16(a, b) : my_mm_mulhrs_epi16(a, b))
#define _mm_shuffle_epi8(a, m) (_supportsSSSE3 ? _mm_shuffle_epi8(a, m) : my_mm_shuffle_epi8(a, m))
#endif

/**
 * Aligned / unaligned load by template parameter.
 */
template<bool aligned> static ALWAYSINLINE __m128i _mm_loadt_si128(__m128i const* src)
{
  return aligned ? _mm_load_si128(src) : _mm_loadu_si128(src);
}

/**
 * Aligned / unaligned store by template parameter.
 */
template<bool aligned> static ALWAYSINLINE void _mm_storet_si128(__m128i* dest, const __m128i val)
{
  if(aligned)
    _mm_store_si128(dest, val);
  else
    _mm_storeu_si128(dest, val);
}

/**
 * 64-bit integer store.
 */
ALWAYSINLINE void _mm_storel_epi64(unsigned long long* dest, const __m128i src)
{
  _mm_storel_pi(reinterpret_cast<__m64*>(dest), _mm_castsi128_ps(src));
}

ALWAYSINLINE bool aligned16(const void* x) {return ((reinterpret_cast<size_t>(x)) & 0xf) == 0;}
