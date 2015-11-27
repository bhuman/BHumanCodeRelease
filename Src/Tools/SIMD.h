/**
 * @file SIMD.h
 *
 * The file defines some compiler specific specializations of MMX and SSE intrinsics
 *
 * @author Alexander Härtl
 */

#pragma once

#include <tmmintrin.h>

#ifdef TARGET_SIM
inline __m128i my_mm_shuffle_epi8(const __m128i& a, const __m128i& m)
{
  __m128i r;
  const char* ac = reinterpret_cast<const char*>(&a);
  const char* mc = reinterpret_cast<const char*>(&m);
  char* rc = reinterpret_cast<char*>(&r);

  for(size_t i = 0; i < 16; ++i)
  {
    if(*(mc + i) & 0x80)
      *(rc + i) = 0;
    else
      *(rc + i) = *(ac + ((*(mc + i)) & 0x0F)); //a[b[i] & 0x0F];
  }
  return r;
}

#define SHUFFLE(a, m) (my_mm_shuffle_epi8(a, m))
#else
#define SHUFFLE(a, m) (_mm_shuffle_epi8(a, m))
#endif
