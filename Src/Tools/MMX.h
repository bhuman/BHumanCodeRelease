/**
 * @file MMX.h
 *
 * The file defines some compiler specific specializations of MMX intrinsics
 *
 * @author Alexander Härtl
 */

#pragma once

#ifdef __clang__
#pragma clang system_header
#endif

#include <tmmintrin.h>

#ifdef TARGET_SIM
inline __m128i my_mm_shuffle_epi8(const __m128i& a, const __m128i& m)
{
  __m128i r;
  char* ac = (char*)&a;
  char* mc = (char*)&m;
  char* rc = (char*)&r;

  for(int i = 0; i < 16; i++)
  {
    if(*(mc + i) & 0x80)
      *(rc + i) =  0;
    else
      *(rc + i) = *(ac + ((*(mc + i)) & 0x0F)); //a[b[i] & 0x0F];
  }
  return r;
}
#define SHUFFLE(a, m) (my_mm_shuffle_epi8(a, m))
#else
#define SHUFFLE(a, m) (_mm_shuffle_epi8(a, m))
#endif

#define MM_EMPTY _mm_empty
