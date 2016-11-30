/**
 * @file SIMD.cpp
 *
 * @author Felix Thielke
 */

#include "AVX.h"

#if !defined TARGET_ROBOT && !defined MACOS
#ifdef _MSC_VER
bool _checkSSSE3()
{
  int info[4];
  __cpuid(info, 0);
  if(info[0] >= 1)
  {
    __cpuidex(info, 1, 0);
    return (info[2] & (1 << 9)) != 0;
  }
  return false;
}
#else
#include <cpuid.h>
bool _checkSSSE3()
{
  if(__get_cpuid_max(0, nullptr) >= 1)
  {
    unsigned int eax, ebx, ecx, edx;
    __get_cpuid(1, &eax, &ebx, &ecx, &edx);
    return (ecx & (1 << 9)) != 0;
  }
  return false;
}
#endif
#endif

#ifndef TARGET_ROBOT
#ifdef _MSC_VER
bool _checkAVX2()
{
  int info[4];
  __cpuid(info, 0);
  if(info[0] >= 7)
  {
    __cpuidex(info, 7, 0);
    return (info[1] & (1 << 5)) != 0;
  }
  return false;
}
#else
#ifndef __cpuid
#include <cpuid.h>
#endif
bool _checkAVX2()
{
  if(__get_cpuid_max(0, nullptr) >= 7)
  {
    unsigned int eax, ebx, ecx, edx;
    __get_cpuid(7, &eax, &ebx, &ecx, &edx);
    return (ebx & (1 << 5)) != 0;
  }
  return false;
}
#endif
#endif
