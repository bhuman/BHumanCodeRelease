#pragma once

/*! \file circleCNSSSE.h

    Contains a collection of SSE intrinsic based subroutines used
    by \c CircleCNSDetector. These routines cover most of the computation
    time and are highly optimized.
 */

#include "Tools/ImageProcessing/SIMD.h"
#include <cassert>
#include "Representations/Perception/ImagePreprocessing/CNSImage.h"
#include "CodedContour.h"

//! Implicit factor involved in the computation of \c responseX16Y16RUsingSSE3
/*! \c responseX16Y16RUsingSSE3 compute
       \code
          ((cx-128)*nx+(cy-128)*ny)^2 >> RESPONSE_COMPUTATION_IMPLICIT_SHIFTRIGHT
       \endcode
    for every pixel on the contour where (cx,cy) is the binary value from the
    \c CnsImage and (nx, ny) is the binary value from the \c CodedContur.

    \c RESPONSE_COMPUTATION_IMPLICIT_SHIFTRIGHT gives the implicit shift in this
    computation.
 */
enum {RESPONSE_COMPUTATION_IMPLICIT_SHIFTRIGHT = 16};

//! The core response computation for one SSE register (more for reference and unit tests)
inline void responseX8YRUsingSSE3(const CNSResponse* srcPixel, int srcOfs, unsigned short* responseBin, const CodedContourPoint& ccp)
{
  srcOfs /= sizeof(CNSResponse);
  static const __m128i cns_const128V     = _mm_set1_epi8(-128);
  __m128i cosSinVal  = _mm_set1_epi16(nOfCCP(ccp));  // put the (nx,ny) normal vector into every component
  const CNSResponse* srcRun = srcPixel + xOfCCP(ccp) + srcOfs * yOfCCP(ccp);
  __m128i cosPSinVal = _mm_maddubs_epi16(cns_const128V, cosSinVal);
  __m128i dataA;

  dataA     = _mm_loadu_si128((__m128i*)(srcRun + 0));
  dataA     = _mm_maddubs_epi16(dataA, cosSinVal);
  dataA     = _mm_subs_epi16(dataA, cosPSinVal);
  dataA     = _mm_mulhi_epi16(dataA, dataA);
  dataA     = _mm_adds_epu16(dataA, *(__m128i*)(responseBin + 0x00));
  *(__m128i*)(responseBin + 0x00) = dataA;
}

//! faster variant of \c reponseX16Y16RUsingC with SSE3
/*! The routine takes an image of contrast normalized Sobel (CNS)
  responses, and a \c contour and computes the contour quality
  value for an 16*16 block of potential reference points. In terms
  of abstract quantities the quality value is defined as

  \f{equation}{
       \newcommand{\mvs}[1]{\!\left(\begin{smallmatrix}#1\end{smallmatrix}\right)}
       CQ(x, y) = \int_0^1 resp\left(p(\lambda)+\mvs{x\\y}, \angle p'(\lambda)+\frac{\pi}{2}\right) d\lambda, \\
       resp(x_0,y_0,\theta) = \left( \mvs{\cos\theta\\\sin\theta}^T \cng(x_0,y_0) \right)^2,
  \f}

  where \f$ p:[0..1]\rightarrow R^2 \f$ is the contour
  and \f$ cng(x,y) \f$ is the contrast normalized gradient image.

  \c srcPixel is a pointer to the CNS result image pixel
  corresponding to the reference point for the contour.  See documentation of
  \c CNSResponse for the coding of the values.

  \c srcOfs is the offset in CNSResponse units between successive lines in the
  CNS image

  \c accPixel is the pointer to 16*16 array of \c unsigned short
  values, where the result CQ(x,y) is stored. The entry (0,0)
  corresponds to a reference point at \c srcPixel. The other ones
  correspond to reference points to the right and down accordingly. The
  results are scaled by 0x10000 ranging [0..1]. The  pointer must be aligned on 16 bytes.
 */
void responseX16Y16RUsingSSE3(const CNSResponse* srcPixel, int srcOfs, signed short* responseBin, const CodedContour& contour);

//! 8*8 block variant of \c reponseX16Y16RUsingSSE3
/*! Used in refinement.
 */
void responseX8Y8RUsingSSE3(const CNSResponse* srcPixel, int srcOfs, signed short* responseBin, const CodedContour& contour);

//! scales and shifts a raw response converting it from uint16 to signed int16
/*! \x (uint16) is mapped to \c x*scale>>16+offset. \c scale must be >=0.
 */
inline void scaleOffsetUsingSSE(unsigned short* src, short* dst, int n, short scale, signed short offset)
{
  __m128i* pEnd = (__m128i*)(src + n);
  __m128i* p    = (__m128i*) src;
  __m128i* pDst = (__m128i*) dst;
  assert(aligned16(src) && n % 32 == 0 && aligned16(dst));
  __m128i scaleV = _mm_set1_epi16(scale);
  __m128i offsetV = _mm_set1_epi16(offset);
  while(p != pEnd)
  {
    for(int unrollCtr = 0; unrollCtr < 4; unrollCtr++)
    {
      *pDst = _mm_add_epi16(_mm_mulhi_epu16(*p, scaleV), offsetV);
      p++;
      pDst++;
    }
  }
}

//! Optimized SSE2 implementation of \c maximum
static const __m128i cns_const8     = _mm_set1_epi16(8);
static const __m128i cns_constCount    = _mm_set_epi16(7, 6, 5, 4, 3, 2, 1, 0);
static const __m128i cns_constMinShort = _mm_set1_epi16(-0x8000);

inline void maximumUsingSSE2(int& max, int& argMax, int baseArg, const short* accPixel, int nAccPixel)
{
  assert(nAccPixel % 32 == 0);
  assert(baseArg + nAccPixel < 0x10000);

  // First build max and argMax in 8 entry-blocks
  __m128i lMax    = cns_constMinShort;
  __m128i lArgMax = _mm_setzero_si128();
  __m128i lIndex  = _mm_add_epi16(_mm_set1_epi16(static_cast<short>(baseArg)), cns_constCount);
  __m128i* accPixelEnd = reinterpret_cast<__m128i*>(const_cast<short*>(accPixel + nAccPixel));
  __m128i* p = reinterpret_cast<__m128i*>(const_cast<short*>(accPixel));
  while(p != accPixelEnd)
  {
    for(int unrollCtr = 0; unrollCtr < 4; unrollCtr++)
    {
      __m128i val = *p;
      lMax = _mm_max_epi16(lMax, val);
      __m128i mask = _mm_cmpeq_epi16(val, lMax);  // 0xffff for new maxima
      lArgMax = _mm_max_epi16(lArgMax, _mm_and_si128(mask, lIndex));   // update values in lArgmax where mask is 0xffff with index
      lIndex = _mm_add_epi16(lIndex, cns_const8);
      p++;
    }
  }
  alignas(16) signed short lMaxShort[8];
  alignas(16) signed short lArgMaxShort[8];
  _mm_store_si128((__m128i*) lMaxShort, lMax);
  _mm_store_si128((__m128i*) lArgMaxShort, lArgMax);

  // Then find overall maximum as maximum of the 8 entries
  for(int i = 0; i < 8; i++)
  {
    if(lMaxShort[i] > max)
    {
      max    = lMaxShort[i];
      argMax = lArgMaxShort[i];
    }
  }
}

// This is the body of the innermost computation loop. It computes CNS
// response for a single contour pixel in parallel applied to 8 CNS
// pixels src[0..7] and accumulates the response in acc[0..7].  The
// fact that this routine is so highly optimized, being reduced to
// only 5 operations when inlined dominates the overall computation
// time.
//
// The formal computation is specified in \c
// cns_reponseX8YRUsingC.  In general it computes \c
// ((cns_x-128)*normal_x+(cns_y-128)*normal_y)^2>>16
// \c cns_x and \c cns_y are given in \c
// srcPixel, \c normal_x and \c normal_y by odd and even signed bytes
// in \c cosSinVal. \c cosPSinVal is needed for technical reasons and
// contains the signed 16bit result of \c _mm_maddubs_epi16
// (cns_const128V, cosSinVal);
inline void cns_reponseX8YRUsingSSE3(const CNSResponse* __restrict srcPixel, __m128i cosSinVal, __m128i cosPSinVal, __m128i& acc)
{
  __m128i val = _mm_subs_epi16(_mm_maddubs_epi16(_mm_loadu_si128((__m128i*) srcPixel), cosSinVal), cosPSinVal);
  acc = _mm_adds_epu16(acc, _mm_mulhi_epi16(val, val));
}

inline void cns_reponseX16YRUsingSSE3(const CNSResponse* __restrict srcPixel, __m128i cosSinVal, __m128i cosPSinVal, __m128i* __restrict acc)
{
  __m128i srcA = _mm_loadu_si128((__m128i*) srcPixel);
  __m128i srcB = _mm_loadu_si128((__m128i*)(srcPixel + 8));

  __m128i valA = _mm_subs_epi16(_mm_maddubs_epi16(srcA, cosSinVal), cosPSinVal);
  __m128i valB = _mm_subs_epi16(_mm_maddubs_epi16(srcB, cosSinVal), cosPSinVal);
  acc[0] = _mm_adds_epu16(acc[0], _mm_mulhi_epi16(valA, valA));
  acc[1] = _mm_adds_epu16(acc[1], _mm_mulhi_epi16(valB, valB));
}

ALWAYSINLINE void cns_zeroAccumulator(unsigned short* acc, int nAcc)
{
  __m128i* p = (__m128i*) acc;
  __m128i* pEnd = (__m128i*)(acc + nAcc);
  __m128i zero = _mm_setzero_si128();
  while(p != pEnd)
  {
    *p = zero;
    p++;
  }
}

ALWAYSINLINE void cns_copyAccumulator(unsigned short* dst, unsigned short* src, int nAcc);
inline void cns_copyAccumulator(unsigned short* dst, unsigned short* src, int nAcc)
{
  __m128i* p = (__m128i*) src;
  __m128i* pEnd = (__m128i*)(src + nAcc);
  __m128i* pD = (__m128i*) dst;
  while(p != pEnd)
  {
    *pD = *p;
    p++;
    pD++;
  }
}

//! Internal subroutine for cnsResponse
/*! load 8 image pixel and convert to 16 bit, also generates 1 pixel shifts for later filter computation
  img[i] contains src[i], imgL[i] contains src[i-1] and, imgR[i] contains src[i+1], i=0..7
  when interpreting __m128i as unsigned short [8];
  \c isFirst must be set for first block in a row, which does not access src[-1] and uses 0 instead
  \c isLast must be set for the last block in a row, which does not access src[8] and uses 0 instead
 */
ALWAYSINLINE void load8PixelUsingSSE(__m128i& imgL, __m128i& img, __m128i& imgR, const unsigned char* src, bool isFirst, bool isLast)
{
  __m128i imgRaw;
  if(!isLast)
  {
    if(!isFirst)
      imgRaw = _mm_loadu_si128((__m128i*)(src - 1));          // regular pixel in the middle of the image
    else
      imgRaw = _mm_slli_si128(_mm_loadu_si128((__m128i*)src), 1);   // on the left border take one adress later and shift
  }
  else
    imgRaw = _mm_srli_si128(_mm_loadu_si128((__m128i*)(src - 8)), 7);  // right border take 7 adress earlier and shift
  imgL   = _mm_unpacklo_epi8(imgRaw, _mm_setzero_si128());
  img    = _mm_unpacklo_epi8(_mm_srli_si128(imgRaw, 1), _mm_setzero_si128());
  imgR   = _mm_unpacklo_epi8(_mm_srli_si128(imgRaw, 2), _mm_setzero_si128());
}

//! Computes SIMD a+2*b+c
ALWAYSINLINE __m128i blur_epi16(__m128i a, __m128i b, __m128i c) {return _mm_add_epi16(a, _mm_add_epi16(b, _mm_add_epi16(b, c)));}

//! Computes SIMD a+2*b+c
ALWAYSINLINE __m128i blur_epi32(__m128i a, __m128i b, __m128i c) {return _mm_add_epi32(a, _mm_add_epi32(b, _mm_add_epi32(b, c)));}

//! Computes SIMD a+2*b+c
ALWAYSINLINE __m128 blur_ps(__m128 a, __m128 b, __m128 c) {return _mm_add_ps(a, _mm_add_ps(b, _mm_add_ps(b, c)));}

//! Compute a gaussian [1 2 1]^T*[1 2 1] filter
/*! First, the [1 2 1] convolution is computed horizontally, the result stored in currentIV.
    Then this result and the values of \c previousIV and the old value of \c currentIV
    are combined vertically with the result stored in \c gaussI.
 */
ALWAYSINLINE void gauss(__m128i& currentIV, const __m128i& previousIV, __m128i& gaussI,
                        __m128i imgL, __m128i img, __m128i imgR)
{
  __m128i blurX      =  blur_epi16(imgL, img, imgR);  // [1 2 1]*I
  gaussI             = blur_epi16(blurX, previousIV, currentIV);   // [1 2 1]*[1 2 1]*I
  currentIV          = blurX;
}

//! Overloaded function that only computes intermediate results in \c currentIV not final ones
ALWAYSINLINE void gauss(__m128i& currentIV, const __m128i& previousIV,
                        __m128i imgL, __m128i img, __m128i imgR)
{
  // Call \c filters with dummy variables. Compiler will optimize unnecessary computations out.
  __m128i gaussI;
  gauss(currentIV, previousIV, gaussI, imgL, img, imgR);
}

//! Sets dst[i] to src[2*i] for i=0..w/2
/*! This helper function is used for the margin in the downsampled image
 */
void decimateLineUsingSSE(const unsigned char* src, int w, unsigned char* ds);

//! converts 8 unsigned short responses in acc into floats storing at \c response[0..1]
/*! The conversion is by multiplying with \c invResponseAccumulatorScale */
ALWAYSINLINE void convertX8ResponsesToFloatUsingSSE(__m128* response, __m128i acc, __m128 invResponseAccumulatorScale)
{
  _mm_stream_ps(reinterpret_cast<float*>(response), _mm_mul_ps(_mm_cvtepi32_ps(_mm_unpacklo_epi16(acc, _mm_setzero_si128())), invResponseAccumulatorScale));
  _mm_stream_ps(reinterpret_cast<float*>(response + 1), _mm_mul_ps(_mm_cvtepi32_ps(_mm_unpackhi_epi16(acc, _mm_setzero_si128())), invResponseAccumulatorScale));
}

//! Converts a 16*16 block of responses in \c acc from unsigned short into float
/*! .. by multiplying with \c invResponseAccumulatorScale
    \c acc[x+16*y] is stored into \c response[x+responseOfs*y]
 */
void convertX16Y16RResponsesToFloatUsingSSE(float* response, int responseOfs, unsigned short* acc, float invResponseAccumulatorScale);

//! Fills response with n times 0
ALWAYSINLINE void cns_zeroFloatArray(float* response, int n)
{
  float* rEnd = response + n;
  __m128i zeroV = _mm_setzero_si128();
  while(response != rEnd)
  {
    *(__m128i*) response = zeroV;
    response++;
  }
}

//! optimized SSE implementation for projecting 3D points in the image
/*! See \c projectUsingC, used by \LutRasterizer */
inline bool projectUsingSSE(float img[2], float p[4], float p0[4], float p1[4], float p2[4])
{
  __m128 pV  = *((__m128*)p);
  __m128 nom = _mm_hadd_ps(_mm_mul_ps(pV, *((__m128*)p0)), _mm_mul_ps(pV, *((__m128*)p1)));
  nom = _mm_hadd_ps(nom, nom);
  __m128 denom = _mm_mul_ps(pV, *((__m128*)p2));
  denom = _mm_hadd_ps(denom, denom);
  denom = _mm_hadd_ps(denom, denom);
  _mm_storel_pi((__m64*) img, _mm_div_ps(nom, denom));
  return _mm_movemask_ps(denom) == 0;
}

//! optimized SSE implementation to make a \c CodedContourPoint from an edge in the image
/*! See \c code2DEdgeUsingC, used by \LutRasterizer */
inline unsigned int code2DEdgeUsingSSE(float img[4], float clipRange[4])
{
  static __m128 half_ps = _mm_set1_ps(0.5);
  __m128 pAB = *((__m128*)img);
  __m128 pBA = _mm_shuffle_ps(pAB, pAB, _MM_SHUFFLE(1, 0, 3, 2));
  __m128 pCC = _mm_mul_ps(_mm_add_ps(pAB, pBA), half_ps);   // center (x0+x1)/2,(y0+y1)/2, (x0+x1)/2,(y0+y1)/2
  if(_mm_movemask_ps(_mm_sub_ps(pCC, *(__m128*) clipRange)) != 12)
    return EMPTYCONTOURPOINT;

  __m128 pDD = _mm_sub_ps(pBA, pAB);    // x1-x0, y1-y0, x1-x0, y1-y0
  __m128 pLL = _mm_mul_ps(pDD, pDD);    // (x1-x0)^2, (y1-y0)^2, (x1-x0)^2, (y1-y0)^2
  pLL = _mm_hadd_ps(pLL, pLL);  // 4 times (x1-x0)^2+(y1-y0)^2
  pLL = _mm_rsqrt_ss(pLL); // 1/srqt((x1-x0)^2+(y1-y0)^2)
  pLL = _mm_shuffle_ps(pLL, pLL, _MM_SHUFFLE(0, 0, 0, 0));
  static __m128 c127_ps = _mm_set_ps(-127, 127, -127, 127);
  pDD =  _mm_mul_ps(_mm_mul_ps(pLL, c127_ps), pDD);
  // pDD is the vector scaled to length 127 and rotated 90 deg, just need to swap x and y
  // so combine x from PC, y from PC, y from pDD, x from PDD
  __m128 combined  = _mm_shuffle_ps(pCC, pDD, _MM_SHUFFLE(0, 1, 1, 0));

  __m128i coded = _mm_cvtps_epi32(combined);
  coded = _mm_packs_epi32(coded, coded);
  coded = _mm_packs_epi16(coded, coded);

  return _mm_cvtsi128_si32(coded);
}

//! Scales the normal vector of \c ccp[0..n-1] by \c factor
/*! Optimized implementation. Warning, may overwrite up to \c ccp[n+2]. */
inline void scaleNormalUsingSSE(CodedContourPoint* ccp, int n, float factor)
{
  // Warning we write over the end of [n] for performance reasons
  static __m128i makeNormal16s = _mm_set_epi8(15, -0x80, 14, -0x80, 11, -0x80,   10, -0x80, 7, -0x80,    6, -0x80, 3, -0x80,    2, -0x80);
  static __m128i makeNormal8s  = _mm_set_epi8(15, 13, -0x80, -0x80, 11,    9, -0x80, -0x80, 7,    5, -0x80, -0x80, 3,    1, -0x80, -0x80);
  static __m128i maskNormalOut = _mm_set_epi8(0,  0, -1, -1,  0,    0, -1, -1, 0,    0, -1, -1, 0,    0, -1, -1);
  int factorI = static_cast<int>(factor * 0x10000);
  __m128i factorV = _mm_set1_epi16(static_cast<short>(factorI));
  assert(abs(factorI) < 0x8000);
  CodedContourPoint* ccpEnd = ccp + n;
  while(ccp < ccpEnd)
  {
    __m128i ccpI = *(__m128i*) ccp;
    __m128i result = _mm_shuffle_epi8(_mm_mulhi_epi16(_mm_shuffle_epi8(ccpI, makeNormal16s), factorV), makeNormal8s);  // (_*factorI)>>16
    result = _mm_or_si128(result, _mm_and_si128(ccpI, maskNormalOut));
    *(__m128i*) ccp = result;
    ccp += 4;
  }
}

//! x[0]=x[2]; x[1]=x[3];
inline void shift4Floats(float x[4])
{
  __m128 tmp = *(__m128*)x;
  *(__m128*)x = _mm_shuffle_ps(tmp, tmp, _MM_SHUFFLE(3, 2, 3, 2));
}
