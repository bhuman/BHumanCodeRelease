/**
 * This file implements a module that calculates contrast normalized Sobel (cns) images.
 * @author Udo Frese
 * @author Thomas Röfer
 * @author Jesse Richter-Klug
 * @author Lukas Post
 */

#include "CNSImageProvider.h"
#include "Representations/Infrastructure/CameraImage.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/BHMath.h"
#include "Tools/ImageProcessing/AVX.h"
#include "Tools/Math/Transformation.h"
#include "Tools/ImageProcessing/InImageSizeCalculations.h"

MAKE_MODULE(CNSImageProvider, perception);

///////////////////////////////////////////////////////////////////////////
// Local helpers

/** Intermediate values stored in a buffer of two lines for CNS computation. */
struct IntermediateValues
{
  /** [+1 0 -1]*I horizontal derivation filter (epi16). */
  __m128i dX;

  /** [1 2 1]*I horizontal Gaussian (epi16). */
  alignas(16) __m128i gaussIX;

  /** 16*[1 2 1]*I^2 horizontal Gaussian on squared image (ps). */
  __m128 gaussI2XA, gaussI2XB;

  /** [1 2 1]^T*[1 2 1]*I Gaussian (epi16). */
  __m128i gaussI;

  short getDX(int i) const
  {
    return (reinterpret_cast<const short*>(&dX))[i];
  }

  short getGaussIX(int i) const
  {
    return (reinterpret_cast<const short*>(&gaussIX))[i];
  }

  float getGaussI2X(int i) const
  {
    if(i < 4)
      return (reinterpret_cast<const float*>(&gaussI2XA))[i];
    else
      return (reinterpret_cast<const float*>(&gaussI2XB))[i - 4];
  }

  int getGaussI(int i) const
  {
    return (reinterpret_cast<const short*>(&gaussI))[i];
  }

  /** Default: Leave uninitialized. */
  IntermediateValues() = default;

  /** Constructor to explicitly initialize with zero. */
  IntermediateValues(int zero)
    : dX(_mm_set1_epi16(0)),
      gaussIX(_mm_set1_epi16(0)),
      gaussI2XA(_mm_set1_ps(0)),
      gaussI2XB(_mm_set1_ps(0)),
      gaussI(_mm_set1_epi16(0))
  {}
};

/**
 * Internal subroutine for cnsResponse.
 * Load 2 x 8 image pixel and convert to 16 bit, also generates 1 pixel shifts for later filter computation.
 * img[i] contains src[i], imgL[i] contains src[i-1] and, imgR[i] contains src[i+1], i = 0..7
 * when interpreting __m128i as unsigned short[8].
 * lastScr is the __m128i directly before the current one (src), which is directly folllowed by nextSrc
 */
ALWAYSINLINE static void load2x8PixelUsingSSE(__m128i& imgL, __m128i& img, __m128i& imgR,
  __m128i& imgL2, __m128i& img2, __m128i& imgR2,
  __m128i& lastSrc, __m128i& src,  const __m128i* const nextSrcP)
{
  const __m128i nextSrc = _mm_load_si128(nextSrcP);

  //imgL = _mm_unpacklo_epi8(_mmauto_add_epi8(_mmauto_srli_si_all(lastSrc, 15), _mmauto_slli_si_all(src, 1)), _mm_setzero_si128());
  imgL = _mm_unpacklo_epi8(_mm_alignr_epi8(src, lastSrc, 15), _mm_setzero_si128());
  img = _mm_unpacklo_epi8(src, _mm_setzero_si128());
  imgR = _mm_unpacklo_epi8(_mm_srli_si128(src, 1), _mm_setzero_si128());

  imgL2 = _mm_unpacklo_epi8(_mm_srli_si128(src, 7), _mm_setzero_si128());
  img2 = _mm_unpacklo_epi8(_mm_srli_si128(src, 8), _mm_setzero_si128());
  //imgR2 = _mm_unpacklo_epi8(_mmauto_add_epi8(_mmauto_srli_si_all(src, 9), _mmauto_slli_si_all(nextSrc, 7)), _mm_setzero_si128());
  imgR2 = _mm_unpacklo_epi8(_mm_alignr_epi8(nextSrc, src, 9), _mm_setzero_si128());

  lastSrc = src;
  src = nextSrc;
}

/** Computes SIMD a+2*b+c. */
ALWAYSINLINE static __m128i blur_epi16(__m128i a, __m128i b, __m128i c)
{
  return _mm_add_epi16(a, _mm_add_epi16(b, _mm_add_epi16(b, c)));
}

/** Computes SIMD a+2*b+c. */
ALWAYSINLINE static __m128i blur_epi32(__m128i a, __m128i b, __m128i c)
{
  return _mm_add_epi32(a, _mm_add_epi32(b, _mm_add_epi32(b, c)));
}

/** Computes SIMD a+2*b+c. */
ALWAYSINLINE static  __m128 blur_ps(__m128 a, __m128 b, __m128 c)
{
  return _mm_add_ps(a, _mm_add_ps(b, _mm_add_ps(b, c)));
}

static __m128i cnsOffsetV = _mm_set1_epi8(static_cast<unsigned char>(CNSResponse::OFFSET));

/**
 * Sets \c cns[i] to \c CNSResponse() for \c i = 0 .. width-1.
 * \c width must be a multiple of 8.
 */
static void fillWithCNSOffsetUsingSSE(short* cns, int width)
{
  short* cnsEnd = cns + width;
  while(cns < cnsEnd)
  {
    _mm_store_si128(reinterpret_cast<__m128i*>(cns), cnsOffsetV);
    cns += 8;
  }
}

/**
 * SSE Implementation of \c cnsFormula (subroutine of cnsResponse).
 * \c scale, \c gaussI2 and \c regVar are 32bit floats (gaussI2 as A and B).
 * \c sobelX, \c sobelY, \c gaussI are signed short.
 * \c result is a packed vector of unsigned signed 8bit number with the x and y component
 * alternating and \c offset (unsigned char) added.
 */
ALWAYSINLINE static void cnsFormula(__m128i& result, __m128i sobelX, __m128i sobelY, __m128i& gaussI,
                                    const __m128& gaussI2A, const __m128& gaussI2B,
                                    const __m128& scale, const __m128& regVar, __m128i offset)
{
  __m128 gaussIA = _mm_cvtepi32_ps(_mm_unpacklo_epi16(gaussI, _mm_setzero_si128()));
  __m128 gaussIB = _mm_cvtepi32_ps(_mm_unpackhi_epi16(gaussI, _mm_setzero_si128()));

  __m128 factorA = _mm_add_ps(_mm_sub_ps(gaussI2A, _mm_mul_ps(gaussIA, gaussIA)), regVar); // gaussI2-gaussI^2+regVar
  __m128 factorB = _mm_add_ps(_mm_sub_ps(gaussI2B, _mm_mul_ps(gaussIB, gaussIB)), regVar);

  factorA = _mm_mul_ps(_mm_rsqrt_ps(factorA), scale); // scale/sqrt(gaussI2-gaussI^2+regVar)
  factorB = _mm_mul_ps(_mm_rsqrt_ps(factorB), scale);

  // (2^-11)*sobelX*(scale/sqrt(gaussI2-gaussI^2+regVar))
  __m128i factor = _mm_packs_epi32(_mm_cvtps_epi32(factorA), _mm_cvtps_epi32(factorB));
  __m128i resultXepi16 = _mm_mulhi_epi16(_mm_slli_epi16(sobelX, 5), factor);
  __m128i resultYepi16 = _mm_mulhi_epi16(_mm_slli_epi16(sobelY, 5), factor);

  // Convert to 8bit and interleave X and Y
  // the second argument of packs duplicates values to higher bytes, but these are ignored later, unpacklo interleaves X and Y
  __m128i resultepi8 = _mm_unpacklo_epi8(_mm_packs_epi16(resultXepi16, resultXepi16), _mm_packs_epi16(resultYepi16, resultYepi16));

  result = _mm_add_epi8(resultepi8, offset); // add offset, switching to epu8
}

/**
 * Computes the various filters involved in CNS computation.
 * First, \c dX, blurX and blurX2 are computed horizontally from \c imgL, img, imgR and stored in \c currentIV.
 * Then, these intermediate values, the one from the previous line (\c previousIV) and the one from the line
 * 2 above (passed in \c currentIV) are used to compute sobelX, sobelY, gaussI and gaussI2A/B. The latter one
 * is floating point and separated into two halves.
 *
 * Also \c gaussI is stored in \c currentIV.gaussI (used for downsampling).
 */
ALWAYSINLINE static void filters(IntermediateValues& currentIV, const IntermediateValues& previousIV,
                                 __m128i& sobelX, __m128i& sobelY, __m128i& gaussI, __m128& gaussI2A, __m128& gaussI2B,
                                 __m128i imgL, __m128i img, __m128i imgR)
{
  __m128i dX = _mm_sub_epi16(imgR, imgL);   // [+1 0 -1]*I
  sobelX = blur_epi16(dX, previousIV.dX, currentIV.dX);   // [1 2 1]^T*[+1 0 -1]*I
  currentIV.dX = dX;

  __m128i blurX =  blur_epi16(imgL, img, imgR); // [1 2 1]*I
  sobelY = _mm_sub_epi16(blurX, currentIV.gaussIX);  // [+1 0 -1]*[1 2 1]*I
  gaussI = blur_epi16(blurX, previousIV.gaussIX, currentIV.gaussIX);  // [1 2 1]*[1 2 1]*I
  currentIV.gaussIX = blurX;

  __m128i img2 = _mm_mullo_epi16(img, img);
  __m128i img2A = _mm_unpacklo_epi16(img2, _mm_setzero_si128());
  __m128i img2B = _mm_unpackhi_epi16(img2, _mm_setzero_si128());  // (img2A, img2B) I^2 32bit

  __m128i img2L = _mm_mullo_epi16(imgL, imgL);
  __m128i img2LA = _mm_unpacklo_epi16(img2L, _mm_setzero_si128());
  __m128i img2LB = _mm_unpackhi_epi16(img2L, _mm_setzero_si128()); // (img2LA, img2LB) I^2 32bit shifted -1

  __m128i img2R = _mm_mullo_epi16(imgR, imgR);
  __m128i img2RA = _mm_unpacklo_epi16(img2R, _mm_setzero_si128());
  __m128i img2RB = _mm_unpackhi_epi16(img2R, _mm_setzero_si128());  // (img2RA, img2RB) img^2 shifted +1

  __m128i blurI2XA = blur_epi32(img2LA, img2A, img2RA); // [1 2 1]*I^2
  __m128i blurI2XB = blur_epi32(img2LB, img2B, img2RB); // [1 2 1]*I^2
  __m128 blurI2XAf = _mm_cvtepi32_ps(_mm_slli_epi32(blurI2XA, 4));
  __m128 blurI2XBf = _mm_cvtepi32_ps(_mm_slli_epi32(blurI2XB, 4));  // (blurI2XA, blurI2XB) = 16.0*[1 2 1]*I^2

  gaussI2A = blur_ps(blurI2XAf, previousIV.gaussI2XA, currentIV.gaussI2XA);
  gaussI2B = blur_ps(blurI2XBf, previousIV.gaussI2XB, currentIV.gaussI2XB);  // (gaussI2A, gaussI2B) = 16.0*[1 2 1]^T*[1 2 1]*I^2
  currentIV.gaussI2XA = blurI2XAf;
  currentIV.gaussI2XB = blurI2XBf;
  currentIV.gaussI = gaussI;
}

/** Overloaded function that only computes intermediate results in \c currentIV not final ones. */
ALWAYSINLINE static void filters(IntermediateValues& currentIV, const IntermediateValues& previousIV,
                                 __m128i imgL, __m128i img, __m128i imgR)
{
  // Call \c filters with dummy variables. Compiler will optimize unnecessary computations out.
  __m128i sobelX, sobelY, gaussI;
  __m128 gaussI2A, gaussI2B;
  filters(currentIV, previousIV, sobelX, sobelY, gaussI, gaussI2A, gaussI2B, imgL, img, imgR);
}

///////////////////////////////////////////////////////////////////////////

void CNSImageProvider::cnsResponse(const unsigned char* src, int width, int height,
                                   int srcOfs, short* cns, float regVar)
{
  ASSERT(CNSResponse::SCALE == 128);

  __m128i offset = _mm_set1_epi8(static_cast<unsigned char>(CNSResponse::OFFSET));

  // Image noise of variance \c regVar increases Gauss*I^2 by 16*regVar
  // an additional factor of 16 is needed, since Gauss*I^2 is multiplied by 16
  __m128 regVarF = _mm_set1_ps(16 * 16 * regVar);

  // A pure X-gradient gives: sobelX=8, sobelY=0, gaussI=0, gaussI2=8
  // hence the fraction sobelX/sqrt(16*gaussI2-gaussI*gaussI)=1/sqrt(2)
  // The assembler code implicitly multiplies with 2^(5-16), so
  // to get the desired CNSResponse::SCALE, we multiply with
  __m128 scaleF = _mm_set1_ps(CNSResponse::SCALE / std::pow(2.f, 5.f - 16.f) * std::sqrt(2.f));

  // Buffers for intermediate values for two lines
  alignas(16) IntermediateValues iv[2][CameraImage::maxResolutionWidth / 8]; // always 8 Pixel in one IntermediateValues object
  ASSERT((reinterpret_cast<size_t>(cns) & 0xf) == 0);

  int srcY = 0; // line in the source image

  // *** Go through two lines to fill up the intermediate Buffers
  // This is exactly the same code as below apart from the final computations being removed
  ASSERT(intptr_t(src) % 16 == 0);
  ASSERT(srcOfs % 8 == 0);
  ASSERT(width % 8 == 0);
  for(int i = 0; i < 2; ++i, ++srcY)
  {
    IntermediateValues* ivCurrent = &iv[srcY & 1][0];
    IntermediateValues* ivLast = &iv[1 - (srcY & 1)][0];
    const __m128i* pStart = reinterpret_cast<const __m128i*>(src + srcY * srcOfs - (srcY * srcOfs % 16 != 0 ? 8 : 0));
    const __m128i* pEnd = (pStart + width / 16) + (srcY * srcOfs % 16 != 0 ? 1 : 0);
    __m128i lastSrc, src;
    const __m128i* p = pStart;
    lastSrc = src = _mm_load_si128(p); //TODO change me (prev)
    for(; p != pEnd; ++ivCurrent, ++ivLast)
    {
      __m128i imgL, img, imgR;
      __m128i imgL2, img2, imgR2;
      load2x8PixelUsingSSE(imgL, img, imgR, imgL2, img2, imgR2, lastSrc, src, ++p);
      filters(*ivCurrent, *ivLast, imgL, img, imgR);
      filters(*(++ivCurrent), *(++ivLast), imgL2, img2, imgR2);
    }
  }

  // **** Now continue until the end of the image
  int yEnd = height;
  for(; srcY != yEnd; ++srcY)
  {
    IntermediateValues* ivCurrent = &iv[srcY & 1][0];
    IntermediateValues* ivLast = &iv[1 - (srcY & 1)][0];
    const __m128i* pStart = reinterpret_cast<const __m128i*>(src + srcY * srcOfs);
    const __m128i* pEnd = (pStart + width / 16);
    short* myCns = cns + (srcY - 1) * srcOfs;

    __m128i lastSrc, src;
    const __m128i* p = pStart;
    lastSrc = src = _mm_load_si128(p); //TODO change me (prev)

    for(; p < pEnd; ++ivCurrent, ++ivLast, myCns += 8)
    {
      __m128i imgL, img, imgR;
      __m128i imgL2, img2, imgR2;
      __m128i sobelX, sobelY, gaussI;
      __m128 gaussI2A, gaussI2B;
      load2x8PixelUsingSSE(imgL, img, imgR, imgL2, img2, imgR2, lastSrc, src, ++p);
      filters(*ivCurrent, *ivLast, sobelX, sobelY, gaussI, gaussI2A, gaussI2B, imgL, img, imgR);
      cnsFormula(*reinterpret_cast<__m128i*>(myCns), sobelX, sobelY, gaussI, gaussI2A, gaussI2B, scaleF, regVarF, offset);

      filters(*(++ivCurrent), *(++ivLast), sobelX, sobelY, gaussI, gaussI2A, gaussI2B, imgL2, img2, imgR2);
      cnsFormula(*reinterpret_cast<__m128i*>(myCns += 8), sobelX, sobelY, gaussI, gaussI2A, gaussI2B, scaleF, regVarF, offset);
    }

    // Left and right margin: set cns to offset (means 0) and ds to the source pixel
    myCns[-1] = myCns[-width] = static_cast<short>(static_cast<unsigned short>(CNSResponse::OFFSET + (CNSResponse::OFFSET << 8)));
  }

  // **** Finally set the top and bottom margin in the cns output if necessary
  fillWithCNSOffsetUsingSSE(cns, width);
  fillWithCNSOffsetUsingSSE(cns + (height - 1) * srcOfs, width);
}

void CNSImageProvider::update(CNSImage& cnsImage)
{
  DECLARE_DEBUG_DRAWING("module:CNSImageProvider:expectedRadius", "drawingOnImage");

  cnsImage.setResolution(theECImage.grayscaled.width, theECImage.grayscaled.height);

  if(fullImage)
    cnsResponse(theECImage.grayscaled[0], theECImage.grayscaled.width,
                theECImage.grayscaled.height, theECImage.grayscaled.width,
                reinterpret_cast<short*>(cnsImage[0]), sqr(minContrast));
  else
    for(const Boundaryi& region : theCNSRegions.regions)
      cnsResponse(&theECImage.grayscaled[region.y.min][region.x.min], region.x.getSize(),
                  region.y.getSize(), theECImage.grayscaled.width,
                  reinterpret_cast<short*>(&cnsImage[region.y.min][region.x.min]), sqr(minContrast));
}
