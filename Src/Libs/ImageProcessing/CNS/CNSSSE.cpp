#include "CNSSSE.h"
#include <cstring>

static const __m128i cns_const128V     = _mm_set1_epi8(-128);

void responseX8Y8RUsingSSE3(const CNSResponse* __restrict srcPixel, int srcOfs,
                            signed short* __restrict responseBin, const CodedContour& contour)
{
  srcOfs /= sizeof(CNSResponse);
  assert(aligned16(responseBin));
  alignas(16) unsigned short accPixelCopy[8 * 8];
  cns_zeroAccumulator(accPixelCopy, 8 * 8);

  // Go through all contour pixels
  for(CodedContour::const_iterator ccp = contour.begin(); ccp != contour.end(); ccp++)
  {
    CodedContourPoint ccpI = *ccp;
    __m128i cosSinVal  = _mm_set1_epi16(nOfCCP(ccpI));  // put the (nx,ny) normal vector into every component
    const CNSResponse* srcRun = srcPixel + xOfCCP(ccpI) + srcOfs * yOfCCP(ccpI);
    __m128i cosPSinVal = _mm_maddubs_epi16(cns_const128V, cosSinVal);

    // The body of this loop implement 8 cns_responseX8YRUsingSSE3 calls, manually inlined
    // and interleaved in a pipelined fashion with two chunks of data (dataA, dataB) processed in parallel
    // it was necessary to do this manually as gcc 4.5.4 wasn't able to produce efficient code else
    // similar the code is much less efficient when accPixel instead of accPixelCopy is used.
    // In case of doubt check the disassembly.
    __m128i dataA, dataB;

    dataA     = _mm_loadu_si128((__m128i*)(srcRun));
    srcRun += srcOfs;

    // Line 0/1, we have loaded dataA

    dataA     = _mm_maddubs_epi16(dataA, cosSinVal);
    dataB     = _mm_loadu_si128((__m128i*)(srcRun));

    dataA     = _mm_subs_epi16(dataA, cosPSinVal);
    srcRun += srcOfs;

    dataA     = _mm_mulhi_epi16(dataA, dataA);
    dataB     = _mm_maddubs_epi16(dataB, cosSinVal);

    dataA     = _mm_adds_epu16(dataA, *(__m128i*)(accPixelCopy + 0x00));
    dataB     = _mm_subs_epi16(dataB, cosPSinVal);

    *(__m128i*)(accPixelCopy + 0x00) = dataA;
    dataB     = _mm_mulhi_epi16(dataB, dataB);

    dataB     = _mm_adds_epu16(dataB, *(__m128i*)(accPixelCopy + 0x08));
    dataA     = _mm_loadu_si128((__m128i*)(srcRun));

    srcRun += srcOfs;
    *(__m128i*)(accPixelCopy + 0x08) = dataB;

    // Line 2/3, we have loaded dataA

    dataA     = _mm_maddubs_epi16(dataA, cosSinVal);
    dataB     = _mm_loadu_si128((__m128i*)(srcRun));

    dataA     = _mm_subs_epi16(dataA, cosPSinVal);
    srcRun += srcOfs;

    dataA     = _mm_mulhi_epi16(dataA, dataA);
    dataB     = _mm_maddubs_epi16(dataB, cosSinVal);

    dataA     = _mm_adds_epu16(dataA, *(__m128i*)(accPixelCopy + 0x10));
    dataB     = _mm_subs_epi16(dataB, cosPSinVal);

    *(__m128i*)(accPixelCopy + 0x10) = dataA;
    dataB     = _mm_mulhi_epi16(dataB, dataB);

    dataB     = _mm_adds_epu16(dataB, *(__m128i*)(accPixelCopy + 0x18));
    dataA     = _mm_loadu_si128((__m128i*)(srcRun));

    srcRun += srcOfs;
    *(__m128i*)(accPixelCopy + 0x18) = dataB;

    // Line 4/5, we have loaded dataA

    dataA     = _mm_maddubs_epi16(dataA, cosSinVal);
    dataB     = _mm_loadu_si128((__m128i*)(srcRun));

    dataA     = _mm_subs_epi16(dataA, cosPSinVal);
    srcRun += srcOfs;

    dataA     = _mm_mulhi_epi16(dataA, dataA);
    dataB     = _mm_maddubs_epi16(dataB, cosSinVal);

    dataA     = _mm_adds_epu16(dataA, *(__m128i*)(accPixelCopy + 0x20));
    dataB     = _mm_subs_epi16(dataB, cosPSinVal);

    *(__m128i*)(accPixelCopy + 0x20) = dataA;
    dataB     = _mm_mulhi_epi16(dataB, dataB);

    dataB     = _mm_adds_epu16(dataB, *(__m128i*)(accPixelCopy + 0x28));
    dataA     = _mm_loadu_si128((__m128i*)(srcRun));

    srcRun += srcOfs;
    *(__m128i*)(accPixelCopy + 0x28) = dataB;

    // Line 6/7, we have loaded dataA

    dataA     = _mm_maddubs_epi16(dataA, cosSinVal);
    dataB     = _mm_loadu_si128((__m128i*)(srcRun));

    dataA     = _mm_subs_epi16(dataA, cosPSinVal);

    dataA     = _mm_mulhi_epi16(dataA, dataA);
    dataB     = _mm_maddubs_epi16(dataB, cosSinVal);

    dataA     = _mm_adds_epu16(dataA, *(__m128i*)(accPixelCopy + 0x30));
    dataB     = _mm_subs_epi16(dataB, cosPSinVal);

    *(__m128i*)(accPixelCopy + 0x30) = dataA;
    dataB     = _mm_mulhi_epi16(dataB, dataB);

    dataB     = _mm_adds_epu16(dataB, *(__m128i*)(accPixelCopy + 0x38));

    *(__m128i*)(accPixelCopy + 0x38) = dataB;

    // Finished
  }

  //cns_copyAccumulator ((unsigned short*) responseBin, (unsigned short*) accPixelCopy, 8*8);
  scaleOffsetUsingSSE(accPixelCopy, static_cast<signed short*>(responseBin), 8 * 8, contour.mapping.rawBin2FinalBinScale, contour.mapping.rawBin2FinalBinOffset);
}

void responseX16Y16RUsingSSE3(const CNSResponse* __restrict srcPixel, int srcOfs,
                              signed short* __restrict responseBin, const CodedContour& contour)
{
  srcOfs /= sizeof(CNSResponse);
  assert(aligned16(responseBin));
  alignas(16) unsigned short accPixelCopy[16 * 16];
  cns_zeroAccumulator(accPixelCopy, 16 * 16);

  // Go through all contour pixels
  for(CodedContour::const_iterator ccp = contour.begin(); ccp != contour.end(); ccp++)
  {
    CodedContourPoint ccpI = *ccp;
    __m128i cosSinVal  = _mm_set1_epi16(nOfCCP(ccpI));  // put the (nx,ny) normal vector into every component
    const CNSResponse* srcRun = srcPixel + xOfCCP(ccpI) + srcOfs * yOfCCP(ccpI);
    __m128i cosPSinVal = _mm_maddubs_epi16(cns_const128V, cosSinVal);

    // The body of this loop implement 2*16 cns_responseX8YRUsingSSE3 calls, manually inlined
    // and interleaved in a pipelined fashion with two chunks of data (dataA, dataB) processed in parallel
    // it was necessary to do this manually as gcc 4.5.4 wasn't able to produce efficient code else
    // similar the code is much less efficient when accPixel instead of accPixelCopy is used.
    // In case of doubt check the disassembly.
    __m128i dataA, dataB;

    dataA     = _mm_loadu_si128((__m128i*)(srcRun + 0));

    // Line 0, we have loaded dataA

    dataA     = _mm_maddubs_epi16(dataA, cosSinVal);
    dataB     = _mm_loadu_si128((__m128i*)(srcRun + 0x08));

    dataA     = _mm_subs_epi16(dataA, cosPSinVal);
    srcRun += srcOfs;

    dataA     = _mm_mulhi_epi16(dataA, dataA);
    dataB     = _mm_maddubs_epi16(dataB, cosSinVal);

    dataA     = _mm_adds_epu16(dataA, *(__m128i*)(accPixelCopy + 0x00));
    dataB     = _mm_subs_epi16(dataB, cosPSinVal);

    *(__m128i*)(accPixelCopy + 0x00) = dataA;
    dataB     = _mm_mulhi_epi16(dataB, dataB);

    dataB     = _mm_adds_epu16(dataB, *(__m128i*)(accPixelCopy + 0x08));
    dataA     = _mm_loadu_si128((__m128i*)(srcRun + 0));

    *(__m128i*)(accPixelCopy + 0x08) = dataB;

    // Line 1, we have loaded dataA

    dataA     = _mm_maddubs_epi16(dataA, cosSinVal);
    dataB     = _mm_loadu_si128((__m128i*)(srcRun + 0x08));

    dataA     = _mm_subs_epi16(dataA, cosPSinVal);
    srcRun += srcOfs;

    dataA     = _mm_mulhi_epi16(dataA, dataA);
    dataB     = _mm_maddubs_epi16(dataB, cosSinVal);

    dataA     = _mm_adds_epu16(dataA, *(__m128i*)(accPixelCopy + 0x10));
    dataB     = _mm_subs_epi16(dataB, cosPSinVal);

    *(__m128i*)(accPixelCopy + 0x10) = dataA;
    dataB     = _mm_mulhi_epi16(dataB, dataB);

    dataB     = _mm_adds_epu16(dataB, *(__m128i*)(accPixelCopy + 0x18));
    dataA     = _mm_loadu_si128((__m128i*)(srcRun + 0));

    *(__m128i*)(accPixelCopy + 0x18) = dataB;

    // Line 2, we have loaded dataA

    dataA     = _mm_maddubs_epi16(dataA, cosSinVal);
    dataB     = _mm_loadu_si128((__m128i*)(srcRun + 0x08));

    dataA     = _mm_subs_epi16(dataA, cosPSinVal);
    srcRun += srcOfs;

    dataA     = _mm_mulhi_epi16(dataA, dataA);
    dataB     = _mm_maddubs_epi16(dataB, cosSinVal);

    dataA     = _mm_adds_epu16(dataA, *(__m128i*)(accPixelCopy + 0x20));
    dataB     = _mm_subs_epi16(dataB, cosPSinVal);

    *(__m128i*)(accPixelCopy + 0x20) = dataA;
    dataB     = _mm_mulhi_epi16(dataB, dataB);

    dataB     = _mm_adds_epu16(dataB, *(__m128i*)(accPixelCopy + 0x28));
    dataA     = _mm_loadu_si128((__m128i*)(srcRun + 0));

    *(__m128i*)(accPixelCopy + 0x28) = dataB;

    // Line 3, we have loaded dataA

    dataA     = _mm_maddubs_epi16(dataA, cosSinVal);
    dataB     = _mm_loadu_si128((__m128i*)(srcRun + 0x08));

    dataA     = _mm_subs_epi16(dataA, cosPSinVal);
    srcRun += srcOfs;

    dataA     = _mm_mulhi_epi16(dataA, dataA);
    dataB     = _mm_maddubs_epi16(dataB, cosSinVal);

    dataA     = _mm_adds_epu16(dataA, *(__m128i*)(accPixelCopy + 0x30));
    dataB     = _mm_subs_epi16(dataB, cosPSinVal);

    *(__m128i*)(accPixelCopy + 0x30) = dataA;
    dataB     = _mm_mulhi_epi16(dataB, dataB);

    dataB     = _mm_adds_epu16(dataB, *(__m128i*)(accPixelCopy + 0x38));
    dataA     = _mm_loadu_si128((__m128i*)(srcRun + 0));

    *(__m128i*)(accPixelCopy + 0x38) = dataB;

    // Line 4, we have loaded dataA

    dataA     = _mm_maddubs_epi16(dataA, cosSinVal);
    dataB     = _mm_loadu_si128((__m128i*)(srcRun + 0x08));

    dataA     = _mm_subs_epi16(dataA, cosPSinVal);
    srcRun += srcOfs;

    dataA     = _mm_mulhi_epi16(dataA, dataA);
    dataB     = _mm_maddubs_epi16(dataB, cosSinVal);

    dataA     = _mm_adds_epu16(dataA, *(__m128i*)(accPixelCopy + 0x40));
    dataB     = _mm_subs_epi16(dataB, cosPSinVal);

    *(__m128i*)(accPixelCopy + 0x40) = dataA;
    dataB     = _mm_mulhi_epi16(dataB, dataB);

    dataB     = _mm_adds_epu16(dataB, *(__m128i*)(accPixelCopy + 0x48));
    dataA     = _mm_loadu_si128((__m128i*)(srcRun + 0));

    *(__m128i*)(accPixelCopy + 0x48) = dataB;

    // Line 5, we have loaded dataA

    dataA     = _mm_maddubs_epi16(dataA, cosSinVal);
    dataB     = _mm_loadu_si128((__m128i*)(srcRun + 0x08));

    dataA     = _mm_subs_epi16(dataA, cosPSinVal);
    srcRun += srcOfs;

    dataA     = _mm_mulhi_epi16(dataA, dataA);
    dataB     = _mm_maddubs_epi16(dataB, cosSinVal);

    dataA     = _mm_adds_epu16(dataA, *(__m128i*)(accPixelCopy + 0x50));
    dataB     = _mm_subs_epi16(dataB, cosPSinVal);

    *(__m128i*)(accPixelCopy + 0x50) = dataA;
    dataB     = _mm_mulhi_epi16(dataB, dataB);

    dataB     = _mm_adds_epu16(dataB, *(__m128i*)(accPixelCopy + 0x58));
    dataA     = _mm_loadu_si128((__m128i*)(srcRun + 0));

    *(__m128i*)(accPixelCopy + 0x58) = dataB;

    // Line 6, we have loaded dataA

    dataA     = _mm_maddubs_epi16(dataA, cosSinVal);
    dataB     = _mm_loadu_si128((__m128i*)(srcRun + 0x08));

    dataA     = _mm_subs_epi16(dataA, cosPSinVal);
    srcRun += srcOfs;

    dataA     = _mm_mulhi_epi16(dataA, dataA);
    dataB     = _mm_maddubs_epi16(dataB, cosSinVal);

    dataA     = _mm_adds_epu16(dataA, *(__m128i*)(accPixelCopy + 0x60));
    dataB     = _mm_subs_epi16(dataB, cosPSinVal);

    *(__m128i*)(accPixelCopy + 0x60) = dataA;
    dataB     = _mm_mulhi_epi16(dataB, dataB);

    dataB     = _mm_adds_epu16(dataB, *(__m128i*)(accPixelCopy + 0x68));
    dataA     = _mm_loadu_si128((__m128i*)(srcRun + 0));

    *(__m128i*)(accPixelCopy + 0x68) = dataB;

    // Line 7, we have loaded dataA

    dataA     = _mm_maddubs_epi16(dataA, cosSinVal);
    dataB     = _mm_loadu_si128((__m128i*)(srcRun + 0x08));

    dataA     = _mm_subs_epi16(dataA, cosPSinVal);
    srcRun += srcOfs;

    dataA     = _mm_mulhi_epi16(dataA, dataA);
    dataB     = _mm_maddubs_epi16(dataB, cosSinVal);

    dataA     = _mm_adds_epu16(dataA, *(__m128i*)(accPixelCopy + 0x70));
    dataB     = _mm_subs_epi16(dataB, cosPSinVal);

    *(__m128i*)(accPixelCopy + 0x70) = dataA;
    dataB     = _mm_mulhi_epi16(dataB, dataB);

    dataB     = _mm_adds_epu16(dataB, *(__m128i*)(accPixelCopy + 0x78));
    dataA     = _mm_loadu_si128((__m128i*)(srcRun + 0));

    *(__m128i*)(accPixelCopy + 0x78) = dataB;

    // Line 8, we have loaded dataA

    dataA     = _mm_maddubs_epi16(dataA, cosSinVal);
    dataB     = _mm_loadu_si128((__m128i*)(srcRun + 0x08));

    dataA     = _mm_subs_epi16(dataA, cosPSinVal);
    srcRun += srcOfs;

    dataA     = _mm_mulhi_epi16(dataA, dataA);
    dataB     = _mm_maddubs_epi16(dataB, cosSinVal);

    dataA     = _mm_adds_epu16(dataA, *(__m128i*)(accPixelCopy + 0x80));
    dataB     = _mm_subs_epi16(dataB, cosPSinVal);

    *(__m128i*)(accPixelCopy + 0x80) = dataA;
    dataB     = _mm_mulhi_epi16(dataB, dataB);

    dataB     = _mm_adds_epu16(dataB, *(__m128i*)(accPixelCopy + 0x88));
    dataA     = _mm_loadu_si128((__m128i*)(srcRun + 0));

    *(__m128i*)(accPixelCopy + 0x88) = dataB;

    // Line 9, we have loaded dataA

    dataA     = _mm_maddubs_epi16(dataA, cosSinVal);
    dataB     = _mm_loadu_si128((__m128i*)(srcRun + 0x08));

    dataA     = _mm_subs_epi16(dataA, cosPSinVal);
    srcRun += srcOfs;

    dataA     = _mm_mulhi_epi16(dataA, dataA);
    dataB     = _mm_maddubs_epi16(dataB, cosSinVal);

    dataA     = _mm_adds_epu16(dataA, *(__m128i*)(accPixelCopy + 0x90));
    dataB     = _mm_subs_epi16(dataB, cosPSinVal);

    *(__m128i*)(accPixelCopy + 0x90) = dataA;
    dataB     = _mm_mulhi_epi16(dataB, dataB);

    dataB     = _mm_adds_epu16(dataB, *(__m128i*)(accPixelCopy + 0x98));
    dataA     = _mm_loadu_si128((__m128i*)(srcRun + 0));

    *(__m128i*)(accPixelCopy + 0x98) = dataB;

    // Line A, we have loaded dataA

    dataA     = _mm_maddubs_epi16(dataA, cosSinVal);
    dataB     = _mm_loadu_si128((__m128i*)(srcRun + 0x08));

    dataA     = _mm_subs_epi16(dataA, cosPSinVal);
    srcRun += srcOfs;

    dataA     = _mm_mulhi_epi16(dataA, dataA);
    dataB     = _mm_maddubs_epi16(dataB, cosSinVal);

    dataA     = _mm_adds_epu16(dataA, *(__m128i*)(accPixelCopy + 0xA0));
    dataB     = _mm_subs_epi16(dataB, cosPSinVal);

    *(__m128i*)(accPixelCopy + 0xA0) = dataA;
    dataB     = _mm_mulhi_epi16(dataB, dataB);

    dataB     = _mm_adds_epu16(dataB, *(__m128i*)(accPixelCopy + 0xA8));
    dataA     = _mm_loadu_si128((__m128i*)(srcRun + 0));

    *(__m128i*)(accPixelCopy + 0xA8) = dataB;

    // Line B, we have loaded dataA

    dataA     = _mm_maddubs_epi16(dataA, cosSinVal);
    dataB     = _mm_loadu_si128((__m128i*)(srcRun + 0x08));

    dataA     = _mm_subs_epi16(dataA, cosPSinVal);
    srcRun += srcOfs;

    dataA     = _mm_mulhi_epi16(dataA, dataA);
    dataB     = _mm_maddubs_epi16(dataB, cosSinVal);

    dataA     = _mm_adds_epu16(dataA, *(__m128i*)(accPixelCopy + 0xB0));
    dataB     = _mm_subs_epi16(dataB, cosPSinVal);

    *(__m128i*)(accPixelCopy + 0xB0) = dataA;
    dataB     = _mm_mulhi_epi16(dataB, dataB);

    dataB     = _mm_adds_epu16(dataB, *(__m128i*)(accPixelCopy + 0xB8));
    dataA     = _mm_loadu_si128((__m128i*)(srcRun + 0));

    *(__m128i*)(accPixelCopy + 0xB8) = dataB;

    // Line C, we have loaded dataA

    dataA     = _mm_maddubs_epi16(dataA, cosSinVal);
    dataB     = _mm_loadu_si128((__m128i*)(srcRun + 0x08));

    dataA     = _mm_subs_epi16(dataA, cosPSinVal);
    srcRun += srcOfs;

    dataA     = _mm_mulhi_epi16(dataA, dataA);
    dataB     = _mm_maddubs_epi16(dataB, cosSinVal);

    dataA     = _mm_adds_epu16(dataA, *(__m128i*)(accPixelCopy + 0xC0));
    dataB     = _mm_subs_epi16(dataB, cosPSinVal);

    *(__m128i*)(accPixelCopy + 0xC0) = dataA;
    dataB     = _mm_mulhi_epi16(dataB, dataB);

    dataB     = _mm_adds_epu16(dataB, *(__m128i*)(accPixelCopy + 0xC8));
    dataA     = _mm_loadu_si128((__m128i*)(srcRun + 0));

    *(__m128i*)(accPixelCopy + 0xC8) = dataB;

    // Line D, we have loaded dataA

    dataA     = _mm_maddubs_epi16(dataA, cosSinVal);
    dataB     = _mm_loadu_si128((__m128i*)(srcRun + 0x08));

    dataA     = _mm_subs_epi16(dataA, cosPSinVal);
    srcRun += srcOfs;

    dataA     = _mm_mulhi_epi16(dataA, dataA);
    dataB     = _mm_maddubs_epi16(dataB, cosSinVal);

    dataA     = _mm_adds_epu16(dataA, *(__m128i*)(accPixelCopy + 0xD0));
    dataB     = _mm_subs_epi16(dataB, cosPSinVal);

    *(__m128i*)(accPixelCopy + 0xD0) = dataA;
    dataB     = _mm_mulhi_epi16(dataB, dataB);

    dataB     = _mm_adds_epu16(dataB, *(__m128i*)(accPixelCopy + 0xD8));
    dataA     = _mm_loadu_si128((__m128i*)(srcRun + 0));

    *(__m128i*)(accPixelCopy + 0xD8) = dataB;

    // Line E, we have loaded dataA

    dataA     = _mm_maddubs_epi16(dataA, cosSinVal);
    dataB     = _mm_loadu_si128((__m128i*)(srcRun + 0x08));

    dataA     = _mm_subs_epi16(dataA, cosPSinVal);
    srcRun += srcOfs;

    dataA     = _mm_mulhi_epi16(dataA, dataA);
    dataB     = _mm_maddubs_epi16(dataB, cosSinVal);

    dataA     = _mm_adds_epu16(dataA, *(__m128i*)(accPixelCopy + 0xE0));
    dataB     = _mm_subs_epi16(dataB, cosPSinVal);

    *(__m128i*)(accPixelCopy + 0xE0) = dataA;
    dataB     = _mm_mulhi_epi16(dataB, dataB);

    dataB     = _mm_adds_epu16(dataB, *(__m128i*)(accPixelCopy + 0xE8));
    dataA     = _mm_loadu_si128((__m128i*)(srcRun + 0));

    *(__m128i*)(accPixelCopy + 0xE8) = dataB;

    // Line F, we have loaded dataA

    dataA     = _mm_maddubs_epi16(dataA, cosSinVal);
    dataB     = _mm_loadu_si128((__m128i*)(srcRun + 0x08));

    dataA     = _mm_subs_epi16(dataA, cosPSinVal);

    dataA     = _mm_mulhi_epi16(dataA, dataA);
    dataB     = _mm_maddubs_epi16(dataB, cosSinVal);

    dataA     = _mm_adds_epu16(dataA, *(__m128i*)(accPixelCopy + 0xF0));
    dataB     = _mm_subs_epi16(dataB, cosPSinVal);

    *(__m128i*)(accPixelCopy + 0xF0) = dataA;
    dataB     = _mm_mulhi_epi16(dataB, dataB);

    dataB     = _mm_adds_epu16(dataB, *(__m128i*)(accPixelCopy + 0xF8));

    *(__m128i*)(accPixelCopy + 0xF8) = dataB;

    // Finished
  }

  //cns_copyAccumulator ((unsigned short*) responseBin, (unsigned short*) accPixelCopy, 16*16);
  scaleOffsetUsingSSE(accPixelCopy, static_cast<signed short*>(responseBin), 16 * 16, contour.mapping.rawBin2FinalBinScale, contour.mapping.rawBin2FinalBinOffset);
}
