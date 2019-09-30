/**
 * @file Tools/Debugging/DebugImages.cpp
 *
 * @author Felix Thielke
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "DebugImages.h"
#include "Tools/ImageProcessing/AVX.h"
#include "Tools/ImageProcessing/ColorModelConversions.h"
#include "Tools/Global.h"
#include <asmjit/asmjit.h>

using namespace asmjit;

void yuvToBGRA(unsigned int size, const void* const src, void* const dest)
{
  static constexpr bool avx = _supportsAVX2;
  static const __m_auto_i yMask = _mmauto_set1_epi32(0x000000FF);
  static const __m_auto_i channelMask = _mmauto_set1_epi16(0x00FF);
  static const __m_auto_i c_128 = _mmauto_set1_epi16(128);
  static const __m_auto_i c_0 = _mmauto_setzero_si_all();
  static const __m_auto_i alpha = _mmauto_set1_epi8(static_cast<unsigned char>(0xFF));
  static const __m_auto_i scaledInvUCoeff = _mmauto_set1_epi16(static_cast<short>(static_cast<float>(1 << 14) / ColorModelConversions::uCoeff));
  static const __m_auto_i scaledInvVCoeff = _mmauto_set1_epi16(static_cast<short>(static_cast<float>(1 << 14) / ColorModelConversions::vCoeff));
  static const __m_auto_i scaledGCoeffU = _mmauto_set1_epi16(static_cast<short>(ColorModelConversions::yCoeffB / (ColorModelConversions::yCoeffG * ColorModelConversions::uCoeff) * static_cast<float>(1 << 15)));
  static const __m_auto_i scaledGCoeffV = _mmauto_set1_epi16(static_cast<short>(ColorModelConversions::yCoeffR / (ColorModelConversions::yCoeffG * ColorModelConversions::vCoeff) * static_cast<float>(1 << 15)));
  const __m_auto_i* pSrc = reinterpret_cast<const __m_auto_i*>(src);
  __m_auto_i* pDest = reinterpret_cast<__m_auto_i*>(dest);
  for(unsigned int n = (size + (avx ? 63 : 31)) / (avx ? 64 : 32); n; --n)
  {
    const __m_auto_i p0 = _mmauto_loadt_si_all<true>(pSrc++);
    const __m_auto_i p1 = _mmauto_loadt_si_all<true>(pSrc++);

    const __m_auto_i y = _mmauto_correct_256op(_mmauto_packs_epi32(_mmauto_and_si_all(_mmauto_srli_epi32(p0, 16), yMask), _mmauto_and_si_all(_mmauto_srli_epi32(p1, 16), yMask)));

    const __m_auto_i uv0 = _mmauto_sub_epi16(_mmauto_and_si_all(_mmauto_srli_si_all(p0, 1), channelMask), c_128);
    const __m_auto_i uv1 = _mmauto_sub_epi16(_mmauto_and_si_all(_mmauto_srli_si_all(p1, 1), channelMask), c_128);
    const __m_auto_i u = _mmauto_correct_256op(_mmauto_packs_epi32(_mmauto_srai_epi32(_mmauto_slli_epi32(uv0, 16), 16), _mmauto_srai_epi32(_mmauto_slli_epi32(uv1, 16), 16)));
    const __m_auto_i v = _mmauto_correct_256op(_mmauto_packs_epi32(_mmauto_srai_epi32(uv0, 16), _mmauto_srai_epi32(uv1, 16)));

    const __m_auto_i b = _mmauto_correct_256op(_mmauto_packus_epi16(_mmauto_add_epi16(y, _mmauto_slli_epi16(_mmauto_mulhrs_epi16(u, scaledInvUCoeff), 1)), c_0));
    const __m_auto_i g = _mmauto_correct_256op(_mmauto_packus_epi16(_mmauto_sub_epi16(y, _mmauto_add_epi16(_mmauto_mulhrs_epi16(u, scaledGCoeffU), _mmauto_mulhrs_epi16(v, scaledGCoeffV))), c_0));
    const __m_auto_i r = _mmauto_correct_256op(_mmauto_packus_epi16(_mmauto_add_epi16(y, _mmauto_slli_epi32(_mmauto_mulhrs_epi16(v, scaledInvVCoeff), 1)), c_0));

    if(avx)
    {
      __m_auto_i bgra0 = b;
      __m_auto_i tmp = g;
      _mmauto_unpacklohi_epi8(bgra0, tmp);
      __m_auto_i bgra1 = r;
      tmp = alpha;
      _mmauto_unpacklohi_epi8(bgra1, tmp);
      _mmauto_unpacklohi_epi16(bgra0, bgra1);
      _mmauto_storet_si_all<true>(pDest++, bgra0);
      _mmauto_storet_si_all<true>(pDest++, bgra1);
    }
    else
    {
      const __m_auto_i bg = _mmauto_unpacklo_epi8(b, g);
      const __m_auto_i ra = _mmauto_unpacklo_epi8(r, alpha);
      _mmauto_storet_si_all<true>(pDest++, _mmauto_unpacklo_epi16(bg, ra));
      _mmauto_storet_si_all<true>(pDest++, _mmauto_unpackhi_epi16(bg, ra));
    }
  }
}

template<bool avx> void ALWAYSINLINE storeColors(__m_auto_i* dest, const __m_auto_i p)
{
  alignas(avx ? 32 : 16)static __m_auto_i colors[FieldColors::numOfColors] =
  {
    _mmauto_set1_epi32(0xff7f7f7f), //none
    _mmauto_set1_epi32(0xffffffff), //white
    _mmauto_set1_epi32(0xff000000), //black
    _mmauto_set1_epi32(0xff00ff00) //green
  };

  __m_auto_i result = _mmauto_setzero_si_all();

  FOREACH_ENUM(FieldColors::Color, i)
    result = _mmauto_or_si_all(result, _mmauto_and_si_all(_mmauto_cmpeq_epi32(p, _mmauto_set1_epi32(i)), colors[i]));

  _mmauto_storet_si_all<true>(dest, result);
}

void coloredToBGRA(const unsigned int size, const void* const src, void* const dest)
{
  static constexpr bool avx = _supportsAVX2;
  static const __m_auto_i c_0 = _mmauto_setzero_si_all();
  const __m_auto_i* pSrc = reinterpret_cast<const __m_auto_i*>(src);
  __m_auto_i* pDest = reinterpret_cast<__m_auto_i*>(dest);
  for(unsigned int n = (size + (avx ? 31 : 15)) / (avx ? 32 : 16); n; --n)
  {
    const __m_auto_i p = _mmauto_loadt_si_all<true>(pSrc++);

    __m_auto_i pLo = p;
    __m_auto_i pHi = c_0;
    _mmauto_unpacklohi_epi8(pLo, pHi);

    __m_auto_i pLo2 = pLo;
    __m_auto_i pHi2 = c_0;
    _mmauto_unpacklohi_epi8(pLo2, pHi2);
    storeColors<avx>(pDest++, pLo2);
    storeColors<avx>(pDest++, pHi2);

    pLo2 = pHi;
    pHi2 = c_0;
    _mmauto_unpacklohi_epi8(pLo2, pHi2);
    storeColors<avx>(pDest++, pLo2);
    storeColors<avx>(pDest++, pHi2);
  }
}

void grayscaledToBGRA(const unsigned int size, const void* const src, void* const dest)
{
  static constexpr bool avx = _supportsAVX2;
  static const __m_auto_i alpha = _mmauto_set1_epi32(0xFF000000);
  const __m_auto_i* pSrc = reinterpret_cast<const __m_auto_i*>(src);
  __m_auto_i* pDest = reinterpret_cast<__m_auto_i*>(dest);
  for(unsigned int n = (size + (avx ? 31 : 15)) / (avx ? 32 : 16); n; --n)
  {
    const __m_auto_i p = _mmauto_loadt_si_all<true>(pSrc++);

    __m_auto_i pLo = p;
    __m_auto_i pHi = p;
    _mmauto_unpacklohi_epi8(pLo, pHi);

    __m_auto_i pLo2 = pLo;
    __m_auto_i pHi2 = pLo;
    _mmauto_unpacklohi_epi8(pLo2, pHi2);
    _mmauto_storet_si_all<true>(pDest++, _mmauto_or_si_all(pLo2, alpha));
    _mmauto_storet_si_all<true>(pDest++, _mmauto_or_si_all(pHi2, alpha));

    pLo2 = pHi;
    pHi2 = pHi;
    _mmauto_unpacklohi_epi8(pLo2, pHi2);
    _mmauto_storet_si_all<true>(pDest++, _mmauto_or_si_all(pLo2, alpha));
    _mmauto_storet_si_all<true>(pDest++, _mmauto_or_si_all(pHi2, alpha));
  }
}

void binaryToBGRA(const unsigned int size, const void* const src, void* const dest)
{
  static constexpr bool avx = _supportsAVX2;
  static const __m_auto_i alpha = _mmauto_set1_epi32(0xFF000000);
  const __m_auto_i* pSrc = reinterpret_cast<const __m_auto_i*>(src);
  __m_auto_i* pDest = reinterpret_cast<__m_auto_i*>(dest);
  for(unsigned int n = (size + (avx ? 31 : 15)) / (avx ? 32 : 16); n; --n)
  {
    const __m_auto_i p = _mmauto_cmpgt_epi8(_mmauto_loadt_si_all<true>(pSrc++), _mmauto_setzero_si_all());

    __m_auto_i pLo = p;
    __m_auto_i pHi = p;
    _mmauto_unpacklohi_epi8(pLo, pHi);

    __m_auto_i pLo2 = pLo;
    __m_auto_i pHi2 = pLo;
    _mmauto_unpacklohi_epi8(pLo2, pHi2);
    _mmauto_storet_si_all<true>(pDest++, _mmauto_or_si_all(pLo2, alpha));
    _mmauto_storet_si_all<true>(pDest++, _mmauto_or_si_all(pHi2, alpha));

    pLo2 = pHi;
    pHi2 = pHi;
    _mmauto_unpacklohi_epi8(pLo2, pHi2);
    _mmauto_storet_si_all<true>(pDest++, _mmauto_or_si_all(pLo2, alpha));
    _mmauto_storet_si_all<true>(pDest++, _mmauto_or_si_all(pHi2, alpha));
  }
}

void hueToBGRA(const unsigned int size, const PixelTypes::HuePixel* src, PixelTypes::BGRAPixel* dest)
{
  for(unsigned int n = size; n; src++, dest++, --n)
  {
    ColorModelConversions::fromYUVToRGB(128, static_cast<unsigned char>(static_cast<float>(std::cos(static_cast<float>(*src) * 360_deg / 256.f)) * 128.f + 128.f), static_cast<unsigned char>(static_cast<float>(std::sin(static_cast<float>(*src) * 360_deg / 256.f)) * 128.f + 128.f), dest->r, dest->g, dest->b);
    dest->a = 0xFF;
  }
}

void edge2ToBGRA(const unsigned int size, const void* const src, void* const dest)
{
  static constexpr bool avx = _supportsAVX2;
  static const __m_auto_i offset = _mmauto_set1_epi8(char(128));
  const __m_auto_i* pSrc = reinterpret_cast<const __m_auto_i*>(src);
  __m_auto_i* pDest = reinterpret_cast<__m_auto_i*>(dest);
  for(unsigned int n = (size + (avx ? 31 : 15)) / (avx ? 32 : 16); n; --n)
  {
    const __m_auto_i p = _mmauto_loadt_si_all<true>(pSrc++);
    const __m_auto_i offsetCorrected = _mmauto_abs_epi8(_mmauto_sub_epi8(p, offset));
    __m_auto_i res0 = _mmauto_slli_epi16(_mmauto_sqrt_epu16<avx>(_mmauto_slli_epi16(_mmauto_maddubs_epi16(offsetCorrected, offsetCorrected), 1)), 8);
    __m_auto_i res1 = p;
    _mmauto_unpacklohi_epi8(res0, res1);
    _mmauto_storet_si_all<true>(pDest++, res0);
    _mmauto_storet_si_all<true>(pDest++, res1);
  }

  yuvToBGRA(size * 2, dest, dest);
}

void asmReturn(x86::Assembler& a)
{
#if ASMJIT_ARCH_X86 != 64 || defined(_WIN32)
  a.pop(a.zsi());
  a.pop(a.zdi());
#endif
  a.pop(a.zbx());
  a.mov(a.zsp(), a.zbp());
  a.pop(a.zbp());
  a.ret();
}

void rgbToBGRA(x86::Assembler& a)
{
  Label constants = a.newLabel();
  Label loop = a.newLabel();

  // Process 32 / 16 bytes at a time
#if _supportsAVX2
  a.add(x86::edi, 31);
  a.shr(x86::edi, 5);
  a.vbroadcasti128(x86::ymm0, x86::Mem(constants, 0));
  a.vpbroadcastd(x86::ymm1, x86::Mem(constants, 16));
  a.bind(loop);
  a.vmovdqa(x86::ymm2, x86::ptr(a.zsi()));
  a.add(a.zsi(), 32);
  a.vpshufb(x86::ymm2, x86::ymm2, x86::ymm0);
  a.vpor(x86::ymm2, x86::ymm2, x86::ymm1);
  a.vmovntdq(x86::ptr(a.zdx()), x86::ymm2);
  a.add(a.zdx(), 32);
  a.dec(x86::edi);
  a.jnz(loop);
#else
  a.add(x86::edi, 15);
  a.shr(x86::edi, 4);
  a.movdqa(x86::xmm0, x86::Mem(constants, 0));
  a.movdqa(x86::xmm1, x86::Mem(constants, 16));
  a.bind(loop);
  a.movdqa(x86::xmm2, x86::ptr(a.zsi()));
  a.add(a.zsi(), 16);
  a.pshufb(x86::xmm2, x86::xmm0);
  a.por(x86::xmm2, x86::xmm1);
  a.movntdq(x86::ptr(a.zdx()), x86::xmm2);
  a.add(a.zdx(), 16);
  a.dec(x86::edi);
  a.jnz(loop);
#endif

  // Return
  asmReturn(a);

  // Constants
  a.align(AlignMode::kAlignZero, 16);
  a.bind(constants);
  // 0: shuffleMask
  a.dint8(2);
  a.dint8(1);
  a.dint8(0);
  a.dint8(char(0xFF));
  a.dint8(6);
  a.dint8(5);
  a.dint8(4);
  a.dint8(char(0xFF));
  a.dint8(10);
  a.dint8(9);
  a.dint8(8);
  a.dint8(char(0xFF));
  a.dint8(14);
  a.dint8(13);
  a.dint8(12);
  a.dint8(char(0xFF));
  for(size_t i = 0; i < (_supportsAVX2 ? 1 : 4); i++) a.dint32(0xFF000000); // 1: alpha
}

void yuyvToBGRA(x86::Assembler& a)
{
  Label constants = a.newLabel();
  Label loop = a.newLabel();

  // Process 32 / 16 bytes at a time
#if _supportsAVX2
  a.add(x86::edi, 31);
  a.shr(x86::edi, 5);
  a.vbroadcasti128(x86::ymm5, x86::Mem(constants, 0));
  a.vpbroadcastw(x86::ymm6, x86::Mem(constants, 16));
  a.vpbroadcastw(x86::ymm7, x86::Mem(constants, 18));
  a.vpbroadcastd(x86::ymm8, x86::Mem(constants, 20));
  a.vpbroadcastd(x86::ymm9, x86::Mem(constants, 24));
  a.vpcmpeqb(x86::ymm10, x86::ymm10, x86::ymm10);
  a.bind(loop);
  a.vmovdqa(x86::ymm0, x86::ptr(a.zsi()));
  a.add(a.zsi(), 32);
  a.vpsrlw(x86::ymm1, x86::ymm0, 8);
  a.vpsubw(x86::ymm1, x86::ymm1, x86::ymm6); // YMM1 is now 16-bit UV
  a.vpand(x86::ymm0, x86::ymm0, x86::ymm7); // YMM0 is now 16-bit Y
  a.vpmulhrsw(x86::ymm2, x86::ymm1, x86::ymm8);
  a.vpsllw(x86::ymm2, x86::ymm2, 1); // YMM2 is now 16-bit BR without luminance-shift
  a.vpunpckhwd(x86::ymm3, x86::ymm0, x86::ymm0);
  a.vpunpckhdq(x86::ymm4, x86::ymm2, x86::ymm2);
  a.vpaddw(x86::ymm3, x86::ymm3, x86::ymm4); // YMM3 is now 16-bit BR1
  a.vpunpcklwd(x86::ymm4, x86::ymm0, x86::ymm0);
  a.vpunpckldq(x86::ymm2, x86::ymm2, x86::ymm2);
  a.vpaddw(x86::ymm2, x86::ymm2, x86::ymm4); // YMM2 is now 16-bit BR0
  a.vpackuswb(x86::ymm2, x86::ymm2, x86::ymm3); // YMM2 is now 8-bit BR
  a.vpmulhrsw(x86::ymm1, x86::ymm1, x86::ymm9);
  a.vphaddw(x86::ymm1, x86::ymm1, x86::ymm1);
  a.vpshufb(x86::ymm1, x86::ymm1, x86::ymm5);
  a.vpsubw(x86::ymm0, x86::ymm0, x86::ymm1); // YMM0 is now 16-bit G
  a.vpackuswb(x86::ymm0, x86::ymm0, x86::ymm0);
  a.vpunpcklbw(x86::ymm0, x86::ymm0, x86::ymm10); // YMM0 is now 8-bit GA
  a.vpunpcklbw(x86::ymm1, x86::ymm2, x86::ymm0); // YMM1 is now: BGRA0 BGRA1 BGRA2 BGRA3 BGRA8 BGRA9 BGRA10 BGRA11
  a.vpunpckhbw(x86::ymm2, x86::ymm2, x86::ymm0); // YMM2 is now: BGRA4 BGRA5 BGRA6 BGRA7 BGRA12 BGRA13 BGRA14 BGRA15
  a.vperm2i128(x86::ymm0, x86::ymm1, x86::ymm2, 2 << 4); // YMM0 is now BGRA0 BGRA1 BGRA2 BGRA3 BGRA4 BGRA5 BGRA6 BGRA7
  a.vperm2i128(x86::ymm1, x86::ymm1, x86::ymm2, 1 | (3 << 4)); // YMM1 is now BGRA8 BGRA9 BGRA10 BGRA11 BGRA12 BGRA13 BGRA14 BGRA15
  a.vmovntdq(x86::ptr(a.zdx()), x86::ymm0);
  a.vmovntdq(x86::Mem(a.zdx(), 32), x86::ymm1);
  a.add(a.zdx(), 64);
  a.dec(x86::edi);
  a.jnz(loop);
#else
  a.add(x86::edi, 15);
  a.shr(x86::edi, 4);
  a.movdqa(x86::xmm5, x86::Mem(constants, 0));
  a.movdqa(x86::xmm6, x86::Mem(constants, 16));
  a.movdqa(x86::xmm7, x86::Mem(constants, 16 * 2));
#if ASMJIT_ARCH_X86 == 64
  a.movdqa(x86::xmm8, x86::Mem(constants, 16 * 3));
  a.movdqa(x86::xmm9, x86::Mem(constants, 16 * 4));
  a.pcmpeqb(x86::xmm10, x86::xmm10);
#endif
  a.bind(loop);
  a.movdqa(x86::xmm0, x86::ptr(a.zsi()));
  a.add(a.zsi(), 16);
  a.movdqa(x86::xmm1, x86::xmm0);
  a.psrlw(x86::xmm1, 8);
  a.psubw(x86::xmm1, x86::xmm6); // XMM1 is now 16-bit UV
  a.pand(x86::xmm0, x86::xmm7); // XMM0 is now 16-bit Y
#if ASMJIT_ARCH_X86 == 64
  a.movdqa(x86::xmm2, x86::xmm8);
#else
  a.movdqa(x86::xmm2, x86::Mem(constants, 16 * 3));
#endif
  a.pmulhrsw(x86::xmm2, x86::xmm1);
  a.psllw(x86::xmm2, 1); // XMM2 is now 16-bit BR without luminance-shift
  a.movdqa(x86::xmm3, x86::xmm0);
  a.movdqa(x86::xmm4, x86::xmm2);
  a.punpckhwd(x86::xmm3, x86::xmm3);
  a.punpckhdq(x86::xmm4, x86::xmm4);
  a.paddw(x86::xmm3, x86::xmm4); // XMM3 is now 16-bit BR1
  a.movdqa(x86::xmm4, x86::xmm0);
  a.punpcklwd(x86::xmm4, x86::xmm4);
  a.punpckldq(x86::xmm2, x86::xmm2);
  a.paddw(x86::xmm2, x86::xmm4); // XMM2 is now 16-bit BR0
  a.packuswb(x86::xmm2, x86::xmm3); // XMM2 is now 8-bit BR
#if ASMJIT_ARCH_X86 == 64
  a.pmulhrsw(x86::xmm1, x86::xmm9);
#else
  a.pmulhrsw(x86::xmm1, x86::Mem(constants, 16 * 4));
#endif
  a.phaddw(x86::xmm1, x86::xmm1);
  a.pshufb(x86::xmm1, x86::xmm5);
  a.psubw(x86::xmm0, x86::xmm1); // XMM0 is now 16-bit G
  a.packuswb(x86::xmm0, x86::xmm0);
#if ASMJIT_ARCH_X86 == 64
  a.punpcklbw(x86::xmm0, x86::xmm10); // XMM0 is now 8-bit GA
#else
  a.pcmpeqb(x86::xmm1, x86::xmm1);
  a.punpcklbw(x86::xmm0, x86::xmm1); // XMM0 is now 8-bit GA
#endif
  a.movdqa(x86::xmm1, x86::xmm2);
  a.punpcklbw(x86::xmm1, x86::xmm0); // XMM1 is now 8-bit BGRA0
  a.punpckhbw(x86::xmm2, x86::xmm0); // XMM2 is now 8-bit BGRA1
  a.movntdq(x86::ptr(a.zdx()), x86::xmm1);
  a.movntdq(x86::Mem(a.zdx(), 16), x86::xmm2);
  a.add(a.zdx(), 32);
  a.dec(x86::edi);
  a.jnz(loop);
#endif

  // Return
  asmReturn(a);

  // Constants
  a.align(AlignMode::kAlignZero, 16);
  a.bind(constants);
  // 0: shuffleMask
  a.dint8(0);
  a.dint8(1);
  a.dint8(8);
  a.dint8(9);
  a.dint8(2);
  a.dint8(3);
  a.dint8(10);
  a.dint8(11);
  a.dint8(4);
  a.dint8(5);
  a.dint8(12);
  a.dint8(13);
  a.dint8(6);
  a.dint8(7);
  a.dint8(14);
  a.dint8(15);
  for(size_t i = 0; i < (_supportsAVX2 ? 1 : 8); i++) a.dint16(128);    // 1: c16_128
  for(size_t i = 0; i < (_supportsAVX2 ? 1 : 8); i++) a.dint16(0x00FF); // 2: loMask16
  static constexpr short scaledBCoeff = static_cast<short>(static_cast<float>(1 << 14) / ColorModelConversions::uCoeff);
  static constexpr short scaledRCoeff = static_cast<short>(static_cast<float>(1 << 14) / ColorModelConversions::vCoeff);
  for(size_t i = 0; i < (_supportsAVX2 ? 1 : 4); i++)                    // 3: scaledBRCoeffs16
  {
    a.dint16(scaledBCoeff);
    a.dint16(scaledRCoeff);
  }
  static constexpr short scaledGCoeffU = static_cast<short>(ColorModelConversions::yCoeffB / (ColorModelConversions::yCoeffG * ColorModelConversions::uCoeff) * static_cast<float>(1 << 15));
  static constexpr short scaledGCoeffV = static_cast<short>(ColorModelConversions::yCoeffR / (ColorModelConversions::yCoeffG * ColorModelConversions::vCoeff) * static_cast<float>(1 << 15));
  for(size_t i = 0; i < (_supportsAVX2 ? 1 : 4); i++)                    // 4: scaledGCoeffs16
  {
    a.dint16(scaledGCoeffU);
    a.dint16(scaledGCoeffV);
  }
}

DebugImageConverter::DebugImageConverter()
{
  converters[PixelTypes::RGB] = nullptr;
  converters[PixelTypes::BGRA] = nullptr;
  converters[PixelTypes::YUYV] = nullptr;
  converters[PixelTypes::YUV] = static_cast<ConversionFunction>(yuvToBGRA);
  converters[PixelTypes::Colored] = static_cast<ConversionFunction>(coloredToBGRA);
  converters[PixelTypes::Grayscale] = static_cast<ConversionFunction>(grayscaledToBGRA);
  converters[PixelTypes::Hue] = reinterpret_cast<ConversionFunction>(hueToBGRA);
  converters[PixelTypes::Binary] = static_cast<ConversionFunction>(binaryToBGRA);
  converters[PixelTypes::Edge2] = static_cast<ConversionFunction>(edge2ToBGRA);
}

DebugImageConverter::~DebugImageConverter()
{
  if(converters[PixelTypes::RGB])
    Global::getAsmjitRuntime().release<ConversionFunction>(converters[PixelTypes::RGB]);
  if(converters[PixelTypes::YUYV])
    Global::getAsmjitRuntime().release<ConversionFunction>(converters[PixelTypes::YUYV]);
}

void DebugImageConverter::convertToBGRA(const DebugImage& src, void* dest)
{
  if(src.type == PixelTypes::BGRA)
    memcpy(dest, src.getView<PixelTypes::BGRAPixel>()[0], src.width * src.height * PixelTypes::pixelSize(PixelTypes::BGRA));
  else
  {
    ASSERT(simdAligned<_supportsAVX2>(dest));
    if(converters[src.type] == nullptr)
    {
      // Check for a valid type before compiling the function
      switch(src.type)
      {
        case PixelTypes::PixelType::RGB:
        case PixelTypes::PixelType::YUV:
        case PixelTypes::PixelType::YUYV:
        case PixelTypes::PixelType::Colored:
        case PixelTypes::PixelType::Grayscale:
        case PixelTypes::PixelType::Edge2:
        case PixelTypes::PixelType::Binary:
          break;
        default:
          FAIL("Unknown pixel type.");
      }

      // Initialize assembler
      CodeHolder code;
      code.init(Global::getAsmjitRuntime().codeInfo());
      x86::Assembler a(&code);

      // Emit Prolog
      a.push(a.zbp());
      a.mov(a.zbp(), a.zsp());
      a.push(a.zbx());
#if ASMJIT_ARCH_X86 == 64
#ifdef _WIN32
      // Windows64
      a.push(a.zdi());
      a.push(a.zsi());
      a.mov(x86::edi, x86::ecx);
      a.mov(a.zsi(), a.zdx());
      a.mov(a.zdx(), x86::r8);
#endif
#else
      // CDECL
      a.push(a.zdi());
      a.push(a.zsi());
      a.mov(x86::edi, x86::Mem(a.zbp(), 8));
      a.mov(a.zsi(), x86::Mem(a.zbp(), 12));
      a.mov(a.zdx(), x86::Mem(a.zbp(), 16));
#endif
      switch(src.type)
      {
        case PixelTypes::PixelType::RGB:
          rgbToBGRA(a);
          break;
        case PixelTypes::PixelType::YUV:
          asmReturn(a);
          break;
        case PixelTypes::PixelType::YUYV:
          yuyvToBGRA(a);
          break;
        case PixelTypes::PixelType::Colored:
          asmReturn(a);
          break;
        case PixelTypes::PixelType::Grayscale:
          asmReturn(a);
          break;
        case PixelTypes::PixelType::Edge2:
          asmReturn(a);
          break;
        case PixelTypes::PixelType::Binary:
          asmReturn(a);
          break;
      }

      if(Global::getAsmjitRuntime().add<ConversionFunction>(&converters[src.type], &code))
        converters[src.type] = nullptr;
    }
    if(converters[src.type])
      converters[src.type](static_cast<unsigned int>(src.width * src.height * PixelTypes::pixelSize(src.type)), src.getView<unsigned char>()[0], dest);
  }
}
