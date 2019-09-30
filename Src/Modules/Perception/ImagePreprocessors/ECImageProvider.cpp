/**
 * @file ECImageProvider.cpp
 * @author Felix Thielke
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "ECImageProvider.h"
#include "Tools/Global.h"
#include <asmjit/asmjit.h>

MAKE_MODULE(ECImageProvider, perception)

void ECImageProvider::update(ECImage& ecImage)
{
  ecImage.grayscaled.setResolution(theCameraInfo.width, theCameraInfo.height);
  ecImage.colored.setResolution(theCameraInfo.width, theCameraInfo.height);
  ecImage.saturated.setResolution(theCameraInfo.width, theCameraInfo.height);
  ecImage.hued.setResolution(theCameraInfo.width, theCameraInfo.height);

  if(theCameraImage.timestamp > 10 && static_cast<int>(theCameraImage.width) == theCameraInfo.width / 2)
  {
    if(disableClassification)
    {
      if(!eFunc)
        compileE();
      eFunc(theCameraInfo.width * theCameraInfo.height / 16, theCameraImage[0], ecImage.grayscaled[0]);
    }
    else
    {
      if(!ecFunc)
        compileEC();

      // There may be faster ways to copy the colors.
      if(theFieldColors.maxNonColorSaturation != currentMaxNonColorSaturation[0])
        for(size_t i = 0; i < 16; i++)
          currentMaxNonColorSaturation[i] = theFieldColors.maxNonColorSaturation;
      if(theFieldColors.blackWhiteDelimiter != currentBlackWhiteDelimiter[0])
        for(size_t i = 0; i < 16; i++)
          currentBlackWhiteDelimiter[i] = theFieldColors.blackWhiteDelimiter;
      if(theFieldColors.fieldHue.min != currentFieldHueMin[0])
        for(size_t i = 0; i < 16; i++)
          currentFieldHueMin[i] = theFieldColors.fieldHue.min;
      if(theFieldColors.fieldHue.max != currentFieldHueMax[0])
        for(size_t i = 0; i < 16; i++)
          currentFieldHueMax[i] = theFieldColors.fieldHue.max;

      ecFunc(theCameraInfo.width * theCameraInfo.height / 16, theCameraImage[0], ecImage.grayscaled[0], ecImage.saturated[0], ecImage.hued[0], ecImage.colored[0]);
    }
    ecImage.timestamp = theCameraImage.timestamp;
  }
}

using namespace asmjit;

void ECImageProvider::compileE()
{
  ASSERT(!eFunc);

  // Initialize assembler
  CodeHolder code;
  code.init(Global::getAsmjitRuntime().codeInfo());
  x86::Assembler a(&code);

  // Emit prolog
  a.enter(imm(0u), imm(0u));
#if ASMJIT_ARCH_X86 == 64
#ifdef _WIN32
  // Windows64
  x86::Gp src = a.zdx();
  x86::Gp dest = x86::r8;
#else
  // System V x64
  a.mov(a.zcx(), a.zdi());
  x86::Gp src = a.zsi();
  x86::Gp dest = a.zdx();
#endif
#else
  // CDECL
  x86::Gp src = a.zdx();
  x86::Gp dest = a.zax();
  a.mov(a.zcx(), x86::Mem(a.zbp(), 8));
  a.mov(src, x86::Mem(a.zbp(), 12));
  a.mov(dest, x86::Mem(a.zbp(), 16));
#endif

  Label loMask16 = a.newLabel();
  a.movdqa(x86::xmm2, x86::ptr(loMask16));

  Label loop = a.newLabel();
  a.bind(loop);

  a.movdqu(x86::xmm0, x86::ptr(src, 0));
  a.movdqu(x86::xmm1, x86::ptr(src, 16));

  a.pand(x86::xmm0, x86::xmm2);
  a.pand(x86::xmm1, x86::xmm2);
  a.packuswb(x86::xmm0, x86::xmm1);

  a.add(src, imm(16u * 2u));

  a.movdqa(x86::ptr(dest), x86::xmm0);
  a.add(dest, imm(16u));

  a.dec(a.zcx());
  a.jnz(loop);

  // Emit epilog
  a.leave();
  a.ret();

  // Store constant
  a.align(AlignMode::kAlignZero, 16);
  a.bind(loMask16);
  for(size_t i = 0; i < 8; i++) a.dint16(0x00FF);

  // Bind function
  const Error err = Global::getAsmjitRuntime().add<EFunc>(&eFunc, &code);
  if(err)
  {
    OUTPUT_ERROR(err);
    eFunc = nullptr;
  }
}

void ECImageProvider::compileEC()
{
  ASSERT(!ecFunc);

  // Initialize assembler
  CodeHolder code;
  code.init(Global::getAsmjitRuntime().codeInfo());
  x86::Assembler a(&code);

  // Define argument registers
  x86::Gp remainingSteps = x86::edi;
  x86::Gp src = a.zsi();
  x86::Gp grayscaled = a.zdx();
  x86::Gp saturated = a.zcx();
  x86::Gp hued = a.zax();
  x86::Gp colored = a.zbx();

  // Emit Prolog
  a.push(a.zbp());
  a.mov(a.zbp(), a.zsp());
  a.push(a.zbx());
#if ASMJIT_ARCH_X86 == 64
#ifdef _WIN32
  // Windows64
  a.push(a.zdi());
  a.push(a.zsi());
  a.mov(remainingSteps, x86::ecx);
  a.mov(src, a.zdx());
  a.mov(grayscaled, x86::r8);
  a.mov(saturated, x86::r9);
  a.mov(hued, x86::Mem(a.zbp(), 16 + 32));
  a.mov(colored, x86::Mem(a.zbp(), 16 + 32 + 8));
#else
  // System V x64
  a.mov(hued, x86::r8);
  a.mov(colored, x86::r9);
#endif
#else
  // CDECL
  a.push(a.zdi());
  a.push(a.zsi());
  a.mov(remainingSteps, x86::Mem(a.zbp(), 8));
  a.mov(src, x86::Mem(a.zbp(), 12));
  a.mov(grayscaled, x86::Mem(a.zbp(), 16));
  a.mov(saturated, x86::Mem(a.zbp(), 20));
  a.mov(hued, x86::Mem(a.zbp(), 24));
  a.mov(colored, x86::Mem(a.zbp(), 28));
#endif

  // Define constants
  Label constants = a.newLabel();
  x86::Mem loMask16(constants, 0);
  x86::Mem c8_128(constants, 16);
  x86::Mem loMask32(constants, 16 * 2);
  x86::Mem tallyInit(constants, 16 * 3);
  x86::Mem c16_64(constants, 16 * 4);
  x86::Mem c16_128(constants, 16 * 5);
  x86::Mem c16_x8001(constants, 16 * 6);
  x86::Mem c16_5695(constants, 16 * 7);
  x86::Mem c16_11039(constants, 16 * 8);
  x86::Mem maxNonColorSaturation8(constants, 16 * 9);
  x86::Mem fieldHFrom8(constants, 16 * 10);
  x86::Mem fieldHTo8(constants, 16 * 11);
  x86::Mem blackWhiteDelimiter8(constants, 16 * 12);
  x86::Mem classWhite8(constants, 16 * 13);
  x86::Mem classField8(constants, 16 * 14);
  x86::Mem classBlack8(constants, 16 * 15);

  // Start of loop
  Label loop = a.newLabel();
  a.bind(loop);
  // XMM0-XMM1: Source / x64: XMM8-XMM9: Source
  a.movdqu(x86::xmm0, x86::Mem(src, 0));
  a.movdqu(x86::xmm1, x86::Mem(src, 16));
  a.add(src, 32);

  // Compute luminance
  a.movdqa(x86::xmm4, loMask16); // XMM4 is now loMask16
  a.movdqa(x86::xmm2, x86::xmm0);
  a.movdqa(x86::xmm3, x86::xmm1);
  a.pand(x86::xmm2, x86::xmm4); // XMM2 is now 16-bit luminance0
  a.pand(x86::xmm3, x86::xmm4); // XMM3 is now 16-bit luminance1
#if !ASMJIT_ARCH_64BIT
  a.movdqa(x86::xmm5, x86::xmm2);
  a.packuswb(x86::xmm5, x86::xmm3);
  // store grayscaled
  a.movdqa(x86::ptr(grayscaled), x86::xmm5);
#else
  a.movdqa(x86::xmm8, x86::xmm2);
  a.packuswb(x86::xmm8, x86::xmm3); // XMM8 is now 8-bit grayscaled
  // store grayscaled
  a.movntdq(x86::ptr(grayscaled), x86::xmm8);
#endif
  a.add(grayscaled, 16);

  // Convert image data to 8-bit UV in XMM0 / x64: XMM8
  a.psrldq(x86::xmm0, 1);
  a.psrldq(x86::xmm1, 1);
  a.pand(x86::xmm0, x86::xmm4);
  a.pand(x86::xmm1, x86::xmm4);
  a.packuswb(x86::xmm0, x86::xmm1);
  a.psubb(x86::xmm0, c8_128);

  // Compute saturation
  a.pabsb(x86::xmm1, x86::xmm0);
  a.pmaddubsw(x86::xmm1, x86::xmm1);
  a.pxor(x86::xmm4, x86::xmm4);
  a.punpcklwd(x86::xmm4, x86::xmm1);
  a.pslld(x86::xmm4, 1);
  a.cvtdq2ps(x86::xmm4, x86::xmm4);
  a.rsqrtps(x86::xmm4, x86::xmm4); // XMM4 is now rnormUV0
  a.movdqa(x86::xmm5, x86::xmm2);
  a.movdqa(x86::xmm6, x86::xmm3);
  a.psrld(x86::xmm5, 16); // XMM5 is now y1
  a.psrld(x86::xmm6, 16); // XMM6 is now y3
  a.movdqa(x86::xmm7, loMask32); // XMM7 is now loMask32
  a.pand(x86::xmm2, x86::xmm7); // XMM2 is now y0
  a.pand(x86::xmm3, x86::xmm7); // XMM3 is now y2
  a.cvtdq2ps(x86::xmm2, x86::xmm2);
  a.cvtdq2ps(x86::xmm5, x86::xmm5);
  a.cvtdq2ps(x86::xmm3, x86::xmm3);
  a.cvtdq2ps(x86::xmm6, x86::xmm6);
  a.mulps(x86::xmm2, x86::xmm4);
  a.mulps(x86::xmm5, x86::xmm4);
  a.rcpps(x86::xmm2, x86::xmm2);
  a.rcpps(x86::xmm5, x86::xmm5);
  a.cvtps2dq(x86::xmm2, x86::xmm2);
  a.cvtps2dq(x86::xmm5, x86::xmm5);
  a.pslld(x86::xmm5, 16);
  a.por(x86::xmm2, x86::xmm5); // XMM2 is now 16-bit sat0
  a.pxor(x86::xmm4, x86::xmm4);
  a.punpckhwd(x86::xmm4, x86::xmm1);
  a.pslld(x86::xmm4, 1);
  a.cvtdq2ps(x86::xmm4, x86::xmm4);
  a.rsqrtps(x86::xmm4, x86::xmm4); // XMM4 is now rnormUV1
  a.mulps(x86::xmm3, x86::xmm4);
  a.mulps(x86::xmm6, x86::xmm4);
  a.rcpps(x86::xmm3, x86::xmm3);
  a.rcpps(x86::xmm6, x86::xmm6);
  a.cvtps2dq(x86::xmm3, x86::xmm3);
  a.cvtps2dq(x86::xmm6, x86::xmm6);
  a.pslld(x86::xmm6, 16);
  a.por(x86::xmm3, x86::xmm6); // XMM3 is now 16-bit sat1
  a.packuswb(x86::xmm2, x86::xmm3); // XMM2 is now 8-bit saturation
  // store saturated
  a.movntdq(x86::ptr(saturated), x86::xmm2);
  a.add(saturated, 16);

  // Compute hue
  a.movdqa(x86::xmm1, x86::xmm0);
  a.psraw(x86::xmm1, 8); // XMM1 is now 16-bit V
  a.psllw(x86::xmm0, 8);
  a.psraw(x86::xmm0, 8); // XMM0 is now 16-bit U
  a.pabsw(x86::xmm3, x86::xmm0); // XMM3 is now 16-bit abs(U)
  a.pabsw(x86::xmm4, x86::xmm1); // XMM4 is now 16-bit abs(V)
  a.movdqa(x86::xmm5, x86::xmm3);
  a.pminsw(x86::xmm5, x86::xmm4); // XMM5 is now 16-bit min(abs(U),abs(V))
  a.pmaxsw(x86::xmm3, x86::xmm4); // XMM3 is now 16-bit max(abs(U),abs(V))
  a.pcmpeqw(x86::xmm4, x86::xmm5); // XMM4 is now (U > V)
  a.movdqa(x86::xmm6, x86::xmm0);
  a.psignw(x86::xmm6, x86::xmm1); // XMM6 is now sign(U,V)
  a.movdqa(x86::xmm7, c16_128); // XMM7 is now c16_128
  a.pand(x86::xmm0, x86::xmm7);
  a.pand(x86::xmm1, x86::xmm7);
  a.pand(x86::xmm0, x86::xmm4);
  a.por(x86::xmm1, c16_64);
  a.movdqa(x86::xmm7, x86::xmm4);
  a.pandn(x86::xmm7, x86::xmm1);
  a.por(x86::xmm0, x86::xmm7); // XMM0 is now the 16-bit atan2-offset
  a.pxor(x86::xmm4, c16_x8001);
  a.psignw(x86::xmm4, x86::xmm6); // XMM4 is now the 16-bit atan2-sign
  // Scale and divide min by max
  a.movdqa(x86::xmm6, tallyInit); // XMM6 is tally
  a.pxor(x86::xmm1, x86::xmm1); // XMM1 is quotient
  a.psllw(x86::xmm3, 5);
  a.psllw(x86::xmm5, 6);
  for(size_t i = 0; i < 5; i++)
  {
    a.movdqa(x86::xmm7, x86::xmm5);
    a.pcmpgtw(x86::xmm7, x86::xmm3); // XMM7 is now (min > max)
    a.pand(x86::xmm7, x86::xmm6);
    a.paddsw(x86::xmm1, x86::xmm7);
    a.movdqa(x86::xmm7, x86::xmm5);
    a.pcmpgtw(x86::xmm7, x86::xmm3); // XMM7 is now (min > max)
    a.pand(x86::xmm7, x86::xmm3);
    a.psubw(x86::xmm5, x86::xmm7);
    a.psrlw(x86::xmm6, 1);
    a.psrlw(x86::xmm3, 1);
  }
  // XMM1 is now (min << 15) / max
  a.movdqa(x86::xmm3, x86::xmm1);
  a.pmulhrsw(x86::xmm3, c16_5695);
  a.movdqa(x86::xmm5, c16_11039);
  a.psubw(x86::xmm5, x86::xmm3);
  a.pmulhrsw(x86::xmm1, x86::xmm5); // XMM1 is now the 16-bit absolute unrotated atan2
  a.psignw(x86::xmm1, x86::xmm4); // XMM1 is now the 16-bit unrotated atan2
  a.paddw(x86::xmm0, x86::xmm1); // XMM0 is now 16-bit hue
  a.psllw(x86::xmm0, 8);
  a.movdqa(x86::xmm1, x86::xmm0);
  a.psrlw(x86::xmm1, 8);
  a.por(x86::xmm0, x86::xmm1); // XMM0 is now 8-bit hue
  // store hued
  a.movntdq(x86::ptr(hued), x86::xmm0);
  a.add(hued, 16);

  // Classify hue
  a.movdqa(x86::xmm1, fieldHFrom8);
  a.psubusb(x86::xmm1, x86::xmm0);
  a.psubusb(x86::xmm0, fieldHTo8);
  a.pcmpeqb(x86::xmm0, x86::xmm1); // XMM0 is now 8-bit isHueField

  // Classify saturation
  a.movdqa(x86::xmm1, maxNonColorSaturation8);
  a.psubusb(x86::xmm1, x86::xmm2);
  a.pxor(x86::xmm2, x86::xmm2); // XMM2 is now zeroed
  a.pcmpeqb(x86::xmm1, x86::xmm2); // XMM1 is now 8-bit isColored

  // Classify luminance
  a.movdqa(x86::xmm3, blackWhiteDelimiter8);
#if !ASMJIT_ARCH_64BIT
  a.psubusb(x86::xmm3, x86::Mem(grayscaled, -16));
#else
  a.psubusb(x86::xmm3, x86::xmm8);
#endif
  a.pcmpeqb(x86::xmm2, x86::xmm3); // XMM2 is now 8-bit isWhite

  // Map color classes
  a.pand(x86::xmm0, classField8);
  a.pand(x86::xmm0, x86::xmm1); // XMM0 now contains the mapped 'field' entries
  a.movdqa(x86::xmm3, classWhite8);
  a.pand(x86::xmm3, x86::xmm2);
  a.pandn(x86::xmm2, classBlack8);
  a.por(x86::xmm2, x86::xmm3);
  a.pandn(x86::xmm1, x86::xmm2); // XMM1 now contains the mapped 'white' and 'black' entries
  a.por(x86::xmm0, x86::xmm1); // XMM0 is now 8-bit colored
  // store colored
  a.movntdq(x86::ptr(colored), x86::xmm0);
  a.add(colored, 16);

  // End of loop
  a.dec(remainingSteps);
  a.jnz(loop);

  // Return
#if ASMJIT_ARCH_X86 != 64 || defined(_WIN32)
  a.pop(a.zsi());
  a.pop(a.zdi());
#endif
  a.pop(a.zbx());
  a.mov(a.zsp(), a.zbp());
  a.pop(a.zbp());
  a.ret();

  // Constants
  a.align(AlignMode::kAlignZero, 16);
  a.bind(constants);
  for(size_t i = 0; i < 8; i++) a.dint16(0x00FF);        // 0: loMask16
  for(size_t i = 0; i < 16; i++) a.dint8(char(128));     // 1: c8_128
  for(size_t i = 0; i < 4; i++) a.dint32(0x0000FFFF);    // 2: loMask32
  for(size_t i = 0; i < 8; i++) a.dint16(1 << 5);        // 3: init for tally
  for(size_t i = 0; i < 8; i++) a.dint16(64);            // 4: c16_64
  for(size_t i = 0; i < 8; i++) a.dint16(128);           // 5: c16_128
  for(size_t i = 0; i < 8; i++) a.dint16(short(0x8001)); // 6: c16_x8001
  for(size_t i = 0; i < 8; i++) a.dint16(5695);          // 7: c16_5695
  for(size_t i = 0; i < 8; i++) a.dint16(11039);         // 8: c16_11039
  for(size_t i = 0; i < 16; i++) a.dint8(0);             // 9: maxNonColorSaturation8
  for(size_t i = 0; i < 16; i++) a.dint8(0);             // 10: fieldHFrom8
  for(size_t i = 0; i < 16; i++) a.dint8(0);             // 11: fieldHTo8
  for(size_t i = 0; i < 16; i++) a.dint8(0);             // 12: blackWhiteDelimiter8
  for(size_t i = 0; i < 16; i++) a.dint8(FieldColors::Color::white); // 13: classWhite8
  for(size_t i = 0; i < 16; i++) a.dint8(FieldColors::Color::field); // 14: classField8
  for(size_t i = 0; i < 16; i++) a.dint8(FieldColors::Color::black); // 15: classBlack8

  // Bind function
  const Error err = Global::getAsmjitRuntime().add<EcFunc>(&ecFunc, &code);
  if(err)
  {
    OUTPUT_ERROR(err);
    ecFunc = nullptr;
    return;
  }

  auto constantsPtr = reinterpret_cast<uint8_t*>(ecFunc) + code.labelOffset(constants);
  currentMaxNonColorSaturation = constantsPtr + 16 * 9;
  currentFieldHueMin = constantsPtr + 16 * 10;
  currentFieldHueMax = constantsPtr + 16 * 11;
  currentBlackWhiteDelimiter = constantsPtr + 16 * 12;
}

ECImageProvider::~ECImageProvider()
{
  if(eFunc)
    Global::getAsmjitRuntime().release(eFunc);
  if(ecFunc)
    Global::getAsmjitRuntime().release(ecFunc);
}
