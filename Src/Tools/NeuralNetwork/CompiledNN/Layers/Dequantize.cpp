/**
 * @author Felix Thielke
 */

#include "Dequantize.h"

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    void DequantizeLayerCompiler::initialize()
    {
      constants.resize(1);
      std::vector<float>& params = constants.back().data;

      const unsigned int zeroPoint = quantizationParams.zeroPoint;
      for(size_t i = 4; i; --i)
        params.emplace_back(*reinterpret_cast<const float*>(&zeroPoint));
      for(size_t i = 4; i; --i)
        params.emplace_back(quantizationParams.scale);
    }

    void DequantizeLayerCompiler::compile(X86Assembler& a, ActivationFunctionHandler& afHandler, const float* const input, const float* const output) const
    {
      const Label& params = constants.back().label;
      a.mov(a.zsi(), imm_ptr<>(input));
      a.mov(a.zdi(), imm_ptr<>(output));

      // Load constants
      a.pxor(x86::xmm(settings.xmmRegs() - 1), x86::xmm(settings.xmmRegs() - 1));
      a.movdqa(x86::xmm(settings.xmmRegs() - 2), x86::ptr(params));
      a.movaps(x86::xmm(settings.xmmRegs() - 3), x86::ptr(params, 16));

      unsigned int remainingChannels = dimensions[0] * dimensions[1] * dimensions[2];
      for(unsigned int stepSize = (settings.xmmRegs() - 3) / 4; stepSize; stepSize--)
      {
        const unsigned int channelsPerStep = stepSize * 16;
        if(remainingChannels < channelsPerStep)
          continue;

        Label loop;
        if(remainingChannels >= channelsPerStep * 2)
        {
          loop = a.newLabel();
          a.mov(a.zcx(), imm_u(remainingChannels / channelsPerStep));
          a.bind(loop);
        }

        // Load quantized values
        for(unsigned int step = 0; step < stepSize; step++)
          a.movdqa(x86::xmm(step), a.ptr_zsi(step * 16));

        // Convert to 16-bit
        for(unsigned int step = 0; step < stepSize; step++)
          a.movdqa(x86::xmm(step + stepSize * 2), x86::xmm(step));
        for(unsigned int step = 0; step < stepSize; step++)
          a.punpcklbw(x86::xmm(step), x86::xmm(settings.xmmRegs() - 1));
        for(unsigned int step = 0; step < stepSize; step++)
          a.punpckhbw(x86::xmm(step + stepSize * 2), x86::xmm(settings.xmmRegs() - 1));

        // Convert to 32-bit
        for(unsigned int step = 0; step < stepSize; step++)
          a.movdqa(x86::xmm(step + stepSize * 1), x86::xmm(step));
        for(unsigned int step = 0; step < stepSize; step++)
          a.movdqa(x86::xmm(step + stepSize * 3), x86::xmm(step + stepSize * 2));
        for(unsigned int step = 0; step < stepSize; step++)
          a.punpcklwd(x86::xmm(step), x86::xmm(settings.xmmRegs() - 1));
        for(unsigned int step = 0; step < stepSize; step++)
          a.punpckhwd(x86::xmm(step + stepSize * 1), x86::xmm(settings.xmmRegs() - 1));
        for(unsigned int step = 0; step < stepSize; step++)
          a.punpcklwd(x86::xmm(step + stepSize * 2), x86::xmm(settings.xmmRegs() - 1));
        for(unsigned int step = 0; step < stepSize; step++)
          a.punpckhwd(x86::xmm(step + stepSize * 3), x86::xmm(settings.xmmRegs() - 1));

        // Subtract zero point
        for(unsigned int step = 0; step < stepSize * 4; step++)
          a.psubd(x86::xmm(step), x86::xmm(settings.xmmRegs() - 2));

        // Convert to floats
        for(unsigned int step = 0; step < stepSize * 4; step++)
          a.cvtdq2ps(x86::xmm(step), x86::xmm(step));

        // Multiply with scale
        for(unsigned int step = 0; step < stepSize * 4; step++)
          a.mulps(x86::xmm(step), x86::xmm(settings.xmmRegs() - 3));

        // Store result
        for(unsigned int step = 0; step < stepSize * 4; step++)
          a.movaps(a.ptr_zdi(step * 4 * sizeof(float)), x86::xmm(step));

        // Increase pointers
        if(remainingChannels != channelsPerStep)
        {
          a.add(a.zsi(), imm_u(stepSize * 16));
          a.add(a.zdi(), imm_u(stepSize * 16 * sizeof(float)));
        }

        if(remainingChannels >= channelsPerStep * 2)
          a.loop(loop);

        remainingChannels %= channelsPerStep;
      }

      if(remainingChannels)
      {
        // Load quantized values
        a.movdqa(x86::xmm0, a.ptr_zsi());

        // Convert to 16-bit
        if(remainingChannels <= 8)
        {
          a.punpcklbw(x86::xmm0, x86::xmm(settings.xmmRegs() - 1));
        }
        else
        {
          a.movdqa(x86::xmm2, x86::xmm0);
          a.punpcklbw(x86::xmm0, x86::xmm(settings.xmmRegs() - 1));
          a.punpckhbw(x86::xmm2, x86::xmm(settings.xmmRegs() - 1));
        }

        // Convert to 32-bit
        if(remainingChannels > 4)
        {
          a.movdqa(x86::xmm1, x86::xmm0);
          if(remainingChannels > 12)
            a.movdqa(x86::xmm3, x86::xmm2);
        }
        a.punpcklwd(x86::xmm0, x86::xmm(settings.xmmRegs() - 1));
        if(remainingChannels > 4)
        {
          a.punpckhwd(x86::xmm1, x86::xmm(settings.xmmRegs() - 1));
          if(remainingChannels > 8)
          {
            a.punpcklwd(x86::xmm2, x86::xmm(settings.xmmRegs() - 1));
            if(remainingChannels > 12)
              a.punpckhwd(x86::xmm3, x86::xmm(settings.xmmRegs() - 1));
          }
        }

        const unsigned int remainingRegisters = (remainingChannels + 3) / 4;

        // Subtract zero point
        for(unsigned int step = 0; step < remainingRegisters; step++)
          a.psubd(x86::xmm(step), x86::xmm(settings.xmmRegs() - 2));

        // Convert to floats
        for(unsigned int step = 0; step < remainingRegisters; step++)
          a.cvtdq2ps(x86::xmm(step), x86::xmm(step));

        // Multiply with scale
        for(unsigned int step = 0; step < remainingRegisters; step++)
          a.mulps(x86::xmm(step), x86::xmm(settings.xmmRegs() - 3));

        // Store result
        for(unsigned int step = 0; step < remainingRegisters; step++)
          a.movaps(a.ptr_zdi(step * 4 * sizeof(float)), x86::xmm(step));
      }
    }
  }
}
