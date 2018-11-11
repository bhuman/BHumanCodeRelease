/**
 * @author Felix Thielke
 */

#include "Quantize.h"

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    void QuantizeLayerCompiler::initialize()
    {
      constants.resize(1);
      std::vector<float>& params = constants.back().data;

      const float invScale = 1.f / quantizationParams.scale;
      const float zeroPoint = static_cast<float>(quantizationParams.zeroPoint);
      for(size_t i = 4; i; --i)
        params.emplace_back(invScale);
      for(size_t i = 4; i; --i)
        params.emplace_back(zeroPoint);
      if(settings.quantizationResolution < 8)
      {
        const unsigned int maxValue = ((1 << settings.quantizationResolution) - 1) | (((1 << settings.quantizationResolution) - 1) << 8) | (((1 << settings.quantizationResolution) - 1) << 16) | (((1 << settings.quantizationResolution) - 1) << 24);
        for(size_t i = 4; i; --i)
          params.emplace_back(*reinterpret_cast<const float*>(&maxValue));
      }
    }

    void QuantizeLayerCompiler::compile(X86Assembler& a, ActivationFunctionHandler& afHandler, const float* const data) const
    {
      const Label& params = constants.back().label;
      a.mov(a.zsi(), imm_ptr<>(data));
      a.mov(a.zdi(), a.zsi());

      // Load constants
      a.movaps(x86::xmm(settings.xmmRegs() - 1), x86::ptr(params));
      a.movaps(x86::xmm(settings.xmmRegs() - 2), x86::ptr(params, 4 * sizeof(float)));
      if(settings.quantizationResolution < 8)
        a.movaps(x86::xmm(settings.xmmRegs() - 1), x86::ptr(params, 8 * sizeof(float)));

      unsigned int remainingChannels = dimensions[0] * dimensions[1] * dimensions[2];
      for(unsigned int stepSize = settings.xmmRegs() - 4; stepSize; stepSize -= 4) // stepSize is always a multiple of 4; the last 4 registers are left for the constants
      {
        const unsigned int channelsPerStep = stepSize * 4;
        if(remainingChannels < channelsPerStep)
          continue;

        Label loop;
        if(remainingChannels >= channelsPerStep * 2)
        {
          loop = a.newLabel();
          a.mov(a.zcx(), imm_u(remainingChannels / channelsPerStep));
          a.bind(loop);
        }

        // Load 4 floats per register
        for(unsigned int step = 0; step < stepSize; step++)
          a.movaps(x86::xmm(step), a.ptr_zsi(step * 4 * sizeof(float)));

        // Divide by scale (= multiply by the inverse scale)
        for(unsigned int step = 0; step < stepSize; step++)
          a.mulps(x86::xmm(step), x86::xmm(settings.xmmRegs() - 1));

        // Add zero-point
        for(unsigned int step = 0; step < stepSize; step++)
          a.addps(x86::xmm(step), x86::xmm(settings.xmmRegs() - 2));

        // Convert to 32-bit signed integer
        for(unsigned int step = 0; step < stepSize; step++)
          a.cvtps2dq(x86::xmm(step), x86::xmm(step));   // cvtps2dq rounds to the nearest integer, which is exactly what we want

        // Convert to 16-bit signed integer with saturation
        for(unsigned int step = 0; step < stepSize; step += 2)
          a.packssdw(x86::xmm(step), x86::xmm(step + 1));

        // Convert to 8-bit unsigned integer with saturation, thus clamping to 0..255
        for(unsigned int step = 0; step < stepSize; step += 4)
          a.packuswb(x86::xmm(step), x86::xmm(step + 2));

        if(settings.quantizationResolution < 8)
        {
          // Clamp to max value
          for(unsigned int step = 0; step < stepSize; step += 4)
            a.pminub(x86::xmm(step), x86::xmm(settings.xmmRegs() - 3));
        }

        // Store values
        for(unsigned int step = 0; step < stepSize; step += 4)
          a.movdqa(a.ptr_zdi(step * 4 * sizeof(unsigned char)), x86::xmm(step));

        // Increase pointers
        if(remainingChannels != channelsPerStep)
        {
          a.add(a.zsi(), imm_u(stepSize * 4 * sizeof(float)));
          a.add(a.zdi(), imm_u(stepSize * 4 * sizeof(unsigned char)));
        }

        if(remainingChannels >= channelsPerStep * 2)
          a.loop(loop);
        CHECK_ASMJIT_ERROR;

        remainingChannels %= channelsPerStep;
      }
      if(remainingChannels)
      {
        const unsigned int stepSize = (remainingChannels + 3) / 4;

        // Load 4 floats per register
        for(unsigned int step = 0; step < stepSize; step++)
          a.movaps(x86::xmm(step), a.ptr_zsi(step * 4 * sizeof(float)));

        // Divide by scale (= multiply by the inverse scale)
        for(unsigned int step = 0; step < stepSize; step++)
          a.mulps(x86::xmm(step), x86::xmm(settings.xmmRegs() - 1));

        // Add zero-point
        for(unsigned int step = 0; step < stepSize; step++)
          a.addps(x86::xmm(step), x86::xmm(settings.xmmRegs() - 2));

        // Convert to 32-bit signed integer
        for(unsigned int step = 0; step < stepSize; step++)
          a.cvtps2dq(x86::xmm(step), x86::xmm(step));   // cvtps2dq rounds to the nearest integer, which is exactly what we want

        // Convert to 16-bit signed integer with saturation
        for(unsigned int step = 0; step < stepSize; step += 2)
          a.packssdw(x86::xmm(step), x86::xmm(step + 1));

        // Convert to 8-bit unsigned integer with saturation, thus clamping to 0..255
        for(unsigned int step = 0; step < stepSize; step += 4)
          a.packuswb(x86::xmm(step), x86::xmm(step + 2));

        if(settings.quantizationResolution < 8)
        {
          // Clamp to max value
          for(unsigned int step = 0; step < stepSize; step += 4)
            a.pminub(x86::xmm(step), x86::xmm(settings.xmmRegs() - 3));
        }

        // Store values
        for(unsigned int step = 0; step < stepSize; step += 4)
          a.movdqa(a.ptr_zdi(step * 4 * sizeof(unsigned char)), x86::xmm(step));
      }
    }
  }
}
