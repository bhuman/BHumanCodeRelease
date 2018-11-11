/**
 * @author Felix Thielke
 */

#include "Softmax.h"
#include "Tools/NeuralNetwork/CompiledNN/Util/ExpApprox.h"
#include <cmath>

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    void SoftmaxLayerCompiler::initialize()
    {
      ASSERT(l.inputDimensions[1] == 1);
      ASSERT(l.inputDimensions[2] == 1);

      constants.resize(1);

      std::vector<float>& vals = constants.back().data;
      vals.resize(8);

      // Set exp(x) black magic constants
      const float factor = ExpApprox::factor();
      const int offset = ExpApprox::offset();
      for(unsigned int i = 0; i < 4; i++)
        vals[i] = factor;
      for(unsigned int i = 4; i < 8; i++)
        vals[i] = *reinterpret_cast<const float*>(&offset);
    }

    void SoftmaxLayerCompiler::compile(X86Assembler& a, ActivationFunctionHandler& afHandler, const float* const data) const
    {
      a.mov(a.zsi(), imm_ptr<const float*>(data));

      if(l.inputDimensions[0] >= 4 && (l.inputDimensions[0] % 4 != 0 || l.inputDimensions[0] / 4 > 5))
        a.mov(a.zdi(), a.zsi());

      // Load constants
      a.movaps(x86::xmm(settings.xmmRegs() - 2), x86::ptr(constants.back().label));
      a.movdqa(x86::xmm(settings.xmmRegs() - 1), x86::ptr(constants.back().label, 16));

      // Initialise sum
      a.xorps(x86::xmm0, x86::xmm0);

      // Calculate sum of exponentiated values
      unsigned int remainingChannels = l.inputDimensions[0];
      for(unsigned int stepSize = settings.xmmRegs() - 3; stepSize; --stepSize)
      {
        if(remainingChannels >= stepSize * 4)
        {
          // Begin loop if the stepsize can be applied multiple times
          Label loop;
          if(remainingChannels >= stepSize * 8)
          {
            a.mov(a.zcx(), imm_u(remainingChannels / (stepSize * 4)));
            loop = a.newLabel();
            a.bind(loop);
          }

          // Load values
          for(unsigned int i = 0; i < stepSize; ++i)
            a.movaps(x86::xmm(i + 1), a.ptr_zsi(i * 4 * sizeof(float)));

          // Approximate exp(x)
          std::vector<X86Xmm> valueRegs;
          for(unsigned int i = 0; i < stepSize; ++i)
            valueRegs.emplace_back(x86::xmm(i + 1));
          ExpApprox::apply(a, valueRegs, x86::xmm(settings.xmmRegs() - 2), x86::xmm(settings.xmmRegs() - 1));

          // Write back exponentiated values
          if(stepSize * 4 != l.inputDimensions[0])
          {
            for(unsigned int i = 0; i < stepSize; ++i)
              a.movaps(a.ptr_zsi(i * 4 * sizeof(float)), x86::xmm(i + 1));
          }

          // Accumulate sum
          bool odd = stepSize % 2 != 0;
          for(unsigned int stride = stepSize / 2; stride; stride /= 2)
          {
            for(unsigned int i = 0; i < stride; i++)
              a.addps(x86::xmm(i + 1), x86::xmm(i + 1 + stride));
            if(odd)
              a.addps(x86::xmm0, x86::xmm(stride * 2));
            odd = stride % 2 != 0;
          }
          a.addps(x86::xmm0, x86::xmm1);

          // Increment pointer if there will be further steps
          if(remainingChannels != stepSize * 4)
            a.add(a.zsi(), imm_u(stepSize * 4 * sizeof(float)));

          // End loop if the stepsize can be applied multiple times
          if(remainingChannels >= stepSize * 8)
            a.loop(loop);

          remainingChannels %= stepSize * 4;
        }
      }
      if(remainingChannels > 0)
      {
        for(unsigned int i = 0; i < remainingChannels; ++i)
          a.movss(x86::xmm(i + 1), a.ptr_zsi(i * sizeof(float)));
        for(unsigned int i = 0; i < remainingChannels; ++i)
          a.mulss(x86::xmm(i + 1), x86::xmm(settings.xmmRegs() - 2));
        for(unsigned int i = 0; i < remainingChannels; ++i)
          a.cvtps2dq(x86::xmm(i + 1), x86::xmm(i + 1));
        for(unsigned int i = 0; i < remainingChannels; ++i)
          a.paddd(x86::xmm(i + 1), x86::xmm(settings.xmmRegs() - 1));
        if(remainingChannels != l.inputDimensions[0])
        {
          for(unsigned int i = 0; i < remainingChannels; ++i)
            a.movss(a.ptr_zsi(i * sizeof(float)), x86::xmm(i + 1));
        }
        for(unsigned int i = 0; i < remainingChannels; ++i)
          a.addss(x86::xmm0, x86::xmm(i + 1));
      }
      if(l.inputDimensions[0] >= 4)
      {
        a.haddps(x86::xmm0, x86::xmm0);
        a.haddps(x86::xmm0, x86::xmm0);
      }

      // Compute reciprocal of sum
      if(l.inputDimensions[0] >= 4)
        a.rcpps(x86::xmm0, x86::xmm0);
      else
        a.rcpss(x86::xmm0, x86::xmm0);

      // Calculate softmax by dividing the exponentiated values by the computed sum
      remainingChannels = l.inputDimensions[0];
      for(unsigned int stepSize = settings.xmmRegs() - 1; stepSize; --stepSize)
      {
        if(remainingChannels >= stepSize * 4)
        {
          // Begin loop if the stepsize can be applied multiple times
          Label loop;
          if(remainingChannels >= stepSize * 8)
          {
            a.mov(a.zcx(), imm_u(remainingChannels / (stepSize * 4)));
            loop = a.newLabel();
            a.bind(loop);
          }

          // Multiply values by the reciprocal of the sum
          if(stepSize > settings.xmmRegs() - 3 || stepSize * 4 != l.inputDimensions[0])
          {
            for(unsigned int i = 0; i < stepSize; ++i)
              a.movaps(x86::xmm(i + 1), a.ptr_zdi(i * 4 * sizeof(float)));
          }
          for(unsigned int i = 0; i < stepSize; ++i)
            a.mulps(x86::xmm(i + 1), x86::xmm0);
          for(unsigned int i = 0; i < stepSize; ++i)
            a.movaps(stepSize > settings.xmmRegs() - 3 || stepSize * 4 != l.inputDimensions[0] ? a.ptr_zdi(i * 4 * sizeof(float)) : a.ptr_zsi(i * 4 * sizeof(float)), x86::xmm(i + 1));

          // Increment pointer if there will be further steps
          if(remainingChannels != stepSize * 4)
            a.add(a.zdi(), imm_u(stepSize * 4 * sizeof(float)));

          // End loop if the stepsize can be applied multiple times
          if(remainingChannels >= stepSize * 8)
            a.loop(loop);

          remainingChannels %= stepSize * 4;
        }
      }
      if(remainingChannels > 0)
      {
        if(remainingChannels != l.inputDimensions[0])
        {
          for(unsigned int i = 0; i < remainingChannels; ++i)
            a.movss(x86::xmm(i + 1), a.ptr_zdi(i * sizeof(float)));
        }
        for(unsigned int i = 0; i < remainingChannels; ++i)
          a.mulss(x86::xmm(i + 1), x86::xmm0);
        for(unsigned int i = 0; i < remainingChannels; ++i)
          a.movss(remainingChannels != l.inputDimensions[0] ? a.ptr_zdi(i * sizeof(float)) : a.ptr_zsi(i * sizeof(float)), x86::xmm(i + 1));
      }
    }
  }
}
