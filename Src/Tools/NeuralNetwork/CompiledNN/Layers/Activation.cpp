/**
 * @author Felix Thielke
 */

#include "../ActivationFunctions.h"
#include "Activation.h"

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    void ActivationLayerCompiler::compile(X86Assembler& a, ActivationFunctionHandler& afHandler, const float* const data) const
    {
      if(l.activationId == ActivationFunctionId::linear)
        return;

      a.mov(a.zsi(), imm_ptr<>(data));

      unsigned int remainingChannels = l.inputDimensions[0] * l.inputDimensions[1] * l.inputDimensions[2];
      for(unsigned int stepSize = settings.xmmRegs() - 1; stepSize; --stepSize)
      {
        const unsigned int channelsPerStep = stepSize * 4;
        if(remainingChannels < channelsPerStep)
          continue;

        // Initialize activation function
        ActivationFn& activationFunction = afHandler.prepare(l.activationId, false, a, {}, {});
        for(unsigned int step = 0; step < stepSize; step++)
          activationFunction.addValue(x86::xmm(step));
        for(unsigned int step = stepSize; step < settings.xmmRegs(); step++)
          activationFunction.addSpare(x86::xmm(step));
        activationFunction.initialize(a);

        Label loop;
        if(remainingChannels >= channelsPerStep * 2)
        {
          loop = a.newLabel();
          a.mov(a.zcx(), imm_u(remainingChannels / channelsPerStep));
          a.bind(loop);
        }

        for(unsigned int step = 0; step < stepSize; step++)
          a.movaps(x86::xmm(step), a.ptr_zsi(step * 4 * sizeof(float)));
        activationFunction.apply(a);
        for(unsigned int step = 0; step < stepSize; step++)
          a.movaps(a.ptr_zsi(step * 4 * sizeof(float)), x86::xmm(step));

        if(remainingChannels != channelsPerStep)
          a.add(a.zsi(), imm_u(stepSize * 4 * sizeof(float)));

        if(remainingChannels >= channelsPerStep * 2)
        {
          if(stepSize <= 7)
            a.loop(loop);
          else
          {
            a.dec(a.zcx());
            a.jne(loop);
          }
        }

        remainingChannels %= channelsPerStep;
      }
      if(remainingChannels == 1)
      {
        ActivationFn& activationFunction = afHandler.prepare(l.activationId, true, a, { }, { x86::xmm0 });
        for(unsigned int i = 1; i < settings.xmmRegs(); i++)
          activationFunction.addSpare(x86::xmm(i));
        activationFunction.initialize(a);
        a.movss(x86::xmm0, a.ptr_zsi());
        activationFunction.apply(a);
        a.movss(a.ptr_zsi(), x86::xmm0);
      }
      else if(remainingChannels)
      {
        ActivationFn& activationFunction = afHandler.prepare(l.activationId, false, a, { }, { x86::xmm0 });
        for(unsigned int i = 1; i < settings.xmmRegs(); i++)
          activationFunction.addSpare(x86::xmm(i));
        activationFunction.initialize(a);
        a.movaps(x86::xmm0, a.ptr_zsi());
        activationFunction.apply(a);
        a.movaps(a.ptr_zsi(), x86::xmm0);
      }
    }
  }
}
