/**
 * @author Felix Thielke
 */

#include "Activation.h"
#include "../ActivationFunctions.h"
#include "Platform/BHAssert.h"

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    void ActivationCompiler::compile(x86::Assembler& a, ActivationFunctionHandler& afHandler, const TensorPointerXf& input, const TensorPointerXf& output) const
    {
      ASSERT(input.dims() == output.dims());
      const bool isInplace = input.data() == output.data();

      if(isInplace && p.activationDesc == CompiledActivationFunctionId::linear)
        return;

      a.mov(a.zsi(), imm(input.data()));
      if(!isInplace)
        a.mov(a.zdi(), imm(output.data()));

      ASSERT(ActivationFunctionHandler::neededSpares(p.activationDesc) < settings.xmmRegs());
      unsigned int remainingChannels = static_cast<unsigned int>(input.size());
      for(unsigned int stepSize = settings.xmmRegs() - ActivationFunctionHandler::neededSpares(p.activationDesc); stepSize; --stepSize)
      {
        const unsigned int channelsPerStep = stepSize * 4;
        if(remainingChannels < channelsPerStep)
          continue;

        // Initialize activation function
        ActivationFn& activationFunction = afHandler.prepare(p.activationDesc, false, a, {}, {});
        for(unsigned int step = 0; step < stepSize; step++)
          activationFunction.addValue(x86::xmm(step));
        for(unsigned int step = stepSize; step < settings.xmmRegs(); step++)
          activationFunction.addSpare(x86::xmm(step));
        activationFunction.initialize(a);

        Label loop;
        if(remainingChannels >= channelsPerStep * 2)
        {
          loop = a.newLabel();
          a.mov(a.zcx(), imm(remainingChannels / channelsPerStep));
          a.bind(loop);
        }

        for(unsigned int step = 0; step < stepSize; step++)
          a.movaps(x86::xmm(step), a.ptr_zsi(step * 4 * sizeof(float)));
        activationFunction.apply(a);
        for(unsigned int step = 0; step < stepSize; step++)
          a.movaps(isInplace ? a.ptr_zsi(step * 4 * sizeof(float)) : a.ptr_zdi(step * 4 * sizeof(float)), x86::xmm(step));

        if(remainingChannels != channelsPerStep)
        {
          a.add(a.zsi(), imm(stepSize * 4 * sizeof(float)));
          if(!isInplace)
            a.add(a.zdi(), imm(stepSize * 4 * sizeof(float)));
        }

        if(remainingChannels >= channelsPerStep * 2)
        {
          a.dec(a.zcx());
          a.jne(loop);
        }

        remainingChannels %= channelsPerStep;
      }
      if(remainingChannels == 1)
      {
        ActivationFn& activationFunction = afHandler.prepare(p.activationDesc, true, a, { }, { x86::xmm0 });
        for(unsigned int i = 1; i < settings.xmmRegs(); i++)
          activationFunction.addSpare(x86::xmm(i));
        activationFunction.initialize(a);
        a.movss(x86::xmm0, a.ptr_zsi());
        activationFunction.apply(a);
        a.movss(isInplace ? a.ptr_zsi() : a.ptr_zdi(), x86::xmm0);
      }
      else if(remainingChannels)
      {
        ASSERT(remainingChannels <= 4);
        ActivationFn& activationFunction = afHandler.prepare(p.activationDesc, false, a, { }, { x86::xmm0 });
        for(unsigned int i = 1; i < settings.xmmRegs(); i++)
          activationFunction.addSpare(x86::xmm(i));
        activationFunction.initialize(a);
        a.movaps(x86::xmm0, a.ptr_zsi());
        activationFunction.apply(a);
        a.movaps(isInplace ? a.ptr_zsi() : a.ptr_zdi(), x86::xmm0);
      }
    }
  }
}
