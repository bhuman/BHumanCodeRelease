/**
 * @author Felix Thielke
 */

#include "Cropping2D.h"
#include "Platform/BHAssert.h"

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    void Cropping2DCompiler::compile(x86::Assembler& a, ActivationFunctionHandler& afHandler, const TensorPointerXf& input, const TensorPointerXf& output) const
    {
      ASSERT(input.rank() == 3);
      ASSERT(output.rank() == 3);
      ASSERT(input.dims(0) - (p.cropping[Cropping2DLayer::TOP] + p.cropping[Cropping2DLayer::BOTTOM]) == output.dims(0));
      ASSERT(input.dims(1) - (p.cropping[Cropping2DLayer::LEFT] + p.cropping[Cropping2DLayer::RIGHT]) == output.dims(1));
      ASSERT(input.dims(2) == output.dims(2));

      const bool inputAligned = p.cropping[Cropping2DLayer::LEFT] * input.dims(2) % 4 == 0 && input.dims(1) * input.dims(2) % 4 == 0;
      const bool outputAligned = output.dims(1) * output.dims(2) % 4 == 0;

      // Crop image
      a.mov(a.zsi(), imm(input.data() + (p.cropping[Cropping2DLayer::TOP] * output.dims(1) + p.cropping[Cropping2DLayer::LEFT]) * output.dims(2)));
      a.mov(a.zdi(), imm(output.data()));

      a.mov(a.zax(), imm(output.dims(0)));
      Label copyLoop = a.newLabel();
      a.bind(copyLoop);

      unsigned int stepsRemaining = (output.dims(1) * output.dims(2) + 3) / 4;
      for(unsigned int stepSize = settings.xmmRegs(); stepSize; --stepSize)
      {
        if(stepsRemaining < stepSize)
          continue;

        Label copyRowLoop;
        if(stepsRemaining >= 2 * stepSize)
        {
          copyRowLoop = a.newLabel();
          a.mov(a.zcx(), imm(stepsRemaining / stepSize));
          a.bind(copyRowLoop);
        }

        if(inputAligned)
        {
          for(unsigned int i = 0; i < stepSize; i++)
            a.movaps(x86::xmm(i), a.ptr_zsi(i * 4 * sizeof(float)));
        }
        else
        {
          for(unsigned int i = 0; i < stepSize; i++)
            a.movups(x86::xmm(i), a.ptr_zsi(i * 4 * sizeof(float)));
        }
        if(outputAligned)
        {
          for(unsigned int i = 0; i < stepSize; i++)
            a.movaps(a.ptr_zdi(i * 4 * sizeof(float)), x86::xmm(i));
        }
        else
        {
          for(unsigned int i = 0; i < stepSize; i++)
            a.movups(a.ptr_zdi(i * 4 * sizeof(float)), x86::xmm(i));
        }

        a.add(a.zsi(), imm(stepSize * 4 * sizeof(float)));
        a.add(a.zdi(), imm(stepSize * 4 * sizeof(float)));

        if(stepsRemaining >= 2 * stepSize)
        {
          a.dec(a.zcx());
          a.jnz(copyRowLoop);
        }

        stepsRemaining %= stepSize;
      }
      const size_t overshoot = (output.dims(1) * output.dims(2) % 4 == 0) ? 0 : 4 - (output.dims(1) * output.dims(2) % 4);
      a.add(a.zsi(), imm(((p.cropping[Cropping2DLayer::LEFT] + p.cropping[Cropping2DLayer::RIGHT]) * input.dims(2) - overshoot) * sizeof(float)));
      if(overshoot > 0)
        a.sub(a.zdi(), imm(overshoot));

      a.dec(a.zax());
      a.jnz(copyLoop);
    }
  }
}
