/**
 * @author Felix Thielke
 * @author Bernd Poppinga
 */

#include "ZeroPadding2D.h"
#include "Platform/BHAssert.h"

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    void ZeroPadding2DCompiler::compile(x86::Assembler& a, ActivationFunctionHandler& afHandler, const TensorPointerXf& input, const TensorPointerXf& output) const
    {
      ASSERT(input.rank() == 3);
      ASSERT(output.rank() == 3);
      ASSERT(input.dims(0) + p.padding[ZeroPadding2DLayer::TOP] + p.padding[ZeroPadding2DLayer::BOTTOM] == output.dims(0));
      ASSERT(input.dims(1) + p.padding[ZeroPadding2DLayer::LEFT] + p.padding[ZeroPadding2DLayer::RIGHT] == output.dims(1));
      ASSERT(input.dims(2) == output.dims(2));

      // Pad image
      a.mov(a.zsi(), imm(input.data()));
      a.mov(a.zdi(), imm(output.data() + ((output.dims(1) * p.padding[ZeroPadding2DLayer::TOP] + p.padding[ZeroPadding2DLayer::LEFT]) * output.dims(2))));
      if(input.dims(1) * input.dims(2) % 4 == 0)
      {
        a.mov(a.zax(), imm(input.dims(0)));
        Label copyLoop = a.newLabel();
        a.bind(copyLoop);

        unsigned int stepsRemaining = input.dims(1) * input.dims(2) / 4;
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

          for(unsigned int i = 0; i < stepSize; i++)
            a.movaps(x86::xmm(i), a.ptr_zsi(i * 4 * sizeof(float)));
          for(unsigned int i = 0; i < stepSize; i++)
            a.movups(a.ptr_zdi(i * 4 * sizeof(float)), x86::xmm(i));

          a.add(a.zsi(), imm(stepSize * 4 * sizeof(float)));
          a.add(a.zdi(), imm(stepSize * 4 * sizeof(float)));

          if(stepsRemaining >= 2 * stepSize)
          {
            a.dec(a.zcx());
            a.jnz(copyRowLoop);
          }

          stepsRemaining %= stepSize;
        }
        a.add(a.zdi(), imm((p.padding[ZeroPadding2DLayer::LEFT] + p.padding[ZeroPadding2DLayer::RIGHT]) * output.dims(2) * sizeof(float)));

        a.dec(a.zax());
        a.jnz(copyLoop);
      }
      else
      {
        // TODO
        ASSERT(false);
      }

      // Prepare setting borders to zero
      a.mov(a.zsi(), imm(output.data()));
      unsigned int clearRegisters = 0;

      // Clear top border
      unsigned int remainingElements = p.padding[ZeroPadding2DLayer::TOP] * output.dims(1) * input.dims(2);
      for(unsigned int stepSize = settings.xmmRegs(); stepSize; --stepSize)
      {
        const unsigned int elementsPerStep = stepSize * 4;
        if(remainingElements < elementsPerStep)
          continue;

        if(clearRegisters < stepSize)
        {
          for(unsigned int i = clearRegisters; i < stepSize; i++)
            a.xorps(x86::xmm(i), x86::xmm(i));
          clearRegisters = stepSize;
        }

        Label clearTopLoop;
        if(remainingElements >= elementsPerStep * 2)
        {
          clearTopLoop = a.newLabel();
          a.mov(a.zcx(), imm(remainingElements / elementsPerStep));
          a.bind(clearTopLoop);
        }

        for(unsigned int i = 0; i < stepSize; i++)
          a.movaps(a.ptr_zsi(i * 4 * sizeof(float)), x86::xmm(i));

        a.add(a.zsi(), imm(stepSize * 4 * sizeof(float)));

        if(remainingElements >= elementsPerStep * 2)
        {
          a.dec(a.zcx());
          a.jnz(clearTopLoop);
        }

        remainingElements %= elementsPerStep;
      }
      if(remainingElements)
      {
        if(clearRegisters == 0)
        {
          a.xorps(x86::xmm0, x86::xmm0);
          clearRegisters = 1;
        }
        for(unsigned int i = 0; i < remainingElements; i++)
          a.movss(a.ptr_zsi(i * sizeof(float)), x86::xmm(i % clearRegisters));
      }

      // Clear bottom border
      const bool bottomAligned = (input.dims(0) + p.padding[ZeroPadding2DLayer::TOP]) * (input.dims(1) + p.padding[ZeroPadding2DLayer::LEFT] + p.padding[ZeroPadding2DLayer::RIGHT]) * input.dims(2) % 4 == 0;
      a.add(a.zsi(), imm((input.dims(0) * (input.dims(1) + p.padding[ZeroPadding2DLayer::LEFT] + p.padding[ZeroPadding2DLayer::RIGHT]) * input.dims(2) + remainingElements) * sizeof(float)));
      remainingElements = p.padding[ZeroPadding2DLayer::BOTTOM] * (input.dims(1) + p.padding[ZeroPadding2DLayer::LEFT] + p.padding[ZeroPadding2DLayer::RIGHT]) * input.dims(2);
      for(unsigned int stepSize = settings.xmmRegs(); stepSize; --stepSize)
      {
        const unsigned int elementsPerStep = stepSize * 4;
        if(remainingElements < elementsPerStep)
          continue;

        if(clearRegisters < stepSize)
        {
          for(unsigned int i = clearRegisters; i < stepSize; i++)
            a.xorps(x86::xmm(i), x86::xmm(i));
          clearRegisters = stepSize;
        }

        Label clearBottomLoop;
        if(remainingElements >= elementsPerStep * 2)
        {
          clearBottomLoop = a.newLabel();
          a.mov(a.zcx(), imm(remainingElements / elementsPerStep));
          a.bind(clearBottomLoop);
        }

        for(unsigned int i = 0; i < stepSize; i++)
        {
          if(bottomAligned)
            a.movaps(a.ptr_zsi(i * 4 * sizeof(float)), x86::xmm(i));
          else
            a.movups(a.ptr_zsi(i * 4 * sizeof(float)), x86::xmm(i));
        }

        a.add(a.zsi(), imm(stepSize * 4 * sizeof(float)));

        if(remainingElements >= elementsPerStep * 2)
        {
          a.dec(a.zcx());
          a.jnz(clearBottomLoop);
        }

        remainingElements %= elementsPerStep;
      }

      if(remainingElements)
      {
        if(clearRegisters == 0)
        {
          a.xorps(x86::xmm0, x86::xmm0);
          clearRegisters = 1;
        }
        for(unsigned int i = 0; i < remainingElements; i++)
          a.movss(a.ptr_zsi(i * sizeof(float)), x86::xmm(i % clearRegisters));
      }

      // Clear left and right borders to zero
      a.mov(a.zsi(), imm(output.data() + p.padding[ZeroPadding2DLayer::TOP] * (input.dims(1) + p.padding[ZeroPadding2DLayer::LEFT] + p.padding[ZeroPadding2DLayer::RIGHT]) * input.dims(2)));
      a.mov(a.zcx(), imm(input.dims(0)));
      Label clearLeftAndRightLoop = a.newLabel();
      a.bind(clearLeftAndRightLoop);
      unsigned int paddingRemainingL = p.padding[ZeroPadding2DLayer::LEFT] * input.dims(2);
      unsigned int paddingRemainingR = p.padding[ZeroPadding2DLayer::RIGHT] * input.dims(2);
      unsigned int offset = 0;
      for(; paddingRemainingL >= 4; paddingRemainingL -= 4)
      {
        a.movups(a.ptr_zsi(offset * sizeof(float)), x86::xmm0);
        offset += 4;
      }
      for(; paddingRemainingL; --paddingRemainingL)
      {
        a.movss(a.ptr_zsi(offset * sizeof(float)), x86::xmm0);
        offset++;
      }
      offset = 0;
      for(; paddingRemainingR >= 4; paddingRemainingR -= 4)
      {
        a.movups(a.ptr_zsi(((input.dims(1) + p.padding[ZeroPadding2DLayer::LEFT]) * input.dims(2) + offset) * sizeof(float)), x86::xmm0);
        offset += 4;
      }
      for(; paddingRemainingR; --paddingRemainingR)
      {
        a.movss(a.ptr_zsi(((input.dims(1) + p.padding[ZeroPadding2DLayer::LEFT]) * input.dims(2) + offset) * sizeof(float)), x86::xmm0);
        offset++;
      }

      a.add(a.zsi(), imm((input.dims(1) + p.padding[ZeroPadding2DLayer::LEFT] + p.padding[ZeroPadding2DLayer::RIGHT]) * input.dims(2) * sizeof(float)));

      a.dec(a.zcx());
      a.jnz(clearLeftAndRightLoop);
    }
  }
}
