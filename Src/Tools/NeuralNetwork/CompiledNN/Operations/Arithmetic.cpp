/**
 * @author Arne Hasselbring
 */

#include "Arithmetic.h"

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    void ArithmeticCompiler::initialize()
    {
      if(p.op == avg)
      {
        constants.emplace_back();
        NetworkConstants& rNumber = constants.back();
        rNumber.data.resize(4);
        rNumber.data[0] = rNumber.data[1] = rNumber.data[2] = rNumber.data[3] = 1.f / static_cast<float>(p.inputSize);
      }
    }

    void ArithmeticCompiler::compile(x86::Assembler& a, ActivationFunctionHandler& afHandler,
                                     const std::vector<TensorPointerXf>& input, const std::vector<TensorPointerXf>& output) const
    {
      ASSERT(output.size() == 1);
      std::size_t inplaceIndex = std::numeric_limits<std::size_t>::max();
      for(std::size_t i = 0; i < input.size(); ++i)
        if(input[i].data() == output[0].data())
        {
          inplaceIndex = i;
          break;
        }

      std::function<void(unsigned int dst, unsigned int src)> doOperation, doOperationSingle;
      switch(p.op)
      {
        case add:
          doOperation = [&a](unsigned int dst, unsigned int src) { a.addps(x86::xmm(dst), x86::xmm(src)); };
          doOperationSingle = [&a](unsigned int dst, unsigned int src) { a.addss(x86::xmm(dst), x86::xmm(src)); };
          break;
        case sub:
          ASSERT(input.size() == 2);
          doOperation = [&a](unsigned int dst, unsigned int src) { a.subps(x86::xmm(dst), x86::xmm(src)); };
          doOperationSingle = [&a](unsigned int dst, unsigned int src) { a.subss(x86::xmm(dst), x86::xmm(src)); };
          break;
        case mul:
          doOperation = [&a](unsigned int dst, unsigned int src) { a.mulps(x86::xmm(dst), x86::xmm(src)); };
          doOperationSingle = [&a](unsigned int dst, unsigned int src) { a.mulss(x86::xmm(dst), x86::xmm(src)); };
          break;
        case avg:
          doOperation = [&a](unsigned int dst, unsigned int src) { a.addps(x86::xmm(dst), x86::xmm(src)); };
          doOperationSingle = [&a](unsigned int dst, unsigned int src) { a.addss(x86::xmm(dst), x86::xmm(src)); };
          break;
        case max:
          doOperation = [&a](unsigned int dst, unsigned int src) { a.maxps(x86::xmm(dst), x86::xmm(src)); };
          doOperationSingle = [&a](unsigned int dst, unsigned int src) { a.maxss(x86::xmm(dst), x86::xmm(src)); };
          break;
        case min:
          doOperation = [&a](unsigned int dst, unsigned int src) { a.minps(x86::xmm(dst), x86::xmm(src)); };
          doOperationSingle = [&a](unsigned int dst, unsigned int src) { a.minss(x86::xmm(dst), x86::xmm(src)); };
          break;
        default:
          FAIL("Unknown operation.");
      }

      std::array<x86::Gp::Id, 4> availablePointerRegs = {{x86::Gp::kIdSi, x86::Gp::kIdAx, x86::Gp::kIdBx, x86::Gp::kIdDx}};

      std::size_t remainingInputs = input.size();
      std::size_t inputIndex = 0;
      while(remainingInputs)
      {
        std::vector<x86::Gp::Id> regs;
        a.mov(a.zdi(), imm(output[0].data()));
        if(!(remainingInputs == input.size() && inplaceIndex == std::numeric_limits<std::size_t>::max()))
        {
          regs.push_back(x86::Gp::kIdDi);
          if(remainingInputs == input.size())
            --remainingInputs;
        }
        for(std::size_t i = 0; i < std::min(availablePointerRegs.size(), remainingInputs); ++i)
        {
          regs.push_back(availablePointerRegs[i]);
          if(inputIndex == inplaceIndex)
            ++inputIndex;
          a.mov(a.gpz(regs.back()), imm(input[inputIndex++].data()));
        }

        remainingInputs -= std::min(availablePointerRegs.size(), remainingInputs);

        unsigned int remainingChannels = static_cast<unsigned int>(output[0].size());
        bool rNumberLoaded = false;
        for(unsigned int stepSize = settings.xmmRegs() / 2; stepSize; --stepSize)
        {
          const unsigned int channelsPerStep = stepSize * 4;
          if(remainingChannels < channelsPerStep)
            continue;

          if(!remainingInputs && p.op == avg && stepSize < settings.xmmRegs() / 2 && !rNumberLoaded)
          {
            rNumberLoaded = true;
            a.movaps(x86::xmm(settings.xmmRegs() - 1), x86::ptr(constants[0].label));
          }

          Label loop;
          if(remainingChannels >= channelsPerStep * 2)
          {
            loop = a.newLabel();
            a.mov(a.zcx(), imm(remainingChannels / channelsPerStep));
            a.bind(loop);
          }

          for(unsigned int step = 0; step < stepSize; step++)
            a.movaps(x86::xmm(step), a.ptr_base(regs[0], step * 4 * sizeof(float)));

          for(std::size_t i = 1; i < regs.size(); ++i)
          {
            for(unsigned int step = 0; step < stepSize; step++)
              a.movaps(x86::xmm(step + stepSize), a.ptr_base(regs[i], step * 4 * sizeof(float)));

            for(unsigned int step = 0; step < stepSize; step++)
              doOperation(step, step + stepSize);
          }

          if(!remainingInputs && p.op == avg)
          {
            // If all registers are in use, the reciprocal of the number of inputs must be loaded temporarily.
            // However, afterwards it will always be available, since it will not be overwritten until the loop is exited.
            if(!rNumberLoaded)
            {
              rNumberLoaded = true;
              a.movaps(x86::xmm(settings.xmmRegs() - 1), x86::ptr(constants[0].label));
            }
            for(unsigned int step = 0; step < stepSize; step++)
              a.mulps(x86::xmm(step), x86::xmm(settings.xmmRegs() - 1));
          }

          for(unsigned int step = 0; step < stepSize; step++)
            a.movaps(a.ptr_zdi(step * 4 * sizeof(float)), x86::xmm(step));

          if(remainingChannels != channelsPerStep)
          {
            for(std::size_t i = 0; i < regs.size(); ++i)
              a.add(a.gpz(regs[i]), imm(stepSize * 4 * sizeof(float)));
            if(regs[0] != x86::Gp::kIdDi)
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
          a.movss(x86::xmm0, a.ptr_base(regs[0]));
          // TODO: Measure whether it would be better to split the loop into two.
          for(std::size_t i = 1; i < regs.size(); ++i)
          {
            a.movss(x86::xmm1, a.ptr_base(regs[i]));
            doOperationSingle(0, 1);
          }
          if(!remainingInputs && p.op == avg)
            rNumberLoaded ? a.mulss(x86::xmm0, x86::xmm(settings.xmmRegs() - 1)) : a.mulss(x86::xmm0, x86::ptr(constants[0].label));
          a.movss(a.ptr_zdi(), x86::xmm0);
        }
        else if(remainingChannels)
        {
          ASSERT(remainingChannels <= 4);
          a.movaps(x86::xmm0, a.ptr_base(regs[0]));
          // TODO: Measure whether it would be better to split the loop into two.
          for(std::size_t i = 1; i < regs.size(); ++i)
          {
            a.movaps(x86::xmm1, a.ptr_base(regs[i]));
            doOperation(0, 1);
          }
          if(!remainingInputs && p.op == avg)
            rNumberLoaded ? a.mulps(x86::xmm0, x86::xmm(settings.xmmRegs() - 1)) : a.mulps(x86::xmm0, x86::ptr(constants[0].label));
          a.movaps(a.ptr_zdi(), x86::xmm0);
        }
      }
    }
  }
}
