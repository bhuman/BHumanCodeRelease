/**
 * @author Felix Thielke
 */

#include "BatchNormalization.h"
#include "Platform/BHAssert.h"

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    void BatchNormalizationCompiler::initialize()
    {
      // Define constants
      constants.resize(1);
      NetworkConstants& norm = constants.back();

      if(p.dimension == 2)
      {
        switch(p.inputSize)
        {
          case 1:
            paramLength = 4;
            norm.data.resize(8);
            norm.data[3] = norm.data[2] = norm.data[1] = norm.data[0] = (*p.factor)[0];
            norm.data[7] = norm.data[6] = norm.data[5] = norm.data[4] = (*p.offset)[0];
            break;
          case 2:
            paramLength = 4;
            norm.data.resize(8);
            norm.data[2] = norm.data[0] = (*p.factor)[0];
            norm.data[3] = norm.data[1] = (*p.factor)[1];
            norm.data[6] = norm.data[4] = (*p.offset)[0];
            norm.data[7] = norm.data[5] = (*p.offset)[1];
            break;
          default:
            paramLength = 0;
            while(paramLength == 0 || paramLength % 4 != 0)
            {
              paramLength += p.inputSize;
              for(unsigned int i = 0; i < p.inputSize; i++)
                norm.data.emplace_back((*p.factor)[i]);
            }
            for(unsigned int c = paramLength / p.inputSize; c; --c)
              for(unsigned int i = 0; i < p.inputSize; i++)
                norm.data.emplace_back((*p.offset)[i]);
        }
      }
      else
      {
        paramLength = ((p.inputSize + 3) & (~3));
        norm.data.resize(paramLength * 2);
        std::copy(p.factor->begin(), p.factor->end(), norm.data.begin());
        std::copy(p.offset->begin(), p.offset->end(), norm.data.begin() + p.inputSize);
      }
    }

    void BatchNormalizationCompiler::compile(x86::Assembler& a, ActivationFunctionHandler& afHandler, const TensorPointerXf& input, const TensorPointerXf& output) const
    {
      ASSERT(input.dims() == output.dims());
      ASSERT(p.dimension < input.rank());
      ASSERT(input.dims(p.dimension) == p.inputSize);

      // Declare label
      const NetworkConstants& norm = constants.back();

      const bool isInplace = input.data() == output.data();
      a.mov(a.zsi(), imm(input.data()));
      if(!isInplace)
        a.mov(a.zdi(), imm(output.data()));

      // Apply normalization
      if(input.rank() == 1)    // Normalize a vector
      {
        unsigned int batchSize = settings.xmmRegs();
        for(; batchSize; batchSize--)
          if(paramLength % (batchSize * 4) == 0)
            break;

        a.mov(a.zcx(), imm((p.inputSize + 3) / (4 * batchSize)));
        a.lea(a.zdx(), x86::ptr(norm.label));
        Label loop = a.newLabel();
        a.bind(loop);
        for(unsigned int i = 0; i < batchSize; i++)
          a.movaps(x86::xmm(i), a.ptr_zsi(i * 4 * sizeof(float)));
        for(unsigned int i = 0; i < batchSize; i++)
          a.mulps(x86::xmm(i), a.ptr_zdx(i * 4 * sizeof(float)));
        for(unsigned int i = 0; i < batchSize; i++)
          a.addps(x86::xmm(i), a.ptr_zdx((paramLength + i * 4) * sizeof(float)));
        for(unsigned int i = 0; i < batchSize; i++)
          a.movaps(isInplace ? a.ptr_zsi(i * 4 * sizeof(float)) : a.ptr_zdi(i * 4 * sizeof(float)), x86::xmm(i));
        a.add(a.zdx(), imm(batchSize * 4 * sizeof(float)));
        a.add(a.zsi(), imm(batchSize * 4 * sizeof(float)));
        if(!isInplace)
          a.add(a.zdi(), imm(batchSize * 4 * sizeof(float)));
        a.dec(a.zcx());
        a.jnz(loop);
      }
      else if(p.dimension == 2)    // Normalize deepest dimension
      {
        if(p.inputSize <= 2)    // Hold multiple normalization vectors in one XMM register
        {
          a.movaps(x86::xmm(settings.xmmRegs() - 2), x86::ptr(norm.label));
          a.movaps(x86::xmm(settings.xmmRegs() - 1), x86::ptr(norm.label, paramLength * sizeof(float)));

          unsigned int steps = static_cast<unsigned int>(input.size() + 3) / 4;
          unsigned int batchSize = settings.xmmRegs() - 2;
          for(; batchSize; batchSize--)
            if(steps % batchSize == 0)
              break;
          steps /= batchSize;

          a.mov(a.zcx(), imm(steps));
          Label loop = a.newLabel();
          a.bind(loop);
          for(unsigned int i = 0; i < batchSize; i++)
          {
            a.movaps(x86::xmm(i), a.ptr_zsi(16 * i));
            a.mulps(x86::xmm(i), x86::xmm(settings.xmmRegs() - 2));
            a.addps(x86::xmm(i), x86::xmm(settings.xmmRegs() - 1));
            a.movaps(isInplace ? a.ptr_zsi(16 * i) : a.ptr_zdi(16 * i), x86::xmm(i));
          }
          a.add(a.zsi(), imm(16 * batchSize));
          if(!isInplace)
            a.add(a.zdi(), imm(16 * batchSize));
          a.dec(a.zcx());
          a.jne(loop);
        }
        else
        {
          if(paramLength == 4)   // Normalization vector fits into a single register
          {
            a.mov(a.zcx(), imm(static_cast<unsigned int>(input.size() + 3) / 4));
            Label loop = a.newLabel();
            a.bind(loop);
            a.movaps(x86::xmm0, a.ptr_zsi());
            a.mulps(x86::xmm0, x86::ptr(norm.label));
            a.addps(x86::xmm0, x86::ptr(norm.label, 16));
            a.movaps(isInplace ? a.ptr_zsi() : a.ptr_zdi(), x86::xmm0);
            a.add(a.zsi(), imm(16u));
            if(!isInplace)
              a.add(a.zdi(), imm(16u));
            a.dec(a.zcx());
            a.jnz(loop);
          }
          else // Iterate over normalization vector offset
          {
            a.mov(a.zcx(), imm(static_cast<unsigned int>(input.size() + 3) / 4 + 1));
            Label resetParamPtr = a.newLabel();
            Label end = a.newLabel();
            a.bind(resetParamPtr);
            a.dec(a.zcx());
            a.jz(end);
            a.mov(a.zax(), imm(paramLength / 4));
            a.lea(a.zdx(), x86::ptr(norm.label));
            Label loop = a.newLabel();
            a.bind(loop);
            a.movaps(x86::xmm0, a.ptr_zsi());
            a.mulps(x86::xmm0, a.ptr_zdx());
            a.addps(x86::xmm0, a.ptr_zdx(paramLength * sizeof(float)));
            a.add(a.zdx(), imm(16u));
            a.movaps(isInplace ? a.ptr_zsi() : a.ptr_zdi(), x86::xmm0);
            a.add(a.zsi(), imm(16u));
            if(!isInplace)
              a.add(a.zdi(), imm(16u));
            a.dec(a.zax());
            a.jz(resetParamPtr);
            a.dec(a.zcx());
            a.jnz(loop);
            a.bind(end);
          }
        }
      }
      else
      {
        FAIL("BatchNormalization layer supports only axis 2 for 3-dimensional tensors or 0 for vectors.");
      }
    }
  }
}
