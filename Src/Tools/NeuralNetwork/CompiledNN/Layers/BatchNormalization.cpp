/**
 * @author Felix Thielke
 */

#include "BatchNormalization.h"

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    void BatchNormalizationLayerCompiler::initialize()
    {
      // Define constants
      constants.resize(1);
      NetworkConstants& norm = constants.back();

      if(l.dimension == 2)
      {
        switch(l.inputSize)
        {
          case 1:
            paramLength = 4;
            norm.data.resize(8);
            norm.data[3] = norm.data[2] = norm.data[1] = norm.data[0] = l.factor[0];
            norm.data[7] = norm.data[6] = norm.data[5] = norm.data[4] = l.offset[0];
            break;
          case 2:
            paramLength = 4;
            norm.data.resize(8);
            norm.data[2] = norm.data[0] = l.factor[0];
            norm.data[3] = norm.data[1] = l.factor[1];
            norm.data[6] = norm.data[4] = l.offset[0];
            norm.data[7] = norm.data[5] = l.offset[1];
            break;
          default:
            paramLength = 0;
            while(paramLength == 0 || paramLength % 4 != 0)
            {
              paramLength += l.inputSize;
              for(unsigned int i = 0; i < l.inputSize; i++)
                norm.data.emplace_back(l.factor[i]);
            }
            for(unsigned int c = paramLength / l.inputSize; c; --c)
              for(unsigned int i = 0; i < l.inputSize; i++)
                norm.data.emplace_back(l.offset[i]);
        }
      }
      else
      {
        paramLength = ((l.inputSize + 3) & (~3));
        norm.data.resize(paramLength * 2);
        memcpy(norm.data.data(), l.factor.data(), l.inputSize * sizeof(float));
        memcpy(norm.data.data() + paramLength, l.offset.data(), l.inputSize * sizeof(float));
      }
    }

    void BatchNormalizationLayerCompiler::compile(X86Assembler& a, ActivationFunctionHandler& afHandler, const float* const data) const
    {
      // Declare label
      const NetworkConstants& norm = constants.back();

      a.mov(a.zdi(), imm_ptr<const float*>(data));

      // Apply normalization
      if(l.inputDimensions[(l.dimension + 1) % 3] == 1 && l.inputDimensions[(l.dimension + 2) % 3] == 1)    // Normalize a vector
      {
        unsigned int batchSize = settings.xmmRegs();
        for(; batchSize; batchSize--)
          if(paramLength % (batchSize * 4) == 0)
            break;

        a.mov(a.zcx(), imm_u((l.inputSize + 3) / (4 * batchSize)));
        a.lea(a.zsi(), x86::ptr(norm.label));
        Label loop = a.newLabel();
        a.bind(loop);
        for(unsigned int i = 0; i < batchSize; i++)
          a.movaps(x86::xmm(i), a.ptr_zdi(16 * i));
        for(unsigned int i = 0; i < batchSize; i++)
          a.mulps(x86::xmm(i), a.ptr_zsi(16 * i));
        for(unsigned int i = 0; i < batchSize; i++)
          a.addps(x86::xmm(i), a.ptr_zsi(paramLength * sizeof(float) + 16 * i));
        for(unsigned int i = 0; i < batchSize; i++)
          a.movaps(a.ptr_zdi(16 * i), x86::xmm(i));
        a.add(a.zsi(), imm_u(16 * batchSize));
        a.add(a.zdi(), imm_u(16 * batchSize));
        if(batchSize < 7)
          a.loop(loop);
        else
        {
          a.dec(a.zcx());
          a.jnz(loop);
        }
      }
      else if(l.dimension == 2)    // Normalize deepest dimension
      {
        if(l.inputSize <= 2)    // Hold multiple normalization vectors in one XMM register
        {
          a.movaps(x86::xmm(settings.xmmRegs() - 2), x86::ptr(norm.label));
          a.movaps(x86::xmm(settings.xmmRegs() - 1), x86::ptr(norm.label, paramLength * sizeof(float)));

          unsigned int steps = static_cast<unsigned int>(l.inputDimensions[0] * l.inputDimensions[1] * l.inputDimensions[2] + 3) / 4;
          unsigned int batchSize = settings.xmmRegs() - 2;
          for(; batchSize; batchSize--)
            if(steps % batchSize == 0)
              break;
          steps /= batchSize;

          a.mov(a.zcx(), imm_u(steps));
          Label loop = a.newLabel();
          a.bind(loop);
          for(unsigned int i = 0; i < batchSize; i++)
          {
            a.movaps(x86::xmm(i), a.ptr_zdi(16 * i));
            a.mulps(x86::xmm(i), x86::xmm(settings.xmmRegs() - 2));
            a.addps(x86::xmm(i), x86::xmm(settings.xmmRegs() - 1));
            a.movaps(a.ptr_zdi(16 * i), x86::xmm(i));
          }
          a.add(a.zdi(), imm_u(16 * batchSize));
          if(batchSize <= 6)
            a.loop(loop);
          else
          {
            a.dec(a.zcx());
            a.jne(loop);
          }
        }
        else
        {
          if(paramLength == 4)   // Normalization vector fits into a single register
          {
            a.mov(a.zcx(), imm_u(static_cast<unsigned int>(l.inputDimensions[0] * l.inputDimensions[1] * l.inputDimensions[2] + 3) / 4));
            Label loop = a.newLabel();
            a.bind(loop);
            a.movaps(x86::xmm0, a.ptr_zdi());
            a.mulps(x86::xmm0, x86::ptr(norm.label));
            a.addps(x86::xmm0, x86::ptr(norm.label, 16));
            a.movaps(a.ptr_zdi(), x86::xmm0);
            a.add(a.zdi(), imm_u(16));
            a.loop(loop);
          }
          else // Iterate over normalization vector offset
          {
            a.mov(a.zcx(), imm_u(static_cast<unsigned int>(l.inputDimensions[0] * l.inputDimensions[1] * l.inputDimensions[2] + 3) / 4 + 1));
            Label resetParamPtr = a.newLabel();
            Label end = a.newLabel();
            a.bind(resetParamPtr);
            a.dec(a.zcx());
            a.jz(end);
            a.mov(a.zax(), imm_u(paramLength / 4));
            a.lea(a.zsi(), x86::ptr(norm.label));
            Label loop = a.newLabel();
            a.bind(loop);
            a.movaps(x86::xmm0, a.ptr_zdi());
            a.mulps(x86::xmm0, a.ptr_zsi());
            a.addps(x86::xmm0, a.ptr_zsi(paramLength * sizeof(float)));
            a.add(a.zsi(), imm_u(16));
            a.movaps(a.ptr_zdi(), x86::xmm0);
            a.add(a.zdi(), imm_u(16));
            a.dec(a.zax());
            a.jz(resetParamPtr);
            a.loop(loop);
            a.bind(end);
          }
        }
      }
      else
      {
        ASSERT(false);
      }
    }
  }
}
