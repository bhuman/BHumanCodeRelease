/**
 * @author Felix Thielke
 */

#include "UInt8Input.h"

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    void UInt8InputCompiler::initialize()
    {
      if(batchNormalization)
      {
        ASSERT(batchNormalization->dimension == 2);

        // Define constants
        constants.resize(1);
        NetworkConstants& norm = constants.back();

        switch(batchNormalization->inputSize)
        {
          case 1:
            paramLength = 4;
            norm.data.resize(8);
            norm.data[3] = norm.data[2] = norm.data[1] = norm.data[0] = batchNormalization->factor[0];
            norm.data[7] = norm.data[6] = norm.data[5] = norm.data[4] = batchNormalization->offset[0];
            break;
          case 2:
            paramLength = 4;
            norm.data.resize(8);
            norm.data[2] = norm.data[0] = batchNormalization->factor[0];
            norm.data[3] = norm.data[1] = batchNormalization->factor[1];
            norm.data[6] = norm.data[4] = batchNormalization->offset[0];
            norm.data[7] = norm.data[5] = batchNormalization->offset[1];
            break;
          default:
            paramLength = 0;
            while(paramLength == 0 || paramLength % 4 != 0)
            {
              paramLength += batchNormalization->inputSize;
              for(unsigned int i = 0; i < batchNormalization->inputSize; i++)
                norm.data.emplace_back(batchNormalization->factor[i]);
            }
            for(unsigned int c = paramLength / batchNormalization->inputSize; c; --c)
              for(unsigned int i = 0; i < batchNormalization->inputSize; i++)
                norm.data.emplace_back(batchNormalization->offset[i]);
        }
      }
    }

    void UInt8InputCompiler::compile(X86Assembler& a, ActivationFunctionHandler& afHandler, const float* const input, const float* const output) const
    {
      a.mov(a.zsi(), imm_ptr<const float*>(input));
      a.mov(a.zdi(), imm_ptr<const float*>(output));

      if(!batchNormalization || paramLength == 4)
      {
        ASSERT(dimensions[0] * dimensions[1] * dimensions[2] % 16 == 0);

        a.pxor(x86::xmm4, x86::xmm4);
        if(batchNormalization)
        {
          a.movaps(x86::xmm5, x86::ptr(constants[0].label));
          a.movaps(x86::xmm6, x86::ptr(constants[0].label, paramLength * sizeof(float)));
        }

        a.mov(a.zcx(), imm_u(dimensions[0] * dimensions[1] * dimensions[2] / 16));
        Label loop = a.newLabel();
        a.bind(loop);
        a.movdqa(x86::xmm0, a.ptr_zsi());
        a.movdqa(x86::xmm2, x86::xmm0);
        a.punpcklbw(x86::xmm0, x86::xmm4);
        a.punpckhbw(x86::xmm2, x86::xmm4);
        a.movdqa(x86::xmm1, x86::xmm0);
        a.movdqa(x86::xmm3, x86::xmm2);
        a.punpcklwd(x86::xmm0, x86::xmm4);
        a.punpckhwd(x86::xmm1, x86::xmm4);
        a.punpcklwd(x86::xmm2, x86::xmm4);
        a.punpckhwd(x86::xmm3, x86::xmm4);
        for(unsigned int i = 0; i < 4; i++)
          a.cvtdq2ps(x86::xmm(i), x86::xmm(i));
        if(batchNormalization)
        {
          for(unsigned int i = 0; i < 4; i++)
            a.mulps(x86::xmm(i), x86::xmm5);
          for(unsigned int i = 0; i < 4; i++)
            a.addps(x86::xmm(i), x86::xmm6);
        }
        for(unsigned int i = 0; i < 4; i++)
          a.movaps(a.ptr_zdi(i * 4 * sizeof(float)), x86::xmm(i));
        a.add(a.zsi(), imm_u(4 * 4));
        a.add(a.zdi(), imm_u(4 * 4 * sizeof(float)));
        a.loop(loop);
      }
      else
      {
        // TODO
        ASSERT(false);
      }
    }
  }
}
