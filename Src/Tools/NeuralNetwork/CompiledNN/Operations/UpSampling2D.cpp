/**
 * @author Arne Hasselbring
 */

#include "UpSampling2D.h"
#include "Platform/BHAssert.h"

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    void UpSampling2DCompiler::compile(x86::Assembler& a, ActivationFunctionHandler& afHandler, const TensorPointerXf& input, const TensorPointerXf& output) const
    {
      ASSERT(input.rank() == 3);
      ASSERT(output.rank() == 3);

      ASSERT((input.dims(2) % 4) == 0); // TODO
      ASSERT(input.dims(2) <= settings.xmmRegs() * 4); // TODO

      a.mov(a.zsi(), imm(input.data()));
      a.mov(a.zdi(), imm(output.data()));

      Label rowLoop;
      if(input.dims(0) > 1)
      {
        rowLoop = a.newLabel();
        a.mov(a.zax(), imm(input.dims(0)));
        a.bind(rowLoop);
      }

      Label columnLoop;
      if(input.dims(1) > 1)
      {
        columnLoop = a.newLabel();
        a.mov(a.zcx(), imm(input.dims(1)));
        a.bind(columnLoop);
      }

      const unsigned int rowStep = output.dims(1) * output.dims(2) * sizeof(float);

      for(unsigned int i = 0; i < input.dims(2) / 4; ++i)
        a.movaps(x86::xmm(i), a.ptr_zsi(i * 4 * sizeof(float)));
      a.add(a.zsi(), imm(input.dims(2) * sizeof(float)));

      for(unsigned int j = 0; j < p.size[0]; ++j)
        for(unsigned int k = 0; k < p.size[1]; ++k)
          for(unsigned int i = 0; i < input.dims(2) / 4; ++i)
            a.movaps(a.ptr_zdi((k * input.dims(2) + i * 4) * sizeof(float) + j * rowStep), x86::xmm(i));

      a.add(a.zdi(), imm(p.size[1] * input.dims(2) * sizeof(float)));

      if(input.dims(1) > 1)
      {
        a.dec(a.zcx());
        a.jnz(columnLoop);
      }

      if(input.dims(0) > 1)
      {
        if(p.size[0] > 1)
          a.add(a.zdi(), imm((p.size[0] - 1) * rowStep));

        a.dec(a.zax());
        a.jnz(rowLoop);
      }
    }
  }
}
