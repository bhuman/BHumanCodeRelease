/**
 * @author Felix Thielke
 */

#include "GlobalPooling2D.h"
#include "Platform/BHAssert.h"

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    void GlobalPooling2DCompiler::initialize()
    {
      if(p.method == PoolingMethod::average)
      {
        constants.emplace_back();
        NetworkConstants& rImageSize = constants.back();
        rImageSize.data.resize(4);
        rImageSize.data[0] = rImageSize.data[1] = rImageSize.data[2] = rImageSize.data[3] = 1.f / static_cast<float>(p.imageSize);
      }
    }

    void GlobalPooling2DCompiler::compile(x86::Assembler& a, ActivationFunctionHandler& afHandler, const TensorPointerXf& input, const TensorPointerXf& output) const
    {
      ASSERT(input.rank() == 3);
      ASSERT(output.rank() == 1);

      const unsigned int channels = input.dims(2);
      ASSERT(channels == output.dims(0));
      const unsigned int imageSize = input.dims(0) * input.dims(1);
      ASSERT(imageSize > 0);
      ASSERT(p.method != PoolingMethod::average || imageSize == p.imageSize);
      const bool aligned = channels % 4 == 0;

      // Special case: no pooling is necessary
      if(imageSize == 1)
      {
        if(input.data() == output.data())
          return;
      }

      // Load input/output base addresses
      a.mov(a.zsi(), imm(input.data()));
      if(input.data() == output.data())
        a.mov(a.zdi(), a.zsi());
      else
        a.mov(a.zdi(), imm(output.data()));

      if(channels <= 2) // Pooling on the sub-register level
      {
        // TODO
        ASSERT(false);
      }
      else if(channels <= (settings.xmmRegs() - (aligned ? 1 : 0)) * 4) // Enough registers to keep the current results in registers at all times
      {
        const unsigned int channelRegs = (channels + 3) / 4;

        // Load initial values
        for(unsigned int i = 0; i < channelRegs; i++)
          a.movaps(x86::xmm(i), a.ptr_zsi(i * 4 * sizeof(float)));
        a.add(a.zsi(), imm(channels * sizeof(float)));

        // Begin loop
        Label loop;
        if(imageSize > 2)
        {
          a.mov(a.zcx(), imm(imageSize));
          loop = a.newLabel();
          a.bind(loop);
        }

        if(!aligned)
        {
          // Inputs are not aligned -> we need additional registers to load the values into
          unsigned int curChannelRegOffset = 0;
          for(unsigned int stepSize = std::min(channelRegs, settings.xmmRegs() - channelRegs); curChannelRegOffset != channelRegs; stepSize = std::min(channelRegs - curChannelRegOffset, settings.xmmRegs() - channelRegs))
          {
            while(curChannelRegOffset + stepSize <= channelRegs)
            {
              if(curChannelRegOffset == channelRegs - 1 && channels % 4 == 1)
              {
                // Exactly one value remains
                if(p.method == PoolingMethod::average)
                  a.addss(x86::xmm(curChannelRegOffset), a.ptr_zsi(curChannelRegOffset * 4 * sizeof(float)));
                else
                  a.maxss(x86::xmm(curChannelRegOffset), a.ptr_zsi(curChannelRegOffset * 4 * sizeof(float)));
              }
              else
              {
                // Read values from memory
                for(unsigned int i = 0; i < stepSize; stepSize++)
                  a.movups(x86::xmm(channelRegs + i), a.ptr_zsi((curChannelRegOffset + i) * 4 * sizeof(float)));

                // Pool values
                if(p.method == PoolingMethod::average)
                {
                  for(unsigned int i = 0; i < stepSize; stepSize++)
                    a.addps(x86::xmm(curChannelRegOffset + i), x86::xmm(channelRegs + i));
                }
                else
                {
                  for(unsigned int i = 0; i < stepSize; stepSize++)
                    a.maxps(x86::xmm(curChannelRegOffset + i), x86::xmm(channelRegs + i));
                }
              }

              curChannelRegOffset += stepSize;
            }
          }
        }
        else
        {
          // Inputs are aligned -> nothing special to worry about
          if(p.method == PoolingMethod::average)
          {
            for(unsigned int i = 0; i < channelRegs; i++)
              a.addps(x86::xmm(i), a.ptr_zsi(i * 4 * sizeof(float)));
          }
          else
          {
            for(unsigned int i = 0; i < channelRegs; i++)
              a.maxps(x86::xmm(i), a.ptr_zsi(i * 4 * sizeof(float)));
          }
        }

        // End loop
        if(imageSize > 2)
        {
          a.add(a.zsi(), imm(channels * sizeof(float)));
          a.dec(a.zcx());
          a.jnz(loop);
        }

        // Calculate the average (if a register is available)
        if(p.method == PoolingMethod::average && settings.xmmRegs() > channelRegs)
        {
          a.movaps(x86::xmm(settings.xmmRegs() - 1), x86::ptr(constants[0].label));

          for(unsigned int i = 0; i < channelRegs; i++)
            a.mulps(x86::xmm(i), x86::xmm(settings.xmmRegs() - 1));
        }

        // Write results
        for(unsigned int i = 0; i < channelRegs; i++)
          a.movaps(a.ptr_zdi(i * 4 * sizeof(float)), x86::xmm(i));

        // Calculate the average (if no registers were available)
        if(p.method == PoolingMethod::average && settings.xmmRegs() == channelRegs)
        {
          a.movaps(x86::xmm0, x86::ptr(constants[0].label));

          for(unsigned int i = 1; i < channelRegs; i++)
            a.movaps(x86::xmm(i), x86::xmm0);
          for(unsigned int i = 0; i < channelRegs; i++)
            a.mulps(x86::xmm(i), a.ptr_zdi(i * 4 * sizeof(float)));
          for(unsigned int i = 0; i < channelRegs; i++)
            a.movaps(a.ptr_zdi(i * 4 * sizeof(float)), x86::xmm(i));
        }
      }
      else
      {
        // TODO
        ASSERT(false);
      }
    }
  }
}
