/**
 * @author Felix Thielke
 */

#include "Pooling2D.h"
#include "Platform/BHAssert.h"

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    void Pooling2DCompiler::initialize()
    {
      if(p.method == PoolingMethod::average && p.kernelSize[0] * p.kernelSize[1] > 1)
      {
        constants.resize(1);
        constants.back().data.clear();
        const float factor = 1.f / static_cast<float>(p.kernelSize[0] * p.kernelSize[1]);
        for(unsigned int i = 4; i; --i)
          constants.back().data.emplace_back(factor);
      }
    }

    void Pooling2DCompiler::pool(x86::Assembler& a, ActivationFunctionHandler& afHandler, const unsigned int paddingHorizontal, const unsigned int paddingVertical, const unsigned int inputWidth, const unsigned int channels) const
    {
      const bool aligned = channels % 4 == 0;
      const bool isPadded = paddingHorizontal + paddingVertical > 0;
      const unsigned int regsPerStep = aligned && !(isPadded && p.method == PoolingMethod::max) ? settings.xmmRegs() : settings.xmmRegs() - 1;
      const x86::Xmm helperReg = aligned ? x86::xmm(settings.xmmRegs() - 1) : x86::xmm(settings.xmmRegs() - 2);

      if(!helperRegInitialized && (channels + 3) / 4 < (aligned ? settings.xmmRegs() : settings.xmmRegs() - 1))
      {
        if(isPadded && p.method == PoolingMethod::max)
          a.xorps(helperReg, helperReg);
        else if(p.method == PoolingMethod::average && p.kernelSize[0] * p.kernelSize[1] > 1)
          a.movaps(helperReg, x86::ptr(constants.back().label));

        helperRegInitialized = true;
      }

      for(unsigned int channelOffset = 0; channelOffset < channels; channelOffset += 4 * regsPerStep)
      {
        const unsigned int processedChannels = std::min(regsPerStep * 4, channels - channelOffset);
        const unsigned int stepSize = (processedChannels + 3) / 4;

        // Apply filter
        bool first = true;
        for(unsigned int filterY = 0; filterY < p.kernelSize[0] - paddingVertical; filterY++)
        {
          for(unsigned int filterX = 0; filterX < p.kernelSize[1] - paddingHorizontal; filterX++)
          {
            unsigned int offset = ((filterY * inputWidth + filterX) * channels + channelOffset) * sizeof(float);
            if(first)
            {
              for(unsigned int step = 0; step < stepSize; step++)
              {
                if(aligned)
                  a.movaps(x86::xmm(step), a.ptr_zsi(offset));
                else
                  a.movups(x86::xmm(step), a.ptr_zsi(offset));
                offset += 4 * sizeof(float);
              }

              first = false;
            }
            else
            {
              if(aligned)
              {
                for(unsigned int step = 0; step < stepSize; step++)
                {
                  if(p.method == PoolingMethod::average)
                    a.addps(x86::xmm(step), a.ptr_zsi(offset));
                  else // method == Pooling2DLayer::PoolingMethod::max
                    a.maxps(x86::xmm(step), a.ptr_zsi(offset));
                  offset += 4 * sizeof(float);
                }
              }
              else
              {
                const unsigned int helperOffset = stepSize;
                const unsigned int helperCount = settings.xmmRegs() - stepSize;
                unsigned int helper = 0;
                for(unsigned int step = 0; step < stepSize;)
                {
                  a.movups(x86::xmm(helperOffset + helper), a.ptr_zsi(offset));
                  step++;
                  offset += 4 * sizeof(float);
                  helper++;

                  if(helper == helperCount)
                  {
                    for(helper = 0; helper < helperCount; helper++)
                    {
                      if(p.method == PoolingMethod::average)
                        a.addps(x86::xmm(step - helperCount + helper), x86::xmm(helperOffset + helper));
                      else // method == Pooling2DLayer::PoolingMethod::max
                        a.maxps(x86::xmm(step - helperCount + helper), x86::xmm(helperOffset + helper));
                    }

                    helper = 0;
                  }
                }

                if(helper != stepSize)
                {
                  for(unsigned int i = stepSize; i < helper; i++)
                  {
                    if(p.method == PoolingMethod::average)
                      a.addps(x86::xmm(stepSize - (helper - i)), x86::xmm(i));
                    else // method == Pooling2DLayer::PoolingMethod::max
                      a.maxps(x86::xmm(stepSize - (helper - i)), x86::xmm(i));
                  }
                }
              }
            }
          }
        }

        if(isPadded && p.method == PoolingMethod::max)
        {
          if(!helperRegInitialized)
          {
            a.xorps(x86::xmm(settings.xmmRegs() - 1), x86::xmm(settings.xmmRegs() - 1));
            for(unsigned int step = 0; step < stepSize; step++)
              a.maxps(x86::xmm(step), x86::xmm(settings.xmmRegs() - 1));
          }
          else
          {
            a.xorps(helperReg, helperReg);
            for(unsigned int step = 0; step < stepSize; step++)
              a.maxps(x86::xmm(step), helperReg);
          }
        }
        if(p.method == PoolingMethod::average && p.kernelSize[0] * p.kernelSize[1] > 1)
        {
          if(!helperRegInitialized)
          {
            for(unsigned int step = 0; step < stepSize; step++)
              a.mulps(x86::xmm(step), x86::ptr(constants.back().label));
          }
          else
          {
            for(unsigned int step = 0; step < stepSize; step++)
              a.mulps(x86::xmm(step), helperReg);
          }
        }

        // Store results
        for(unsigned int step = 0; step < stepSize; step++)
        {
          if(aligned)
            a.movaps(a.ptr_zdi((channelOffset + step * 4) * sizeof(float)), x86::xmm(step));
          else
            a.movups(a.ptr_zdi((channelOffset + step * 4) * sizeof(float)), x86::xmm(step));
        }
      }

      a.add(a.zdi(), imm(channels * sizeof(float)));
    }

    unsigned int Pooling2DCompiler::poolRow(x86::Assembler& a, ActivationFunctionHandler& afHandler, const unsigned int paddingLeft, const unsigned int paddingVertical, const unsigned int inputWidth, const unsigned int outputWidth, const unsigned int channels) const
    {
      unsigned int offset = 0;

      // Pool left-padded cells
      unsigned int inputCol = 0;
      unsigned int outputCol = 0;
      for(; inputCol < paddingLeft; inputCol += p.strides[1], outputCol++)
      {
        pool(a, afHandler, paddingLeft - inputCol, paddingVertical, inputWidth, channels);
      }
      if(inputCol > paddingLeft)
      {
        offset = (inputCol - paddingLeft) * channels * sizeof(float);
        a.add(a.zsi(), imm(offset));
      }

      // Calculate number of non-padded cols
      unsigned int nonPaddedCols = 0;
      for(; inputCol < paddingLeft + inputWidth - p.kernelSize[1] + 1; inputCol += p.strides[1], outputCol++, nonPaddedCols++);

      // Begin loop over image cols
      a.mov(a.zcx(), imm(nonPaddedCols));
      Label inputColLoop = a.newLabel();
      a.bind(inputColLoop);

      // Pool current cell
      pool(a, afHandler, 0, paddingVertical, inputWidth, channels);

      // Set input offset to next column, respecting the stride
      a.add(a.zsi(), imm(p.strides[1] * channels * sizeof(float)));

      // End loop over image cols
      a.dec(a.zcx());
      a.jnz(inputColLoop);
      offset += nonPaddedCols * p.strides[1] * channels * sizeof(float);

      // Pool right-padded cells
      for(; outputCol < outputWidth; inputCol += p.strides[1], outputCol++)
      {
        pool(a, afHandler, inputCol + p.kernelSize[1] - (paddingLeft + inputWidth), paddingVertical, inputWidth, channels);

        if(outputCol < outputWidth - 1)
        {
          offset += p.strides[1] * channels * sizeof(float);
          a.add(a.zsi(), imm(p.strides[1] * channels * sizeof(float)));
        }
      }

      return offset;
    }

    void Pooling2DCompiler::compile(x86::Assembler& a, ActivationFunctionHandler& afHandler, const TensorPointerXf& input, const TensorPointerXf& output) const
    {
      ASSERT(input.rank() == 3);
      ASSERT(output.rank() == 3);

      if(p.kernelSize[0] <= 1 && p.kernelSize[1] <= 1 && p.strides[0] <= 1 && p.strides[1] <= 1)
        return;

      // Calculate padding (cf. https://github.com/eigenteam/eigen-git-mirror/blob/master/unsupported/Eigen/CXX11/src/Tensor/TensorImagePatch.h#L262)
      const bool validPadding = p.padding == PaddingType::valid;
      const unsigned int paddingTop = validPadding ? 0 : ((output.dims(0) - 1) * p.strides[0] + p.kernelSize[0] - input.dims(0)) / 2;
      const unsigned int paddingLeft = validPadding ? 0 : ((output.dims(1) - 1) * p.strides[1] + p.kernelSize[1] - input.dims(1)) / 2;
      if(validPadding)
      {
        ASSERT(output.dims(0) == (input.dims(0) - p.kernelSize[0] + p.strides[0]) / p.strides[0]);
        ASSERT(output.dims(1) == (input.dims(1) - p.kernelSize[1] + p.strides[1]) / p.strides[1]);
      }
      else
      {
        ASSERT(output.dims(0) == (input.dims(0) + p.strides[0] - 1) / p.strides[0]);
        ASSERT(output.dims(1) == (input.dims(1) + p.strides[1] - 1) / p.strides[1]);
      }

      // Load input/output base addresses
      a.mov(a.zsi(), imm(input.data()));
      if(input.data() == output.data())
        a.mov(a.zdi(), a.zsi());
      else
        a.mov(a.zdi(), imm(output.data()));

      // Pool top-padded rows
      unsigned int inputRow = 0;
      unsigned int outputRow = 0;
      for(; inputRow < paddingTop; inputRow += p.strides[0], outputRow++)
      {
        const unsigned int offset = poolRow(a, afHandler, paddingLeft, paddingTop - inputRow, input.dims(1), output.dims(1), input.dims(2));
        if(inputRow + p.strides[0] < paddingTop)
          a.sub(a.zsi(), imm(offset));
        else
          a.add(a.zsi(), imm((inputRow + p.strides[0] - paddingTop) * input.dims(1) * input.dims(2) * sizeof(float) - offset));
      }

      // Calculate number of non-padded rows
      unsigned int nonPaddedRows = 0;
      for(; inputRow < paddingTop + input.dims(0) - p.kernelSize[0] + 1; inputRow += p.strides[0], outputRow++, nonPaddedRows++);

      // Begin loop over image rows
      a.mov(a.zax(), imm(nonPaddedRows));
      Label inputRowLoop = a.newLabel();
      a.bind(inputRowLoop);

      // Apply pooling to current row
      const unsigned int offset = poolRow(a, afHandler, paddingLeft, 0, input.dims(1), output.dims(1), input.dims(2));

      // Set input offset to next row, respecting the stride
      a.add(a.zsi(), imm(p.strides[0] * input.dims(1) * input.dims(2) * sizeof(float) - offset));

      // End loop over image rows
      a.dec(a.zax());
      a.jnz(inputRowLoop);

      // Pool bottom-padded rows
      for(; outputRow < output.dims(0); inputRow += p.strides[0], outputRow++)
      {
        const unsigned int offset = poolRow(a, afHandler, paddingLeft, inputRow + p.kernelSize[0] - (paddingTop + input.dims(0)), input.dims(1), output.dims(1), input.dims(2));

        if(outputRow < output.dims(0) - 1)
          a.add(a.zsi(), imm(p.strides[0] * input.dims(1) * input.dims(2) * sizeof(float) - offset));
      }
    }
  }
}
