/**
 * @author Felix Thielke
 */

#include "Pooling2D.h"

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    void Pooling2DLayerCompiler::initialize()
    {
      if(l.method == Pooling2DLayer::PoolingMethod::average && l.kernelSize[0] * l.kernelSize[1] > 1)
      {
        constants.resize(1);
        constants.back().data.clear();
        const float factor = 1.f / static_cast<float>(l.kernelSize[0] * l.kernelSize[1]);
        for(unsigned int i = 4; i; --i)
          constants.back().data.emplace_back(factor);
      }
    }

    void Pooling2DLayerCompiler::pool(X86Assembler& a, ActivationFunctionHandler& afHandler, const unsigned int paddingHorizontal, const unsigned int paddingVertical) const
    {
      const bool aligned = l.inputDimensions[2] % 4 == 0;
      const bool isPadded = paddingHorizontal + paddingVertical > 0;
      const unsigned int regsPerStep = aligned && !(isPadded && l.method == Pooling2DLayer::PoolingMethod::max) ? settings.xmmRegs() : settings.xmmRegs() - 1;
      const X86Xmm helperReg = aligned ? x86::xmm(settings.xmmRegs() - 1) : x86::xmm(settings.xmmRegs() - 2);

      if(!helperRegInitialized && (l.inputDimensions[2] + 3) / 4 < (aligned ? settings.xmmRegs() : settings.xmmRegs() - 1))
      {
        if(isPadded && l.method == Pooling2DLayer::PoolingMethod::max)
          a.xorps(helperReg, helperReg);
        else if(l.method == Pooling2DLayer::PoolingMethod::average && l.kernelSize[0] * l.kernelSize[1] > 1)
          a.movaps(helperReg, x86::ptr(constants.back().label));

        helperRegInitialized = true;
      }

      for(unsigned int channelOffset = 0; channelOffset < l.inputDimensions[2]; channelOffset += 4 * regsPerStep)
      {
        const unsigned int processedChannels = std::min(regsPerStep * 4, l.inputDimensions[2] - channelOffset);
        const unsigned int stepSize = (processedChannels + 3) / 4;

        // Apply filter
        bool first = true;
        for(unsigned int filterY = 0; filterY < l.kernelSize[0] - paddingVertical; filterY++)
        {
          for(unsigned int filterX = 0; filterX < l.kernelSize[1] - paddingHorizontal; filterX++)
          {
            unsigned int offset = ((filterY * l.inputDimensions[1] + filterX) * l.inputDimensions[2] + channelOffset) * sizeof(float);
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
                  if(l.method == Pooling2DLayer::PoolingMethod::average)
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
                      if(l.method == Pooling2DLayer::PoolingMethod::average)
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
                    if(l.method == Pooling2DLayer::PoolingMethod::average)
                      a.addps(x86::xmm(stepSize - (helper - i)), x86::xmm(i));
                    else // method == Pooling2DLayer::PoolingMethod::max
                      a.maxps(x86::xmm(stepSize - (helper - i)), x86::xmm(i));
                  }
                }
              }
            }
          }
        }

        if(isPadded && l.method == Pooling2DLayer::PoolingMethod::max)
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
        if(l.method == Pooling2DLayer::PoolingMethod::average && l.kernelSize[0] * l.kernelSize[1] > 1)
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

      a.add(a.zdi(), imm_u(l.inputDimensions[2] * sizeof(float)));
    }

    unsigned int Pooling2DLayerCompiler::poolRow(X86Assembler& a, ActivationFunctionHandler& afHandler, const unsigned int paddingLeft, const unsigned int paddingVertical) const
    {
      unsigned int offset = 0;

      // Pool left-padded cells
      unsigned int inputCol = 0;
      unsigned int outputCol = 0;
      for(; inputCol < paddingLeft; inputCol += l.strides[1], outputCol++)
      {
        pool(a, afHandler, paddingLeft - inputCol, paddingVertical);
      }
      if(inputCol > paddingLeft)
      {
        offset = (inputCol - paddingLeft) * l.inputDimensions[2] * sizeof(float);
        a.add(a.zsi(), imm_u(offset));
      }

      // Calculate number of non-padded cols
      unsigned int nonPaddedCols = 0;
      for(; inputCol < paddingLeft + l.inputDimensions[1] - l.kernelSize[1] + 1; inputCol += l.strides[1], outputCol++, nonPaddedCols++);

      // Begin loop over image cols
      a.mov(a.zcx(), imm_u(nonPaddedCols));
      Label inputColLoop = a.newLabel();
      a.bind(inputColLoop);

      // Pool current cell
      pool(a, afHandler, 0, paddingVertical);

      // Set input offset to next column, respecting the stride
      a.add(a.zsi(), imm_u(l.strides[1] * l.inputDimensions[2] * sizeof(float)));

      // End loop over image cols
      if(l.kernelSize[0] * l.kernelSize[1] * l.inputDimensions[2] <= 64)
        a.loop(inputColLoop);
      else
      {
        a.dec(a.zcx());
        a.jnz(inputColLoop);
      }
      offset += nonPaddedCols * l.strides[1] * l.inputDimensions[2] * sizeof(float);

      // Pool right-padded cells
      for(; outputCol < l.outputDimensions[1]; inputCol += l.strides[1], outputCol++)
      {
        pool(a, afHandler, inputCol + l.kernelSize[1] - (paddingLeft + l.inputDimensions[1]), paddingVertical);

        if(outputCol < l.outputDimensions[1] - 1)
        {
          offset += l.strides[1] * l.inputDimensions[2] * sizeof(float);
          a.add(a.zsi(), imm_u(l.strides[1] * l.inputDimensions[2] * sizeof(float)));
        }
      }

      return offset;
    }

    void Pooling2DLayerCompiler::compile(X86Assembler& a, ActivationFunctionHandler& afHandler, const float* const input, const float* const output) const
    {
      if(l.kernelSize[0] <= 1 && l.kernelSize[1] <= 1 && l.strides[0] <= 1 && l.strides[1] <= 1)
        return;

      // Calculate padding (cf. https://github.com/eigenteam/eigen-git-mirror/blob/master/unsupported/Eigen/CXX11/src/Tensor/TensorImagePatch.h#L262)
      const bool validPadding = l.padding == PaddingType::valid;
      const unsigned int paddingTop = validPadding ? 0 : ((l.outputDimensions[0] - 1) * l.strides[0] + l.kernelSize[0] - l.inputDimensions[0]) / 2;
      const unsigned int paddingLeft = validPadding ? 0 : ((l.outputDimensions[1] - 1) * l.strides[1] + l.kernelSize[1] - l.inputDimensions[1]) / 2;
      if(validPadding)
      {
        ASSERT(l.outputDimensions[0] == (l.inputDimensions[0] - l.kernelSize[0] + l.strides[0]) / l.strides[0]);
        ASSERT(l.outputDimensions[1] == (l.inputDimensions[1] - l.kernelSize[1] + l.strides[1]) / l.strides[1]);
      }
      else
      {
        ASSERT(l.outputDimensions[0] == (l.inputDimensions[0] + l.strides[0] - 1) / l.strides[0]);
        ASSERT(l.outputDimensions[1] == (l.inputDimensions[1] + l.strides[1] - 1) / l.strides[1]);
      }

      // Load input/output base addresses
      a.mov(a.zsi(), imm_ptr<>(input));
      if(isInplace())
        a.mov(a.zdi(), a.zsi());
      else
        a.mov(a.zdi(), imm_ptr<>(output));

      // Pool top-padded rows
      unsigned int inputRow = 0;
      unsigned int outputRow = 0;
      for(; inputRow < paddingTop; inputRow += l.strides[0], outputRow++)
      {
        const unsigned int offset = poolRow(a, afHandler, paddingLeft, paddingTop - inputRow);
        if(inputRow + l.strides[0] < paddingTop)
          a.sub(a.zsi(), imm_u(offset));
        else
          a.add(a.zsi(), imm((inputRow + l.strides[0] - paddingTop) * l.inputDimensions[1] * l.inputDimensions[2] * sizeof(float) - offset));
      }

      // Calculate number of non-padded rows
      unsigned int nonPaddedRows = 0;
      for(; inputRow < paddingTop + l.inputDimensions[0] - l.kernelSize[0] + 1; inputRow += l.strides[0], outputRow++, nonPaddedRows++);

      // Begin loop over image rows
      a.mov(a.zax(), imm_u(nonPaddedRows));
      Label inputRowLoop = a.newLabel();
      a.bind(inputRowLoop);

      // Apply pooling to current row
      const unsigned int offset = poolRow(a, afHandler, paddingLeft, 0);

      // Set input offset to next row, respecting the stride
      a.add(a.zsi(), imm_u(l.strides[0] * l.inputDimensions[1] * l.inputDimensions[2] * sizeof(float) - offset));

      // End loop over image rows
      a.dec(a.zax());
      a.jnz(inputRowLoop);

      // Pool bottom-padded rows
      for(; outputRow < l.outputDimensions[0]; inputRow += l.strides[0], outputRow++)
      {
        const unsigned int offset = poolRow(a, afHandler, paddingLeft, inputRow + l.kernelSize[0] - (paddingTop + l.inputDimensions[0]));

        if(outputRow < l.outputDimensions[0] - 1)
          a.add(a.zsi(), imm_u(l.strides[0] * l.inputDimensions[1] * l.inputDimensions[2] * sizeof(float) - offset));
      }
    }
  }
}
