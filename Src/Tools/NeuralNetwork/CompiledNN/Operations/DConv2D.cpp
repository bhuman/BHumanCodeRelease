/**
 * @author Felix Thielke
 */

#include "../ActivationFunctions.h"
#include "DConv2D.h"

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    void DConv2DCompiler::initialize()
    {
      outputBatchSize = 4 * (settings.xmmRegs() - 2);

      // Declare constants
      constants.resize(1);
      // Store weights
      NetworkConstants& weights = constants[0];
      weights.data.clear();
      ASSERT(p.weights->rank() == 4);
      ASSERT(p.weights->dims(3) == 1); // TODO: depthMultiplier != 1 is not supported yet.
      const unsigned int outputChannels = p.weights->dims(2) * p.weights->dims(3);
      for(unsigned int outputOffset = 0; outputOffset < outputChannels; outputOffset += outputBatchSize)
      {
        const unsigned int outputBatchEnd = std::min(outputOffset + outputBatchSize, outputChannels);

        for(unsigned int y = 0; y < p.weights->dims(0); y++)
        {
          for(unsigned int input = 0; input < p.weights->dims(1) ; input += 1)
          {
            for(unsigned int output = outputOffset; output < outputBatchEnd; output += 4)
            {
              const unsigned int remainingOutputs = std::min(4u, outputBatchEnd - output);

              for(unsigned int i = 0; i < remainingOutputs; i++)
              {
                int row = y * p.weights->dims(1) * outputChannels;
                int col = input * outputChannels;
                weights.data.emplace_back((*p.weights)[row + col + output + i]);
              }
              for(unsigned int i = remainingOutputs; i < 4; i++)
                weights.data.emplace_back(0.f);
            }
          }
        }
      }
    }

    void DConv2DCompiler::compileFilter(x86::Assembler& a, const bool inputAligned, const unsigned int remainingOutputs, const unsigned int remainingInput, const bool lastFilter) const
    {
      const unsigned int stepSize = (remainingOutputs + 3) / 4;

      // Apply filter
      unsigned int filterOffset = 0;
      for(unsigned int step = 0; step < stepSize; step++)
      {
        // Load input values
        if(remainingOutputs == 1)
        {
          a.movss(x86::xmm(settings.xmmRegs() - 1), a.ptr_zdx());
          a.shufps(x86::xmm(settings.xmmRegs() - 1), x86::xmm(settings.xmmRegs() - 1), imm(0u));
        }
        else
        {
          if(inputAligned)
            a.movaps(x86::xmm(settings.xmmRegs() - 1), a.ptr_zdx());
          else
            a.movups(x86::xmm(settings.xmmRegs() - 1), a.ptr_zdx());

          //if(remainingOutputs < 4)
          //   a.shufps(x86::xmm(settings.xmmRegs() - 1), x86::xmm(settings.xmmRegs() - 1), imm(0u | ((1 % remainingOutputs) << 2) | ((2 % remainingOutputs) << 4) | ((3 % remainingOutputs) << 6)));
        }

        if(step == stepSize - 1 && remainingOutputs % 4 == 1)
        {
          a.movss(x86::xmm(settings.xmmRegs() - 2), a.ptr_zbx(filterOffset));
          a.mulss(x86::xmm(settings.xmmRegs() - 2), x86::xmm(settings.xmmRegs() - 1));
          a.addss(x86::xmm(step), x86::xmm(settings.xmmRegs() - 2));
        }
        else
        {
          a.movaps(x86::xmm(settings.xmmRegs() - 2), a.ptr_zbx(filterOffset));
          a.mulps(x86::xmm(settings.xmmRegs() - 2), x86::xmm(settings.xmmRegs() - 1));
          a.addps(x86::xmm(step), x86::xmm(settings.xmmRegs() - 2));
        }
        filterOffset += 4 * sizeof(float);
        a.add(a.zdx(), imm(4 * sizeof(float)));
      }

      a.add(a.zbx(), imm(filterOffset));
    }

    void DConv2DCompiler::compileOutputBatch(x86::Assembler& a, const unsigned int inputWidth, const unsigned int remainingOutputs) const
    {
      const bool inputAligned = (p.strides[1] * p.weights->dims(2)) % 4 == 0;
      const bool outputAligned = p.weights->dims(2) % 4 == 0;
      const unsigned int stepSize = (remainingOutputs + 3) / 4;

      // Load input base address in zdx
      a.mov(a.zdx(), a.zsi());

      // Initialize filter result
      for(unsigned int step = 0; step < stepSize; step++)
        a.xorps(x86::xmm(step), x86::xmm(step));

      // Begin loop over weight rows
      Label filterRowLoop = a.newLabel();
      a.mov(a.zax(), imm(p.weights->dims(0)));
      a.bind(filterRowLoop);

      // Begin loop over weight cols
      Label filterColLoop = a.newLabel();
      a.mov(a.zcx(), imm(p.weights->dims(1)));
      a.bind(filterColLoop);
      compileFilter(a, inputAligned, remainingOutputs, remainingOutputs, false);

      // End loop over weight cols
      a.dec(a.zcx());
      a.jnz(filterColLoop);

      // Set input pointer to next row
      a.add(a.zdx(), imm(((inputWidth - p.weights->dims(1)) * p.weights->dims(2) + 0) * sizeof(float)));

      // End loop over weight rows
      a.dec(a.zax());
      a.jnz(filterRowLoop);

      // Store output
      for(unsigned int step = 0; step < stepSize; step++)
      {
        if(step == stepSize - 1 && remainingOutputs % 4 == 1)
          a.movss(a.ptr_zdi(step * 4 * sizeof(float)), x86::xmm(step));
        else if(outputAligned)
          a.movaps(a.ptr_zdi(step * 4 * sizeof(float)), x86::xmm(step));
        else
          a.movups(a.ptr_zdi(step * 4 * sizeof(float)), x86::xmm(step));
      }
      a.add(a.zdi(), imm(remainingOutputs * sizeof(float)));
    }

    void DConv2DCompiler::compileSimpleConvolution(x86::Assembler& a, const unsigned int inputWidth, const unsigned int outputHeight, const unsigned int outputWidth) const
    {
      const unsigned int inputSize = p.weights->dims(1) * p.weights->dims(2);

      // Load weights address
      a.lea(a.zbx(), x86::ptr(constants[0].label));

      // Calculate number of needed registers
      unsigned int regsNeeded = 0;
      unsigned int regOffset = 0;
      for(unsigned int remainingRows = p.weights->dims(0); remainingRows;)
      {
        const unsigned int rowsInThisIteration = std::min((settings.xmmRegs() - regOffset) / inputSize, remainingRows);
        regsNeeded = std::max(regsNeeded, rowsInThisIteration * inputSize);
        remainingRows -= rowsInThisIteration;
        if(regOffset == 0)
          regOffset = 1;
      }

      // If some registers are free, load some weights
      const unsigned int weightRegisterOffset = regsNeeded + 1; // TODO check this
      const unsigned int weightRegisterCount = std::min(p.weights->dims(0) * inputSize, static_cast<unsigned int>(std::max(0, static_cast<int>(settings.xmmRegs()) - static_cast<int>(weightRegisterOffset))));
      const unsigned int weightRegisterMemoryOffset = (p.weights->dims(0) * inputSize - weightRegisterCount) * 4 * sizeof(float);
      for(unsigned int i = 0; i < weightRegisterCount; i++)
        a.movaps(x86::xmm(weightRegisterOffset + i), a.ptr_zbx(weightRegisterMemoryOffset + i * 4 * sizeof(float)));

      // Begin loop over rows
      Label rowLoop;
      if(outputHeight > 1)
      {
        a.mov(a.zax(), imm(outputHeight));
        rowLoop = a.newLabel();
        a.bind(rowLoop);
      }

      // Begin loop over cols
      Label filterLoop;
      if(outputWidth > 1)
      {
        a.mov(a.zcx(), imm(outputWidth));
        filterLoop = a.newLabel();
        a.bind(filterLoop);
      }

      // Apply filter
      unsigned int sourceOffset = 0;
      unsigned int filterOffset = 0;
      regOffset = 0;
      for(unsigned int remainingRows = p.weights->dims(0); remainingRows;)
      {
        const unsigned int rowsInThisIteration = std::min((settings.xmmRegs() - regOffset) / inputSize, remainingRows);

        for(unsigned int filterRow = 0; filterRow < rowsInThisIteration; filterRow++)
        {
          if((p.strides[1] * p.weights->dims(2)) % 4 == 0)
          {
            if(inputSize < 4)
              a.pshufd(x86::xmm(regOffset + filterRow * inputSize), a.ptr_zsi(sourceOffset), imm(0u | ((1 % inputSize) << 2) | ((2 % inputSize) << 4) | ((3 % inputSize) << 6)));
            else
              a.movaps(x86::xmm(regOffset + filterRow * inputSize), a.ptr_zsi(sourceOffset));
          }
          else
          {
            a.movups(x86::xmm(regOffset + filterRow * inputSize), a.ptr_zsi(sourceOffset));
            if(inputSize < 4)
              a.pshufd(x86::xmm(regOffset + filterRow * inputSize), x86::xmm(regOffset + filterRow * inputSize), imm(0u | ((1 % inputSize) << 2) | ((2 % inputSize) << 4) | ((3 % inputSize) << 6)));
          }
          sourceOffset += (inputWidth * p.weights->dims(2)) * sizeof(float);
        }
        for(unsigned int i = 1; i < inputSize; i++)
        {
          for(unsigned int filterRow = 0; filterRow < rowsInThisIteration; filterRow++)
            a.pshufd(x86::xmm(regOffset + filterRow * inputSize + i), x86::xmm(regOffset + filterRow * inputSize + i - 1), imm((1 % inputSize) | ((2 % inputSize) << 2) | ((3 % inputSize) << 4) | ((4 % inputSize) << 6)));
        }
        for(unsigned int i = 0; i < rowsInThisIteration * inputSize; i++)
        {
          if(filterOffset >= weightRegisterMemoryOffset)
            a.mulps(x86::xmm(regOffset + i), x86::xmm(weightRegisterOffset + (filterOffset - weightRegisterMemoryOffset) / (4 * sizeof(float))));
          else
            a.mulps(x86::xmm(regOffset + i), a.ptr_zbx(filterOffset));
          filterOffset += 4 * sizeof(float);
        }

        bool odd = (rowsInThisIteration * inputSize) % 2 != 0;
        for(unsigned int stride = (rowsInThisIteration * inputSize) / 2; stride; stride /= 2)
        {
          for(unsigned int i = 0; i < stride; i++)
            a.addps(x86::xmm(i + regOffset), x86::xmm(i + stride + regOffset));
          if(odd)
            a.addps(x86::xmm(regOffset), x86::xmm(stride * 2 + regOffset));
          odd = stride % 2 != 0;
        }
        if(regOffset)
          a.addps(x86::xmm0, x86::xmm1);

        remainingRows -= rowsInThisIteration;
        if(regOffset == 0)
          regOffset = 1;
      }

      // Store result
      if(p.weights->dims(2) == 1)
        a.movss(a.ptr_zdi(), x86::xmm0);
      else if(p.weights->dims(2) == 4)
        a.movaps(a.ptr_zdi(), x86::xmm0);
      else
        a.movups(a.ptr_zdi(), x86::xmm0);
      a.add(a.zdi(), imm(p.weights->dims(2) * sizeof(float)));

      // End loop over cols
      if(outputWidth > 1)
      {
        a.add(a.zsi(), imm(p.strides[1] * p.weights->dims(2) * sizeof(float)));
        a.dec(a.zcx());
        a.jne(filterLoop);
      }

      // End loop over rows
      if(outputHeight > 1)
      {
        a.add(a.zsi(), imm(((p.strides[0] * inputWidth - outputWidth * p.strides[1]) * p.weights->dims(2) + (outputWidth <= 1 ? p.strides[1] * p.weights->dims(2) : 0)) * sizeof(float)));
        a.dec(a.zax());
        a.jne(rowLoop);
      }
    }

    void DConv2DCompiler::compile(x86::Assembler& a, ActivationFunctionHandler& afHandler, const TensorPointerXf& input, const TensorPointerXf& output) const
    {
      ASSERT(input.rank() == 3);
      ASSERT(output.rank() == 3);
      ASSERT(input.dims(2) == p.weights->dims(2));
      ASSERT(output.dims(2) == p.weights->dims(2) * p.weights->dims(3));

      const NetworkConstants& weights = constants[0];
      unsigned int inputWidth = input.dims(1);

      // Load input/output base addresses
      a.mov(a.zsi(), imm(input.data()));
      if(input.data() == output.data())
        a.mov(a.zdi(), a.zsi());
      else
        a.mov(a.zdi(), imm(output.data()));

      if(p.weights->dims(2) <= 4 && p.weights->dims(1) * p.weights->dims(2) <= 4)
        compileSimpleConvolution(a, inputWidth, output.dims(0), output.dims(1));
      else
      {
        // Begin loop over output image rows
        if(settings.useX64)
          a.mov(x86::r8d, imm(output.dims(0)));
        else
          a.mov(a.ptr_zbp(-4, 4), imm(output.dims(0)));
        Label inputRowLoop = a.newLabel();
        a.bind(inputRowLoop);

        // Begin loop over output image cols
        if(p.weights->dims(2) / outputBatchSize < 2 && p.weights->dims(1) * p.weights->dims(2) <= 4)
          a.mov(a.zax(), imm(output.dims(1)));
        else if(settings.useX64)
          a.mov(x86::r9d, imm(output.dims(1)));
        else
          a.mov(a.ptr_zbp(-8, 4), imm(output.dims(1)));
        Label inputColLoop = a.newLabel();
        a.bind(inputColLoop);

        // Load filter base address
        a.lea(a.zbx(), x86::ptr(weights.label));

        biasOffset = 0;

        if(p.weights->dims(2) > outputBatchSize)
        {
          // Begin loop over output batches (only construct loop if it has more than one iteration)
          Label outputBatchLoop;
          if(p.weights->dims(2) / outputBatchSize >= 2)
          {
            outputBatchLoop = a.newLabel();
            if(p.weights->dims(1) * p.weights->dims(2) > 4)
            {
              if(settings.useX64)
                a.mov(x86::r10d, imm(p.weights->dims(2) / outputBatchSize));
              else
                a.mov(a.ptr_zbp(-12, 4), imm(p.weights->dims(2) / outputBatchSize));
            }
            else
              a.mov(a.zax(), imm(p.weights->dims(2) / outputBatchSize));
            a.bind(outputBatchLoop);
          }

          compileOutputBatch(a, inputWidth, outputBatchSize);

          // End loop over output batches
          if(p.weights->dims(2) / outputBatchSize >= 2)
          {
            if(p.weights->dims(1) * p.weights->dims(2) > 4)
            {
              if(settings.useX64)
                a.dec(x86::r10d);
              else
                a.dec(a.ptr_zbp(-12, 4));
            }
            else
              a.dec(a.zax());
            a.jnz(outputBatchLoop);
          }
        }

        const unsigned int remainingOutputs = p.weights->dims(2) == outputBatchSize ? outputBatchSize : p.weights->dims(2) % outputBatchSize;
        if(remainingOutputs)
          compileOutputBatch(a, inputWidth, remainingOutputs);

        // Set input offset to next column, respecting the stride
        a.add(a.zsi(), imm(p.strides[1] * p.weights->dims(2) * sizeof(float)));

        // End loop over output image cols
        if(output.dims(2) / outputBatchSize < 2 && p.weights->dims(1) * p.weights->dims(2) <= 4)
          a.dec(a.zax());
        else if(settings.useX64)
          a.dec(x86::r9d);
        else
          a.dec(a.ptr_zbp(-8, 4));
        a.jnz(inputColLoop);

        // Set input offset to next row, respecting the stride
        a.add(a.zsi(), imm((p.strides[0] * inputWidth - output.dims(1) * p.strides[1]) * p.weights->dims(2) * sizeof(float)));

        // End loop over output image rows
        if(settings.useX64)
          a.dec(x86::r8d);
        else
          a.dec(a.ptr_zbp(-4, 4));
        a.jnz(inputRowLoop);
      }
    }
  }
}
