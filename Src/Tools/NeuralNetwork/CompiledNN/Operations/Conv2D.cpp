/**
 * @author Felix Thielke
 */

#include "Conv2D.h"
#include "Platform/BHAssert.h"

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    void Conv2DCompiler::initialize()
    {
      outputBatchSize = 4 * (settings.xmmRegs() - std::max(std::max(2u, ActivationFunctionHandler::neededSpares(p.activationDesc)), ActivationFunctionHandler::neededSpares(p.postActivation)));

      // Declare constants
      constants.resize(2);

      // Store weights
      NetworkConstants& weights = constants[0];
      weights.data.clear();
      ASSERT(p.weights->rank() == 4);
      for(unsigned int outputOffset = 0; outputOffset < p.weights->dims(3); outputOffset += outputBatchSize)
      {
        const unsigned int outputBatchEnd = std::min(outputOffset + outputBatchSize, p.weights->dims(3));

        for(unsigned int y = 0; y < p.weights->dims(0); y++)
        {
          for(unsigned int input = 0; input < p.weights->dims(1) * p.weights->dims(2); input += 4)
          {
            const unsigned int remainingInputs = std::min(4u, p.weights->dims(1) * p.weights->dims(2) - input);

            for(unsigned int shuffle = remainingInputs; shuffle; --shuffle)
            {
              for(unsigned int output = outputOffset; output < outputBatchEnd; output += 4)
              {
                const unsigned int remainingOutputs = std::min(4u, outputBatchEnd - output);

                for(unsigned int i = 0; i < remainingOutputs; i++)
                {
                  const float w = (*p.weights)[(y * p.weights->dims(1) * p.weights->dims(2) + (input + ((remainingInputs - shuffle + i) % remainingInputs))) * p.weights->dims(3) + output + i];
                  if(p.batchNormalization && p.activationDesc == CompiledActivationFunctionId::linear)
                    weights.data.emplace_back(w * (*p.batchNormalization->factor)[output + i]);
                  else
                    weights.data.emplace_back(w);
                }
                for(unsigned int i = remainingOutputs; i < 4; i++)
                  weights.data.emplace_back(0.f);
              }
            }
          }
        }
      }

      // Store biases
      NetworkConstants& biases = constants[1];
      if(p.biases)
        biases.data = *p.biases;
      else
        biases.data.resize(p.weights->dims(3), 0.f);
      if(p.batchNormalization && p.activationDesc == CompiledActivationFunctionId::linear)
      {
        for(size_t i = 0; i < biases.data.size(); i++)
          biases.data[i] = biases.data[i] * (*p.batchNormalization->factor)[i] + (*p.batchNormalization->offset)[i];
      }

      // If implicit Batch Normalization is not possible, store the normalization constants
      if(p.batchNormalization && p.activationDesc != CompiledActivationFunctionId::linear)
      {
        constants.resize(4);
        constants[2].data = *p.batchNormalization->factor;
        constants[3].data = *p.batchNormalization->offset;
      }
    }

    void Conv2DCompiler::compileFilter(x86::Assembler& a, const bool inputAligned, const unsigned int remainingOutputs, const unsigned int remainingInput, const bool lastFilter) const
    {
      const unsigned int stepSize = (remainingOutputs + 3) / 4;

      // Load input values
      if(remainingInput == 1)
        a.movss(x86::xmm(settings.xmmRegs() - 1), a.ptr_zdx());
      else if(inputAligned)
        a.movaps(x86::xmm(settings.xmmRegs() - 1), a.ptr_zdx());
      else
        a.movups(x86::xmm(settings.xmmRegs() - 1), a.ptr_zdx());
      if(remainingInput != 4)
        a.shufps(x86::xmm(settings.xmmRegs() - 1), x86::xmm(settings.xmmRegs() - 1), imm(0u | ((1 % remainingInput) << 2) | ((2 % remainingInput) << 4) | ((3 % remainingInput) << 6)));
      if(!lastFilter)
        a.add(a.zdx(), imm(4 * sizeof(float)));

      // Apply filter
      unsigned int filterOffset = 0;
      for(unsigned int shuffle = remainingInput; shuffle; --shuffle)
      {
        for(unsigned int step = 0; step < stepSize; step++)
        {
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
        }

        if(shuffle > 1)
          a.shufps(x86::xmm(settings.xmmRegs() - 1), x86::xmm(settings.xmmRegs() - 1), imm((1 % remainingInput) | ((2 % remainingInput) << 2) | ((3 % remainingInput) << 4) | ((4 % remainingInput) << 6)));
      }
      a.add(a.zbx(), imm(filterOffset));
    }

    void Conv2DCompiler::compileOutputBatch(x86::Assembler& a, ActivationFunctionHandler& afHandler, const unsigned int inputWidth, const unsigned int remainingOutputs) const
    {
      const NetworkConstants& biases = constants[1];
      const bool inputAligned = (p.strides[1] * p.weights->dims(2)) % 4 == 0;
      const bool outputAligned = p.weights->dims(3) % 4 == 0;
      const unsigned int stepSize = (remainingOutputs + 3) / 4;

      // Initialize activation function
      ActivationFn& activationFn = afHandler.prepare(p.activationDesc, remainingOutputs == 1, a, {}, {});
      ActivationFn* postActivationFn = &(afHandler.prepare(p.postActivation, remainingOutputs == 1, a, {}, {}));
      bool activationFnInitialized = false;
      bool postActivationFnInitialized = false;
      for(unsigned int step = 0; step < stepSize; step++)
      {
        activationFn.addValue(x86::xmm(step));
        postActivationFn->addValue(x86::xmm(step));
      }
      if(ActivationFunctionHandler::neededSpares(p.activationDesc) <= settings.xmmRegs() - 2 - stepSize)
      {
        for(unsigned int i = stepSize; i < settings.xmmRegs() - 2; i++)
          activationFn.addSpare(x86::xmm(i));
        activationFn.initialize(a);
        activationFnInitialized = true;
      }
      else
        for(unsigned int i = stepSize; i < settings.xmmRegs(); i++)
          activationFn.addSpare(x86::xmm(i));
      if(activationFnInitialized && p.postActivation == p.activationDesc)
      {
        postActivationFn = &activationFn;
        postActivationFnInitialized = true;
      }
      else if((!activationFnInitialized || p.activationDesc == CompiledActivationFunctionId::linear) && ActivationFunctionHandler::neededSpares(p.postActivation) <= settings.xmmRegs() - 2 - stepSize)
      {
        for(unsigned int i = stepSize; i < settings.xmmRegs() - 2; i++)
          postActivationFn->addSpare(x86::xmm(i));
        postActivationFn->initialize(a);
        postActivationFnInitialized = true;
      }
      else
        for(unsigned int i = stepSize; i < settings.xmmRegs(); i++)
          postActivationFn->addSpare(x86::xmm(i));

      // Load input base address in zdx
      a.mov(a.zdx(), a.zsi());

      // Initialize filter result
      for(unsigned int step = 0; step < stepSize; step++)
        a.xorps(x86::xmm(step), x86::xmm(step));

      // Begin loop over weight rows
      Label filterRowLoop;
      if(p.weights->dims(0) > 1)
      {
        filterRowLoop = a.newLabel();
        a.mov(p.weights->dims(1) * p.weights->dims(2) > 4 ? a.zax() : a.zcx(), imm(p.weights->dims(0)));
        a.bind(filterRowLoop);
      }

      if(p.weights->dims(1) * p.weights->dims(2) > 4)
      {
        // Begin loop over weight cols
        Label filterColLoop = a.newLabel();
        a.mov(a.zcx(), imm(p.weights->dims(1) * p.weights->dims(2) / 4));
        a.bind(filterColLoop);

        compileFilter(a, inputAligned, remainingOutputs, 4);

        // End loop over weight cols
        a.dec(a.zcx());
        a.jnz(filterColLoop);
      }

      const unsigned int remainingInput = p.weights->dims(1) * p.weights->dims(2) == 4 ? 4 : ((p.weights->dims(1) * p.weights->dims(2)) % 4);
      if(remainingInput)
        compileFilter(a, inputAligned, remainingOutputs, remainingInput, true);

      // End loop over weight rows
      if(p.weights->dims(0) > 1)
      {
        // Set input pointer to next row
        a.add(a.zdx(), imm(((inputWidth - p.weights->dims(1)) * p.weights->dims(2) + remainingInput) * sizeof(float)));

        a.dec(p.weights->dims(1) * p.weights->dims(2) > 4 ? a.zax() : a.zcx());
        a.jnz(filterRowLoop);
      }

      // Add bias
      for(unsigned int step = 0; step < stepSize; step++)
      {
        if(step == stepSize - 1 && remainingOutputs % 4 == 1)
          a.addss(x86::xmm(step), x86::ptr(biases.label, biasOffset + step * 4 * sizeof(float)));
        else
          a.addps(x86::xmm(step), x86::ptr(biases.label, biasOffset + step * 4 * sizeof(float)));
      }

      // Apply activation function
      if(!activationFnInitialized)
        activationFn.initialize(a);
      activationFn.apply(a);

      // Apply Batch Normalization if it could not be done implicitly
      if(p.activationDesc != CompiledActivationFunctionId::linear && p.batchNormalization)
      {
        ASSERT(constants.size() == 4);

        // Multiply with factors
        for(unsigned int step = 0; step < stepSize; step++)
        {
          if(step == stepSize - 1 && remainingOutputs % 4 == 1)
            a.mulss(x86::xmm(step), x86::ptr(constants[2].label, biasOffset + step * 4 * sizeof(float)));
          else
            a.mulps(x86::xmm(step), x86::ptr(constants[2].label, biasOffset + step * 4 * sizeof(float)));
        }

        // Add offsets
        for(unsigned int step = 0; step < stepSize; step++)
        {
          if(step == stepSize - 1 && remainingOutputs % 4 == 1)
            a.addss(x86::xmm(step), x86::ptr(constants[3].label, biasOffset + step * 4 * sizeof(float)));
          else
            a.addps(x86::xmm(step), x86::ptr(constants[3].label, biasOffset + step * 4 * sizeof(float)));
        }
      }
      biasOffset += stepSize * 4 * sizeof(float);

      // Apply post activation function
      if(!postActivationFnInitialized)
        postActivationFn->initialize(a);
      postActivationFn->apply(a);

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

    void Conv2DCompiler::compileSimpleConvolution(x86::Assembler& a, ActivationFunctionHandler& afHandler, const unsigned int inputWidth, const unsigned int outputHeight, const unsigned int outputWidth) const
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

      // Prepare activation function
      const unsigned int activationRegsNeeded = ActivationFunctionHandler::neededSpares(p.activationDesc == CompiledActivationFunctionId::linear ? p.postActivation : p.activationDesc);
      ActivationFn* activationFn = &(afHandler.prepare(p.activationDesc == CompiledActivationFunctionId::linear ? p.postActivation : p.activationDesc, inputSize == 1, a, {}, { x86::xmm0 }));
      if(regsNeeded + activationRegsNeeded < settings.xmmRegs())
      {
        for(unsigned int i = regsNeeded; i < regsNeeded + activationRegsNeeded; i++)
          activationFn->addSpare(x86::xmm(i));
        activationFn->initialize(a);
      }

      // Load bias
      const unsigned int biasRegister = regsNeeded + activationRegsNeeded < settings.xmmRegs() ? regsNeeded + activationRegsNeeded : regsNeeded;
      if(biasRegister < settings.xmmRegs())
      {
        if(p.weights->dims(3) == 1)
          a.movss(x86::xmm(biasRegister), x86::ptr(constants[1].label));
        else
          a.movaps(x86::xmm(biasRegister), x86::ptr(constants[1].label));
      }

      // If some registers are free, load some weights
      const unsigned int weightRegisterOffset = biasRegister + 1;
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
              a.pshufd(x86::xmm(regOffset + filterRow * inputSize), a.ptr_zsi(sourceOffset), imm(0 | ((1 % inputSize) << 2) | ((2 % inputSize) << 4) | ((3 % inputSize) << 6)));
            else
              a.movaps(x86::xmm(regOffset + filterRow * inputSize), a.ptr_zsi(sourceOffset));
          }
          else
          {
            a.movups(x86::xmm(regOffset + filterRow * inputSize), a.ptr_zsi(sourceOffset));
            if(inputSize < 4)
              a.pshufd(x86::xmm(regOffset + filterRow * inputSize), x86::xmm(regOffset + filterRow * inputSize), imm(0 | ((1 % inputSize) << 2) | ((2 % inputSize) << 4) | ((3 % inputSize) << 6)));
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

      // Add bias
      if(biasRegister < settings.xmmRegs())
      {
        if(p.weights->dims(3) == 1)
          a.addss(x86::xmm0, x86::xmm(biasRegister));
        else
          a.addps(x86::xmm0, x86::xmm(biasRegister));
      }
      else
      {
        if(p.weights->dims(3) == 1)
          a.addss(x86::xmm0, x86::ptr(constants[1].label));
        else
          a.addps(x86::xmm0, x86::ptr(constants[1].label));
      }

      // Apply activation function
      if(regsNeeded + activationRegsNeeded >= settings.xmmRegs())
      {
        for(unsigned int i = 1; i < settings.xmmRegs(); i++)
          activationFn->addSpare(x86::xmm(i));
        activationFn->initialize(a);
      }
      activationFn->apply(a);

      // Apply batch normalization if it could not be done implicitly
      if(p.activationDesc != CompiledActivationFunctionId::linear && p.batchNormalization)
      {
        ASSERT(constants.size() == 4);
        if(p.weights->dims(3) == 1)
        {
          a.mulss(x86::xmm0, x86::ptr(constants[2].label));
          a.addss(x86::xmm0, x86::ptr(constants[3].label));
        }
        else
        {
          a.mulps(x86::xmm0, x86::ptr(constants[2].label));
          a.addps(x86::xmm0, x86::ptr(constants[3].label));
        }
      }

      // Apply post activation function
      if(p.postActivation != CompiledActivationFunctionId::linear)
      {
        if(p.postActivation == p.activationDesc)
          activationFn->apply(a);
        else
        {
          activationFn = &(afHandler.prepare(p.postActivation, inputSize == 1, a, { }, { x86::xmm0 }));
          for(unsigned int i = 1; i < settings.xmmRegs(); i++)
            activationFn->addSpare(x86::xmm(i));
          activationFn->initialize(a);
          activationFn->apply(a);
        }
      }

      // Store result
      if(p.weights->dims(3) == 1)
        a.movss(a.ptr_zdi(), x86::xmm0);
      else if(p.weights->dims(3) == 4)
        a.movaps(a.ptr_zdi(), x86::xmm0);
      else
        a.movups(a.ptr_zdi(), x86::xmm0);
      a.add(a.zdi(), imm(p.weights->dims(3) * sizeof(float)));

      // End loop over cols
      if(outputWidth > 1)
      {
        a.add(a.zsi(), imm(p.strides[1] * p.weights->dims(2) * sizeof(float)));
        a.dec(a.zcx());
        a.jnz(filterLoop);
      }

      // End loop over rows
      if(outputHeight > 1)
      {
        if(p.strides[0] * inputWidth != outputWidth * p.strides[1] || outputWidth <= 1)
          a.add(a.zsi(), imm(((p.strides[0] * inputWidth - outputWidth * p.strides[1]) * p.weights->dims(2) + (outputWidth <= 1 ? p.strides[1] * p.weights->dims(2) : 0)) * sizeof(float)));
        a.dec(a.zax());
        a.jnz(rowLoop);
      }
    }

    void Conv2DCompiler::compile(x86::Assembler& a, ActivationFunctionHandler& afHandler, const TensorPointerXf& input, const TensorPointerXf& output) const
    {
      ASSERT(input.rank() == 3);
      ASSERT(output.rank() == 3);
      ASSERT(input.dims(2) == p.weights->dims(2));
      ASSERT(output.dims(2) == p.weights->dims(3));

      const NetworkConstants& weights = constants[0];
      unsigned int inputWidth = input.dims(1);

      // Load input/output base addresses
      a.mov(a.zsi(), imm(input.data()));
      if(input.data() == output.data())
        a.mov(a.zdi(), a.zsi());
      else
        a.mov(a.zdi(), imm(output.data()));

      if(p.weights->dims(3) <= 4 && p.weights->dims(1) * p.weights->dims(2) <= 4)
        compileSimpleConvolution(a, afHandler, inputWidth, output.dims(0), output.dims(1));
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
        if(p.weights->dims(3) / outputBatchSize < 2 && p.weights->dims(1) * p.weights->dims(2) <= 4)
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

        if(p.weights->dims(3) > outputBatchSize)
        {
          // Begin loop over output batches (only construct loop if it has more than one iteration)
          Label outputBatchLoop;
          if(p.weights->dims(3) / outputBatchSize >= 2)
          {
            outputBatchLoop = a.newLabel();
            if(p.weights->dims(1) * p.weights->dims(2) > 4)
            {
              if(settings.useX64)
                a.mov(x86::r10d, imm(p.weights->dims(3) / outputBatchSize));
              else
                a.mov(a.ptr_zbp(-12, 4), imm(p.weights->dims(3) / outputBatchSize));
            }
            else
              a.mov(a.zax(), imm(p.weights->dims(3) / outputBatchSize));
            a.bind(outputBatchLoop);
          }

          compileOutputBatch(a, afHandler, inputWidth, outputBatchSize);

          // End loop over output batches
          if(p.weights->dims(3) / outputBatchSize >= 2)
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

        const unsigned int remainingOutputs = p.weights->dims(3) == outputBatchSize ? outputBatchSize : p.weights->dims(3) % outputBatchSize;
        if(remainingOutputs)
          compileOutputBatch(a, afHandler, inputWidth, remainingOutputs);

        // Set input offset to next column, respecting the stride
        a.add(a.zsi(), imm(p.strides[1] * p.weights->dims(2) * sizeof(float)));

        // End loop over output image cols
        if(p.weights->dims(3) / outputBatchSize < 2 && p.weights->dims(1) * p.weights->dims(2) <= 4)
          a.dec(a.zax());
        else if(settings.useX64)
          a.dec(x86::r9d);
        else
          a.dec(a.ptr_zbp(-8, 4));
        a.jnz(inputColLoop);

        // Set input offset to next row, respecting the stride
        if(p.strides[0] * inputWidth != output.dims(1) * p.strides[1])
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
