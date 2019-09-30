/**
 * @author Felix Thielke
 */

#include "Dense.h"
#include "../ActivationFunctions.h"
#include "Platform/BHAssert.h"
#include "Tools/Math/NeumaierSum.h"

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    void DenseCompiler::initialize()
    {
      // Declare constants
      constants.resize(2);

      // Store weights
      ASSERT(p.weights->rank() == 2);
      NetworkConstants& weights = constants[0];
      if(p.weights->dims(1) == 1)
      {
        weights.data.assign(p.weights->begin(), p.weights->end());
        if(p.preBatchNormalization)
        {
          for(size_t i = 0; i < weights.data.size(); i++)
            weights.data[i] *= (*p.preBatchNormalization->factor)[i];
        }
        if(p.postBatchNormalization && p.activationDesc == CompiledActivationFunctionId::linear)
        {
          for(float& w : weights.data)
            w *= (*p.postBatchNormalization->factor)[0];
        }
      }
      else
      {
        weights.data.clear();

        outputBatchSize = 4 * (settings.xmmRegs() - std::max(std::max(2u, ActivationFunctionHandler::neededSpares(p.activationDesc)), ActivationFunctionHandler::neededSpares(p.postActivation)));

        for(unsigned int outputOffset = 0; outputOffset < p.weights->dims(1); outputOffset += outputBatchSize)
        {
          const unsigned int outputBatchEnd = std::min(outputOffset + outputBatchSize, p.weights->dims(1));

          for(unsigned int input = 0; input < p.weights->dims(0); input += 4)
          {
            const unsigned int remainingInputs = std::min(4u, p.weights->dims(0) - input);

            for(unsigned int shuffle = remainingInputs; shuffle; --shuffle)
            {
              for(unsigned int output = outputOffset; output < outputBatchEnd; output += 4)
              {
                const unsigned int remainingOutputs = std::min(4u, outputBatchEnd - output);

                for(unsigned int i = 0; i < remainingOutputs; i++)
                {
                  const size_t effInputIndex = input + ((remainingInputs - shuffle + i) % remainingInputs);
                  const size_t effOutputIndex = output + i;

                  float w = (*p.weights)[effInputIndex * p.weights->dims(1) + effOutputIndex];
                  if(p.preBatchNormalization)
                    w *= (*p.preBatchNormalization->factor)[effInputIndex];
                  if(p.postBatchNormalization && p.activationDesc == CompiledActivationFunctionId::linear)
                    w *= (*p.postBatchNormalization->factor)[effOutputIndex];

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
        biases.data.resize(p.weights->dims(1), 0.f);
      if(p.preBatchNormalization)
      {
        for(size_t output = 0; output < p.weights->dims(1); output++)
        {
          NeumaierSum<float> sum;
          for(size_t input = 0; input < p.weights->dims(0); input++)
            sum += (*p.weights)(input, output) * (*p.preBatchNormalization->offset)[input];
          biases.data[output] += sum;
        }
      }
      if(p.postBatchNormalization && p.activationDesc == CompiledActivationFunctionId::linear)
      {
        for(size_t i = 0; i < biases.data.size(); i++)
          biases.data[i] = biases.data[i] * (*p.postBatchNormalization->factor)[i] + (*p.postBatchNormalization->offset)[i];
      }

      // If implicit Batch Normalization is not possible, store the normalization constants
      if(p.postBatchNormalization && p.activationDesc != CompiledActivationFunctionId::linear)
      {
        constants.resize(4);
        constants[2].data = *p.postBatchNormalization->factor;
        constants[3].data = *p.postBatchNormalization->offset;
      }
    }

    void DenseCompiler::compileInputBatch(x86::Assembler& a, const unsigned int remainingOutputs, const unsigned int stepSize, const unsigned int remainingInputs, const bool lastOutputBatch, const bool lastInputBatch) const
    {
      // Read input
      if(remainingInputs == 1)
        a.movss(x86::xmm(settings.xmmRegs() - 1), a.ptr_zsi());
      else
        a.movaps(x86::xmm(settings.xmmRegs() - 1), a.ptr_zsi());
      if(remainingInputs != 4)
        a.shufps(x86::xmm(settings.xmmRegs() - 1), x86::xmm(settings.xmmRegs() - 1), imm(0u | ((1 % remainingInputs) << 2) | ((2 % remainingInputs) << 4) | ((3 % remainingInputs) << 6)));
      if(!lastInputBatch)
        a.add(a.zsi(), imm(4 * sizeof(float)));

      // Multiply with weights
      unsigned int weightOffset = 0;
      for(unsigned int shuffle = remainingInputs; shuffle; --shuffle)
      {
        for(unsigned int step = 0; step < stepSize; step++)
        {
          if(step == stepSize - 1 && remainingOutputs % 4 == 1)
          {
            a.movss(x86::xmm(settings.xmmRegs() - 2), a.ptr_zdx(weightOffset));
            a.mulss(x86::xmm(settings.xmmRegs() - 2), x86::xmm(settings.xmmRegs() - 1));
            a.addss(x86::xmm(step), x86::xmm(settings.xmmRegs() - 2));
          }
          else
          {
            a.movaps(x86::xmm(settings.xmmRegs() - 2), a.ptr_zdx(weightOffset));
            a.mulps(x86::xmm(settings.xmmRegs() - 2), x86::xmm(settings.xmmRegs() - 1));
            a.addps(x86::xmm(step), x86::xmm(settings.xmmRegs() - 2));
          }
          weightOffset += 4 * sizeof(float);
        }

        if(shuffle > 1)
          a.shufps(x86::xmm(settings.xmmRegs() - 1), x86::xmm(settings.xmmRegs() - 1), imm((1 % remainingInputs) | ((2 % remainingInputs) << 2) | ((3 % remainingInputs) << 4) | ((4 % remainingInputs) << 6)));
      }

      // Adjust weight offset if necessary
      if(!lastOutputBatch || (!lastInputBatch && (p.weights->dims(0) / 4 >= 2 || p.weights->dims(0) % 4 != 0)))
        a.add(a.zdx(), imm(weightOffset));
    }

    void DenseCompiler::compileOutputBatch(x86::Assembler& a, ActivationFunctionHandler& afHandler, const float* const input, const unsigned int remainingOutputs, const bool last) const
    {
      const unsigned int stepSize = (remainingOutputs + 3) / 4;

      // Initialize results with zero
      for(unsigned int step = 0; step < stepSize; step++)
        a.xorps(x86::xmm(step), x86::xmm(step));

      // Initialize input pointer
      a.mov(a.zsi(), imm(input));

      if(p.weights->dims(0) > 4)
      {
        // Begin loop over input (only construct loop if it has more than one iteration)
        Label inputLoop;
        if(p.weights->dims(0) / 4 >= 2)
        {
          inputLoop = a.newLabel();
          a.mov(a.zcx(), imm(p.weights->dims(0) / 4));
          a.bind(inputLoop);
        }

        compileInputBatch(a, remainingOutputs, stepSize, 4, last);

        // End loop over input
        if(p.weights->dims(0) / 4 >= 2)
        {
          a.dec(a.zcx());
          a.jnz(inputLoop);
        }
      }

      const unsigned int remainingInputs = p.weights->dims(0) == 4 ? 4 : (p.weights->dims(0) % 4);
      if(remainingInputs)
        compileInputBatch(a, remainingOutputs, stepSize, remainingInputs, last, true);

      // Add biases
      for(unsigned int step = 0; step < stepSize; step++)
        a.addps(x86::xmm(step), a.ptr_zbx(step * 4 * sizeof(float)));

      if(p.activationDesc != CompiledActivationFunctionId::linear)
      {
        // Apply activation function
        ActivationFn& activationFunction = afHandler.prepare(p.activationDesc, false, a, { x86::xmm(settings.xmmRegs() - 2), x86::xmm(settings.xmmRegs() - 1) }, {});
        for(unsigned int step = 0; step < stepSize; step++)
          activationFunction.addValue(x86::xmm(step));
        for(unsigned int step = stepSize; step < settings.xmmRegs() - 2; step++)
          activationFunction.addSpare(x86::xmm(step));
        activationFunction.initialize(a);
        activationFunction.apply(a);

        // In this case, Batch Normalization was not done implicitly, so we have to do it manually
        if(p.postBatchNormalization)
        {
          // Multiply with factors
          if(settings.useX64)
            a.add(a.zbx(), x86::r8);
          else
            a.add(a.zbx(), a.ptr_zbp(-8, 4));
          for(unsigned int step = 0; step < stepSize; step++)
            a.mulps(x86::xmm(step), a.ptr_zbx(step * 4 * sizeof(float)));

          // Add offsets
          if(settings.useX64)
            a.add(a.zbx(), x86::r9);
          else
            a.add(a.zbx(), a.ptr_zbp(-4, 4));
          for(unsigned int step = 0; step < stepSize; step++)
            a.addps(x86::xmm(step), a.ptr_zbx(step * 4 * sizeof(float)));

          // Reset bias pointer
          if(settings.useX64)
          {
            a.sub(a.zbx(), x86::r9);
            a.sub(a.zbx(), x86::r8);
          }
          else
          {
            a.sub(a.zbx(), a.ptr_zbp(-4, 4));
            a.sub(a.zbx(), a.ptr_zbp(-8, 4));
          }
        }
      }
      if(p.postActivation != CompiledActivationFunctionId::linear)
      {
        // Apply activation function
        ActivationFn& activationFunction = afHandler.prepare(p.postActivation, false, a, { x86::xmm(settings.xmmRegs() - 2), x86::xmm(settings.xmmRegs() - 1) }, {});
        for(unsigned int step = 0; step < stepSize; step++)
          activationFunction.addValue(x86::xmm(step));
        for(unsigned int step = stepSize; step < settings.xmmRegs() - 2; step++)
          activationFunction.addSpare(x86::xmm(step));
        activationFunction.initialize(a);
        activationFunction.apply(a);
      }

      // Advance bias pointer
      if(!last)
        a.add(a.zbx(), imm(stepSize * 4 * sizeof(float)));

      // Store results
      for(unsigned int step = 0; step < stepSize; step++)
        a.movaps(a.ptr_zdi(step * 4 * sizeof(float)), x86::xmm(step));

      // Advance destination pointer if necessary
      if(!last && (p.weights->dims(1) / outputBatchSize >= 2 || p.weights->dims(1) % outputBatchSize != 0))
        a.add(a.zdi(), imm(stepSize * 4 * sizeof(float)));
    }

    void DenseCompiler::compileSimple(x86::Assembler& a, ActivationFunctionHandler& afHandler, const float* const input, const float* const output) const
    {
      // Declare labels
      const NetworkConstants& weights = constants[0];
      const NetworkConstants& biases = constants[1];

      a.mov(a.zsi(), imm(input));

      bool destInZSI = input == output;

      if(!destInZSI)
        a.mov(a.zdi(), imm(output));

      if(p.weights->dims(0) == 1)
      {
        // Only one input
        a.movss(x86::xmm0, a.ptr_zsi());
        a.mulss(x86::xmm0, x86::ptr(weights.label));
      }
      else if(p.weights->dims(0) <= 4)
      {
        // Input fits into one XMM register
        a.movaps(x86::xmm0, a.ptr_zsi());
        // On the V6, dpps has 15 cycles delay, while mulps+haddps is only 11 cycles.
        // Therefore, dpps should not be used for exactly 2 inputs.
        if(settings.useSSE42 && p.weights->dims(0) > 2)
        {
          unsigned char mask = 1;
          for(unsigned int i = 0; i < p.weights->dims(0); i++)
            mask |= (1 << (4 + i));
          a.dpps(x86::xmm0, x86::ptr(weights.label), imm(mask));
        }
        else
        {
          a.mulps(x86::xmm0, x86::ptr(weights.label));
          if(p.weights->dims(0) == 3)
            a.pslldq(x86::xmm0, imm(sizeof(float)));
          a.haddps(x86::xmm0, x86::xmm0);
          if(p.weights->dims(0) > 2)
            a.haddps(x86::xmm0, x86::xmm0);
        }
      }
      else if(p.weights->dims(0) <= (4 * settings.xmmRegs()) && (p.weights->dims(0) % 4 == 0))
      {
        // Input dimension is divisible by a multiple of 4 up to 32 (or 64 on x64)
        // (input fits completely into the XMM registers)
        const unsigned int stepSize = p.weights->dims(0) / 4;

        // Load weights address
        a.lea(a.zdx(), x86::ptr(weights.label));

        // Load channels
        for(unsigned int i = 0; i < stepSize; ++i)
          a.movaps(x86::xmm(i), a.ptr_zsi(i * 4 * sizeof(float)));

        // Multiply channels with weights
        for(unsigned int i = 0; i < stepSize; ++i)
          a.mulps(x86::xmm(i), a.ptr_zdx(i * 4 * sizeof(float)));

        // Accumulate results
        bool odd = stepSize % 2 != 0;
        for(unsigned int stride = stepSize / 2; stride; stride /= 2)
        {
          for(unsigned int i = 0; i < stride; i++)
            a.addps(x86::xmm(i), x86::xmm(i + stride));
          if(odd)
            a.addps(x86::xmm0, x86::xmm(stride * 2));
          odd = stride % 2 != 0;
        }
        a.haddps(x86::xmm0, x86::xmm0);
        a.haddps(x86::xmm0, x86::xmm0);
      }
      else
      {
        // Default case
        // Copy dest pointer to zdi
        if(destInZSI)
        {
          a.mov(a.zdi(), a.zsi());
          destInZSI = false;
        }

        // Load weights address
        a.lea(a.zdx(), x86::ptr(weights.label));

        // Initialise result
        a.xorps(x86::xmm0, x86::xmm0);

        unsigned int remainingChannels = p.weights->dims(0);
        for(unsigned int stepSize = settings.xmmRegs() - 1; stepSize; --stepSize)
        {
          if(remainingChannels >= stepSize * 4)
          {
            // Begin loop if the stepsize can be applied multiple times
            Label loop;
            if(remainingChannels >= stepSize * 8)
            {
              a.mov(a.zcx(), imm(remainingChannels / (stepSize * 4)));
              loop = a.newLabel();
              a.bind(loop);
            }

            // Multiply channels with weights
            for(unsigned int i = 0; i < stepSize; ++i)
              a.movaps(x86::xmm(i + 1), a.ptr_zsi(i * 4 * sizeof(float)));
            for(unsigned int i = 0; i < stepSize; ++i)
              a.mulps(x86::xmm(i + 1), a.ptr_zdx(i * 4 * sizeof(float)));

            // Accumulate results
            bool odd = (stepSize + 1) % 2 != 0;
            for(unsigned int stride = (stepSize + 1) / 2; stride; stride /= 2)
            {
              for(unsigned int i = 0; i < stride; i++)
                a.addps(x86::xmm(i), x86::xmm(i + stride));
              if(odd)
                a.addps(x86::xmm0, x86::xmm(stride * 2));
              odd = stride % 2 != 0;
            }

            // Increment input and weight pointers if there will be further steps
            if(remainingChannels != stepSize * 4)
            {
              a.add(a.zsi(), imm(stepSize * 4 * sizeof(float)));
              a.add(a.zdx(), imm(stepSize * 4 * sizeof(float)));
            }

            // End loop if the stepsize can be applied multiple times
            if(remainingChannels >= stepSize * 8)
            {
              a.dec(a.zcx());
              a.jne(loop);
            }

            remainingChannels %= stepSize * 4;
          }
        }
        if(remainingChannels == 1)
        {
          a.movss(x86::xmm1, a.ptr_zsi());
          a.mulss(x86::xmm1, a.ptr_zdx());
          a.addss(x86::xmm0, x86::xmm1);
        }
        else if(remainingChannels > 0)
        {
          for(unsigned int i = 0; i < remainingChannels; ++i)
            a.movss(x86::xmm(i + 1), a.ptr_zsi(i * sizeof(float)));
          for(unsigned int i = 0; i < remainingChannels; ++i)
            a.mulss(x86::xmm(i + 1), a.ptr_zdx(i * sizeof(float)));
          for(unsigned int i = 0; i < remainingChannels; ++i)
            a.addss(x86::xmm0, x86::xmm(i + 1));
        }
        a.haddps(x86::xmm0, x86::xmm0);
        a.haddps(x86::xmm0, x86::xmm0);
      }

      // Add bias
      a.addss(x86::xmm0, x86::ptr(biases.label));

      if(p.activationDesc != CompiledActivationFunctionId::linear)
      {
        // Apply activation function
        ActivationFn& activationFunction = afHandler.prepare(p.activationDesc, true, a, { }, { x86::xmm0 });
        for(unsigned int i = 1; i < settings.xmmRegs(); i++)
          activationFunction.addSpare(x86::xmm(i));
        activationFunction.initialize(a);
        activationFunction.apply(a);

        if(p.postBatchNormalization)
        {
          // In this case, Batch Normalization was not done implicitly, so we have to do it manually
          ASSERT(constants.size() == 4);

          // Multiply with factor
          a.mulss(x86::xmm0, x86::ptr(constants[2].label));

          // Add offset
          a.addss(x86::xmm0, x86::ptr(constants[3].label));
        }
      }
      if(p.postActivation != CompiledActivationFunctionId::linear)
      {
        // Apply activation function
        ActivationFn& activationFunction = afHandler.prepare(p.postActivation, true, a, { }, { x86::xmm0 });
        for(unsigned int i = 1; i < settings.xmmRegs(); i++)
          activationFunction.addSpare(x86::xmm(i));
        activationFunction.initialize(a);
        activationFunction.apply(a);
      }

      // Store result
      a.movss(destInZSI ? a.ptr_zsi() : a.ptr_zdi(), x86::xmm0);
    }

    void DenseCompiler::compile(x86::Assembler& a, ActivationFunctionHandler& afHandler, const TensorPointerXf& input, const TensorPointerXf& output) const
    {
      ASSERT(input.rank() == 1);
      ASSERT(output.rank() == 1);
      ASSERT(input.dims(0) == p.weights->dims(0));
      ASSERT(output.dims(0) == p.weights->dims(1));

      // Handle the special case of only one output
      if(p.weights->dims(1) == 1)
      {
        compileSimple(a, afHandler, input.data(), output.data());
        return;
      }

      // Declare labels
      const NetworkConstants& weights = constants[0];
      const NetworkConstants& biases = constants[1];

      // Load offsets
      a.lea(a.zdx(), x86::ptr(weights.label));
      a.mov(a.zdi(), imm(output.data()));
      a.lea(a.zbx(), x86::ptr(biases.label));

      if(p.activationDesc != CompiledActivationFunctionId::linear && p.postBatchNormalization)
      {
        // Store coefficient offsets
        ASSERT(constants.size() == 4);
        a.lea(a.zax(), x86::ptr(constants[2].label));
        a.lea(a.zcx(), x86::ptr(constants[3].label));
        a.sub(a.zcx(), a.zax());
        a.sub(a.zax(), a.zbx());
        if(settings.useX64)
        {
          a.mov(x86::r8, a.zax());
          a.mov(x86::r9, a.zcx());
        }
        else
        {
          a.mov(a.ptr_zbp(-8, 4), x86::eax);
          a.mov(a.ptr_zbp(-4, 4), x86::ecx);
        }
      }

      if(p.weights->dims(1) > outputBatchSize)
      {
        // Begin loop over output batches (only construct loop if it has more than one iteration)
        Label outputBatchLoop;
        if(p.weights->dims(1) / outputBatchSize >= 2)
        {
          outputBatchLoop = a.newLabel();
          a.mov(a.zax(), imm(p.weights->dims(1) / outputBatchSize));
          a.bind(outputBatchLoop);
        }

        compileOutputBatch(a, afHandler, input.data(), outputBatchSize);

        // End loop over output batches
        if(p.weights->dims(1) / outputBatchSize >= 2)
        {
          a.dec(a.zax());
          a.jnz(outputBatchLoop);
        }
      }

      const unsigned int remainingOutputs = p.weights->dims(1) == outputBatchSize ? outputBatchSize : (p.weights->dims(1) % outputBatchSize);
      if(remainingOutputs)
        compileOutputBatch(a, afHandler, input.data(), remainingOutputs, true);
    }
  }
}
