/**
 * @author Felix Thielke
 */

#include "../ActivationFunctions.h"
#include "Dense.h"

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    void DenseLayerCompiler::initialize()
    {
      // Declare constants
      constants.resize(2);

      // Store weights
      NetworkConstants& weights = constants[0];
      if(l.outputDimensions[0] == 1)
      {
        weights.data.assign(l.weights.begin(), l.weights.end());
        if(batchNormalization && l.activationId == ActivationFunctionId::linear)
        {
          for(float& w : weights.data)
            w *= batchNormalization->factor[0];
        }
      }
      else
      {
        weights.data.clear();

        constexpr unsigned int outputBatchSize = 4 * 6;
        for(unsigned int outputOffset = 0; outputOffset < l.outputDimensions[0]; outputOffset += outputBatchSize)
        {
          const unsigned int outputBatchEnd = std::min(outputOffset + outputBatchSize, l.outputDimensions[0]);

          for(unsigned int input = 0; input < l.inputDimensions[0]; input += 4)
          {
            const unsigned int remainingInputs = std::min(4u, l.inputDimensions[0] - input);

            for(unsigned int shuffle = remainingInputs; shuffle; --shuffle)
            {
              for(unsigned int output = outputOffset; output < outputBatchEnd; output += 4)
              {
                const unsigned int remainingOutputs = std::min(4u, outputBatchEnd - output);

                for(unsigned int i = 0; i < remainingOutputs; i++)
                {
                  const float w = l.weights[(input + ((remainingInputs - shuffle + i) % remainingInputs)) * l.outputDimensions[0] + output + i];
                  if(batchNormalization && l.activationId == ActivationFunctionId::linear)
                    weights.data.emplace_back(w * batchNormalization->factor[output + i]);
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
      biases.data = l.biases;
      if(batchNormalization && l.activationId == ActivationFunctionId::linear)
      {
        for(size_t i = 0; i < biases.data.size(); i++)
          biases.data[i] = biases.data[i] * batchNormalization->factor[i] + batchNormalization->offset[i];
      }

      // If implicit Batch Normalization is not possible, store the normalization constants
      if(batchNormalization && l.activationId != ActivationFunctionId::linear)
      {
        constants.resize(4);
        constants[2].data = batchNormalization->factor;
        constants[3].data = batchNormalization->offset;
      }
    }

    void DenseLayerCompiler::compileInputBatch(X86Assembler& a, const unsigned int remainingOutputs, const unsigned int stepSize, const unsigned int remainingInputs, const bool lastOutputBatch, const bool lastInputBatch) const
    {
      // Read input
      if(remainingInputs == 1)
      {
        a.movss(x86::xmm(settings.xmmRegs() - 1), a.ptr_zsi());
        a.shufps(x86::xmm(settings.xmmRegs() - 1), x86::xmm(settings.xmmRegs() - 1), imm_u(0));
      }
      else
      {
        a.movaps(x86::xmm(settings.xmmRegs() - 1), a.ptr_zsi());
        if(remainingInputs != 4)
          a.shufps(x86::xmm(settings.xmmRegs() - 1), x86::xmm(settings.xmmRegs() - 1), imm_u(0 | (1 << 2) | ((2 % remainingInputs) << 4) | ((3 % remainingInputs) << 6)));
      }
      if(!lastInputBatch)
        a.add(a.zsi(), imm_u(4 * sizeof(float)));

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
          a.shufps(x86::xmm(settings.xmmRegs() - 1), x86::xmm(settings.xmmRegs() - 1), imm_u(1 | (2 << 2) | (3 << 4)));
      }

      // Adjust weight offset if necessary
      if(!lastOutputBatch || (!lastInputBatch && (l.inputDimensions[0] / 4 >= 2 || l.inputDimensions[0] % 4 != 0)))
        a.add(a.zdx(), imm_u(weightOffset));
    }

    void DenseLayerCompiler::compileOutputBatch(X86Assembler& a, ActivationFunctionHandler& afHandler, const float* const input, const unsigned int remainingOutputs, const bool last) const
    {
      const unsigned int stepSize = (remainingOutputs + 3) / 4;

      // Initialize results with zero
      for(unsigned int step = 0; step < stepSize; step++)
        a.xorps(x86::xmm(step), x86::xmm(step));

      // Initialize input pointer
      a.mov(a.zsi(), imm_ptr<>(input));

      if(l.inputDimensions[0] > 4)
      {
        // Begin loop over input (only construct loop if it has more than one iteration)
        Label inputLoop;
        if(l.inputDimensions[0] / 4 >= 2)
        {
          inputLoop = a.newLabel();
          a.mov(a.zcx(), imm_u(l.inputDimensions[0] / 4));
          a.bind(inputLoop);
        }

        compileInputBatch(a, remainingOutputs, stepSize, 4, last);

        // End loop over input
        if(l.inputDimensions[0] / 4 >= 2)
        {
          a.dec(a.zcx());
          a.jnz(inputLoop);
        }
      }

      const unsigned int remainingInputs = l.inputDimensions[0] == 4 ? 4 : (l.inputDimensions[0] % 4);
      if(remainingInputs)
        compileInputBatch(a, remainingOutputs, stepSize, remainingInputs, last, true);

      // Add biases
      for(unsigned int step = 0; step < stepSize; step++)
        a.addps(x86::xmm(step), a.ptr_zbx(step * 4 * sizeof(float)));

      if(l.activationId != ActivationFunctionId::linear)
      {
        // Apply activation function
        ActivationFn& activationFunction = afHandler.prepare(l.activationId, false, a, { x86::xmm(settings.xmmRegs() - 2), x86::xmm(settings.xmmRegs() - 1) }, {});
        for(unsigned int step = 0; step < stepSize; step++)
          activationFunction.addValue(x86::xmm(step));
        for(unsigned int step = stepSize; step < 6; step++)
          activationFunction.addSpare(x86::xmm(step));
        activationFunction.initialize(a);
        activationFunction.apply(a);

        // In this case, Batch Normalization was not done implicitly, so we have to do it manually
        if(batchNormalization)
        {
          // Multiply with factors
          a.add(a.zbx(), a.ptr_zbp(-8, 4));
          for(unsigned int step = 0; step < stepSize; step++)
            a.mulps(x86::xmm(step), a.ptr_zbx(step * 4 * sizeof(float)));

          // Add offsets
          a.add(a.zbx(), a.ptr_zbp(-4, 4));
          for(unsigned int step = 0; step < stepSize; step++)
            a.addps(x86::xmm(step), a.ptr_zbx(step * 4 * sizeof(float)));

          // Reset bias pointer
          a.sub(a.zbx(), a.ptr_zbp(-4, 4));
          a.sub(a.zbx(), a.ptr_zbp(-8, 4));
        }
      }
      if(postActivation != ActivationFunctionId::linear)
      {
        // Apply activation function
        ActivationFn& activationFunction = afHandler.prepare(postActivation, false, a, { x86::xmm(settings.xmmRegs() - 2), x86::xmm(settings.xmmRegs() - 1) }, {});
        for(unsigned int step = 0; step < stepSize; step++)
          activationFunction.addValue(x86::xmm(step));
        for(unsigned int step = stepSize; step < 6; step++)
          activationFunction.addSpare(x86::xmm(step));
        activationFunction.initialize(a);
        activationFunction.apply(a);
      }

      // Advance bias pointer
      if(!last)
        a.add(a.zbx(), imm_u(stepSize * 4 * sizeof(float)));

      // Store results
      for(unsigned int step = 0; step < stepSize; step++)
        a.movaps(a.ptr_zdi(step * 4 * sizeof(float)), x86::xmm(step));

      // Advance destination pointer if necessary
      if(!last && (l.outputDimensions[0] / (4 * 6) >= 2 || l.outputDimensions[0] % (4 * 6) != 0))
        a.add(a.zdi(), imm_u(stepSize * 4 * sizeof(float)));
    }

    void DenseLayerCompiler::compileSimple(X86Assembler& a, ActivationFunctionHandler& afHandler, const float* const input, const float* const output) const
    {
      // Declare labels
      const NetworkConstants& weights = constants[0];
      const NetworkConstants& biases = constants[1];

      a.mov(a.zsi(), imm_ptr<>(input));

      bool destInZSI = true;
      bool denseDone = false;

      if(l.inputDimensions[0] == 1)
      {
        // Only one input
        a.movss(x86::xmm0, a.ptr_zsi());
        a.mulss(x86::xmm0, x86::ptr(weights.label));
        denseDone = true;
      }
      else if(l.inputDimensions[0] <= 4)
      {
        // Input fits into one XMM register
        a.movaps(x86::xmm0, a.ptr_zsi());
        if(settings.useSSE42)
        {
          unsigned char mask = 1;
          for(unsigned int i = 0; i < l.inputDimensions[0]; i++)
            mask |= (1 << (4 + i));
          a.dpps(x86::xmm0, x86::ptr(weights.label), imm(mask));
        }
        else
        {
          a.mulps(x86::xmm0, x86::ptr(weights.label));
          if(l.inputDimensions[0] < 4)
            a.pslldq(x86::xmm0, imm((4 - l.inputDimensions[0]) * sizeof(float)));
          a.haddps(x86::xmm0, x86::xmm0);
          if(l.inputDimensions[0] > 2)
            a.haddps(x86::xmm0, x86::xmm0);
          else
            a.psrldq(x86::xmm0, imm((4 - l.inputDimensions[0]) * sizeof(float)));
        }
        denseDone = true;
      }
      else
      {
        // Input dimension is divisible by a multiple of 4 up to 32 (or 64 on x64)
        // (input fits completely into the XMM registers)
        for(unsigned int stepSize = settings.xmmRegs(); stepSize > 1; --stepSize)
        {
          if(l.inputDimensions[0] == stepSize * 4)
          {
            // Load weights address
            a.lea(a.zdx(), x86::ptr(weights.label));

            // Load channels
            for(unsigned int i = 0; i < stepSize; ++i)
              a.movaps(x86::xmm(i), a.ptr_zsi(i * 4 * sizeof(float)));

            if(settings.useSSE42)
            {
              // Multiply channels with weights
              for(unsigned int i = 0; i < stepSize; ++i)
                a.dpps(x86::xmm(i), a.ptr_zdx(i * 4 * sizeof(float)), imm(0xF1));

              // Accumulate results
              bool odd = stepSize % 2 != 0;
              for(unsigned int stride = stepSize / 2; stride; stride /= 2)
              {
                for(unsigned int i = 0; i < stride; i++)
                  a.addss(x86::xmm(i), x86::xmm(i + stride));
                if(odd)
                  a.addss(x86::xmm0, x86::xmm(stride * 2 - 1));
                odd = stride % 2 != 0;
              }
            }
            else
            {
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
                  a.addps(x86::xmm0, x86::xmm(stride * 2 - 1));
                odd = stride % 2 != 0;
              }
              a.haddps(x86::xmm0, x86::xmm0);
              a.haddps(x86::xmm0, x86::xmm0);
            }

            denseDone = true;
            break;
          }
        }
      }
      if(!denseDone)   // Default case
      {
        // Copy dest pointer to zdi
        a.mov(a.zdi(), a.zsi());
        destInZSI = false;

        // Load weights address
        a.lea(a.zdx(), x86::ptr(weights.label));

        // Initialise result
        a.xorps(x86::xmm0, x86::xmm0);

        unsigned int remainingChannels = l.inputDimensions[0];
        for(unsigned int stepSize = settings.xmmRegs() - 1; stepSize; --stepSize)
        {
          if(remainingChannels >= stepSize * 4)
          {
            // Begin loop if the stepsize can be applied multiple times
            Label loop;
            if(remainingChannels >= stepSize * 8)
            {
              a.mov(a.zcx(), imm_u(remainingChannels / (stepSize * 4)));
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
              a.add(a.zsi(), imm_u(stepSize * 4 * sizeof(float)));
              a.add(a.zdx(), imm_u(stepSize * 4 * sizeof(float)));
            }

            // End loop if the stepsize can be applied multiple times
            if(remainingChannels >= stepSize * 8)
            {
              if(stepSize <= 7)
                a.loop(loop);
              else
              {
                a.dec(a.zcx());
                a.jne(loop);
              }
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
          a.movaps(x86::xmm1, a.ptr_zsi());
          a.mulps(x86::xmm1, a.ptr_zdx());

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

      if(l.activationId != ActivationFunctionId::linear)
      {
        // Apply activation function
        ActivationFn& activationFunction = afHandler.prepare(l.activationId, true, a, { }, { x86::xmm0 });
        for(unsigned int i = 1; i < settings.xmmRegs(); i++)
          activationFunction.addSpare(x86::xmm(i));
        activationFunction.initialize(a);
        activationFunction.apply(a);

        if(batchNormalization)
        {
          // In this case, Batch Normalization was not done implicitly, so we have to do it manually
          ASSERT(constants.size() == 4);

          // Multiply with factor
          a.mulss(x86::xmm0, x86::ptr(constants[2].label));

          // Add offset
          a.addss(x86::xmm0, x86::ptr(constants[3].label));
        }
      }
      if(postActivation != ActivationFunctionId::linear)
      {
        // Apply activation function
        ActivationFn& activationFunction = afHandler.prepare(postActivation, true, a, { }, { x86::xmm0 });
        for(unsigned int i = 1; i < settings.xmmRegs(); i++)
          activationFunction.addSpare(x86::xmm(i));
        activationFunction.initialize(a);
        activationFunction.apply(a);
      }

      // Store result
      a.movss(destInZSI ? a.ptr_zsi() : a.ptr_zdi(), x86::xmm0);
    }

    void DenseLayerCompiler::compile(X86Assembler& a, ActivationFunctionHandler& afHandler, const float* const input, const float* const output) const
    {
      // Handle the special case of only one output
      if(l.outputDimensions[0] == 1)
      {
        compileSimple(a, afHandler, input, output);
        return;
      }

      // Declare labels
      const NetworkConstants& weights = constants[0];
      const NetworkConstants& biases = constants[1];

      // Load offsets
      a.lea(a.zdx(), x86::ptr(weights.label));
      a.mov(a.zdi(), imm_ptr<>(output));
      a.lea(a.zbx(), x86::ptr(biases.label));

      if(l.activationId != ActivationFunctionId::linear && batchNormalization)
      {
        // Store coefficient offsets
        ASSERT(constants.size() == 4);
        a.lea(a.zax(), x86::ptr(constants[2].label));
        a.lea(a.zcx(), x86::ptr(constants[3].label));
        a.sub(a.zcx(), a.zax());
        a.sub(a.zax(), a.zbx());
        a.mov(a.ptr_zbp(-8, 4), x86::eax);
        a.mov(a.ptr_zbp(-4, 4), x86::ecx);
      }

      if(l.outputDimensions[0] > 4 * 6)
      {
        // Begin loop over output batches (only construct loop if it has more than one iteration)
        Label outputBatchLoop;
        if(l.outputDimensions[0] / (4 * 6) >= 2)
        {
          outputBatchLoop = a.newLabel();
          a.mov(a.zax(), imm_u(l.outputDimensions[0] / (4 * 6)));
          a.bind(outputBatchLoop);
        }

        compileOutputBatch(a, afHandler, input, 4 * 6);

        // End loop over output batches
        if(l.outputDimensions[0] / (4 * 6) >= 2)
        {
          a.dec(a.zax());
          a.jnz(outputBatchLoop);
        }
      }

      const unsigned int remainingOutputs = l.outputDimensions[0] == (4 * 6) ? (4 * 6) : (l.outputDimensions[0] % (4 * 6));
      if(remainingOutputs)
        compileOutputBatch(a, afHandler, input, remainingOutputs, true);
    }
  }
}
