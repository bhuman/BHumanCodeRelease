/**
 * @author Felix Thielke
 */

#pragma once

#include "../ActivationFunctions.h"
#include "../CompiledNNImplBase.h"
#include "BatchNormalization.h"

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    struct DenseCompiler : public SISOOperationCompiler
    {
      struct Parameters
      {
        // Order of operations:
        // preBatchNormalization -> Dense -> activationDesc -> postBatchNormalization -> postActivation

        const BatchNormalizationCompiler::Parameters* preBatchNormalization = nullptr;
        const BatchNormalizationCompiler::Parameters* postBatchNormalization = nullptr;
        const Tensor<float, 1>* weights;
        const std::vector<float>* biases;
        ActivationFunctionDescriptor activationDesc;
        ActivationFunctionDescriptor postActivation;
      };
      const Parameters p;

      DenseCompiler(const CompilationSettings& settings, const Parameters& p) : SISOOperationCompiler(settings), p(p) {}

      inline bool canBeInplace() const override
      {
        return p.weights->dims(1) == 1;
      }

      void initialize() override;
      void compile(x86::Assembler& a, ActivationFunctionHandler& afHandler, const TensorPointerXf& input, const TensorPointerXf& output) const override;

      inline std::vector<unsigned int> calcOutputDimensions(const std::vector<unsigned int>& inputDimensions) const override
      {
        return {p.weights->dims(1)};
      }

    private:
      unsigned int outputBatchSize = 0;

      void compileInputBatch(x86::Assembler& a, const unsigned int remainingOutputs, const unsigned int stepSize, const unsigned int remainingInputs, const bool lastOutputBatch, const bool lastInputBatch = false) const;
      void compileOutputBatch(x86::Assembler& a, ActivationFunctionHandler& afHandler, const float* const input, const unsigned int remainingOutputs, const bool last = false) const;
      void compileSimple(x86::Assembler& a, ActivationFunctionHandler& afHandler, const float* const input, const float* const output) const;
    };
  }
}
