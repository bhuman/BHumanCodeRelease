/**
 * @author Felix Thielke
 */

#pragma once

#include "../CompiledNNImplBase.h"

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    struct DenseLayerCompiler : public LayerCompiler
    {
      const DenseLayer& l;
      const BatchNormalizationLayer* batchNormalization = nullptr;
      ActivationFunctionId postActivation = ActivationFunctionId::linear;

      DenseLayerCompiler(const CompilationSettings& settings, const DenseLayer& l) : LayerCompiler(settings), l(l) {}

      inline bool isInplace() const override
      {
        return l.outputDimensions[0] == 1;
      }

      void initialize() override;
      void compile(X86Assembler& a, ActivationFunctionHandler& afHandler, const float* const input, const float* const output) const override;

    private:
      void compileInputBatch(X86Assembler& a, const unsigned int remainingOutputs, const unsigned int stepSize, const unsigned int remainingInputs, const bool lastOutputBatch, const bool lastInputBatch = false) const;
      void compileOutputBatch(X86Assembler& a, ActivationFunctionHandler& afHandler, const float* const input, const unsigned int remainingOutputs, const bool last = false) const;
      void compileSimple(X86Assembler& a, ActivationFunctionHandler& afHandler, const float* const input, const float* const output) const;
    };
  }
}
