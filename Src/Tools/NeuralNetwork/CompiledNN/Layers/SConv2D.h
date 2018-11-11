/**
 * @author Felix Thielke
 */

#pragma once

#include "../CompiledNNImplBase.h"

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    struct SConv2DLayerCompiler : public LayerCompiler
    {
      const SConv2DLayer& l;

      SConv2DLayerCompiler(const CompilationSettings& settings, const SConv2DLayer& l) : LayerCompiler(settings), l(l) {}

      inline bool isInplace() const override
      {
        return l.strides[0] >= l.weights.dims(0) && l.strides[1] >= l.weights.dims(1) && l.inputDimensions[2] >= l.outputDimensions[2];
      }

      void initialize() override;
      void compile(X86Assembler& a, ActivationFunctionHandler& afHandler, const float* const input, const float* const output) const override;
      size_t bufferSizeNeeded() const;

    private:
      mutable unsigned int biasOffset = 0;

      unsigned int padImage(X86Assembler& a, const float* const input) const;
      void compileFilter(X86Assembler& a, const bool inputAligned, const unsigned int remainingOutputs, const unsigned int remainingInput, const bool lastFilter = false) const;
      void compileOutputBatch(X86Assembler& a, const unsigned int inputWidth, const unsigned int remainingOutputs) const;
      void compileSimpleConvolution(X86Assembler& a, const unsigned int inputWidth) const;
    };
  }
}
