/**
 * @author Felix Thielke
 */

#pragma once

#include "../CompiledNNImplBase.h"

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    struct UInt8InputCompiler : public LayerCompiler
    {
      const BatchNormalizationLayer* batchNormalization;
      const std::array<unsigned int, 3> dimensions;
      unsigned int paramLength;

      UInt8InputCompiler(const CompilationSettings& settings, const BatchNormalizationLayer* batchNormalization) : LayerCompiler(settings), batchNormalization(batchNormalization), dimensions(batchNormalization->inputDimensions) {}
      UInt8InputCompiler(const CompilationSettings& settings, const BatchNormalizationLayer* batchNormalization, const std::array<unsigned int, 3>& dimensions) : LayerCompiler(settings), batchNormalization(batchNormalization), dimensions(dimensions) {}

      inline bool isInplace() const override { return false; }

      void initialize() override;
      void compile(X86Assembler& a, ActivationFunctionHandler& afHandler, const float* const input, const float* const output) const override;
    };
  }
}
