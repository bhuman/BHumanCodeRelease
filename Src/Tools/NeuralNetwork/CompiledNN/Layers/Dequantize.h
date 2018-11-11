/**
 * @author Felix Thielke
 */

#pragma once

#include "../CompiledNNImplBase.h"
#include "Tools/NeuralNetwork/CompiledNN/Util/Quantization.h"

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    struct DequantizeLayerCompiler : public LayerCompiler
    {
      const std::array<unsigned int, 3> dimensions;
      const Quantization::QuantizationParams& quantizationParams;

      DequantizeLayerCompiler(const CompilationSettings& settings, const std::array<unsigned int, 3>& dimensions, const Quantization::QuantizationParams& quantizationParams) : LayerCompiler(settings), dimensions(dimensions), quantizationParams(quantizationParams) {}

      inline bool isInplace() const override { return false; }

      void initialize() override;
      void compile(X86Assembler& a, ActivationFunctionHandler& afHandler, const float* const input, const float* const output) const override;
    };
  }
}
