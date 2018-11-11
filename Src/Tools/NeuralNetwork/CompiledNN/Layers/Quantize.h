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
    struct QuantizeLayerCompiler : public InPlaceLayerCompiler
    {
      const std::array<unsigned int, 3> dimensions;
      const Quantization::QuantizationParams& quantizationParams;

      QuantizeLayerCompiler(const CompilationSettings& settings, const std::array<unsigned int, 3>& dimensions, const Quantization::QuantizationParams& quantizationParams) : InPlaceLayerCompiler(settings), dimensions(dimensions), quantizationParams(quantizationParams) {}

      void initialize() override;
      void compile(X86Assembler& a, ActivationFunctionHandler& afHandler, const float* const data) const override;
    };
  }
}
