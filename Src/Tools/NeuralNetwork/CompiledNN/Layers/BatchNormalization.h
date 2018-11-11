/**
 * @author Felix Thielke
 */

#pragma once

#include "../CompiledNNImplBase.h"

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    struct BatchNormalizationLayerCompiler : public InPlaceLayerCompiler
    {
      const BatchNormalizationLayer& l;
      unsigned int paramLength;

      BatchNormalizationLayerCompiler(const CompilationSettings& settings, const BatchNormalizationLayer& l) : InPlaceLayerCompiler(settings), l(l) {}

      void initialize() override;
      void compile(X86Assembler& a, ActivationFunctionHandler& afHandler, const float* const data) const override;
    };
  }
}
