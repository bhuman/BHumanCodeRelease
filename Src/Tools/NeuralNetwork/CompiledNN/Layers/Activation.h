/**
 * @author Felix Thielke
 */

#pragma once

#include "../CompiledNNImplBase.h"

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    struct ActivationLayerCompiler : public InPlaceLayerCompiler
    {
      const ActivationLayer& l;

      ActivationLayerCompiler(const CompilationSettings& settings, const ActivationLayer& l) : InPlaceLayerCompiler(settings), l(l) {}

      void initialize() override {}
      void compile(X86Assembler& a, ActivationFunctionHandler& afHandler, const float* const data) const override;
    };
  }
}
