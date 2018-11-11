/**
 * @author Felix Thielke
 */

#pragma once

#include "../CompiledNNImplBase.h"

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    struct SoftmaxLayerCompiler : public InPlaceLayerCompiler
    {
      const SoftmaxLayer& l;

      SoftmaxLayerCompiler(const CompilationSettings& settings, const SoftmaxLayer& l) : InPlaceLayerCompiler(settings), l(l) {}

      void initialize() override;
      void compile(X86Assembler& a, ActivationFunctionHandler& afHandler, const float* const data) const override;
    };
  }
}
