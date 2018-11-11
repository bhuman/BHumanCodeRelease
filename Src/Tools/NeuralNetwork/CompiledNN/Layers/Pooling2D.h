/**
 * @author Felix Thielke
 */

#pragma once

#include "../CompiledNNImplBase.h"

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    struct Pooling2DLayerCompiler : public LayerCompiler
    {
      const Pooling2DLayer& l;

      Pooling2DLayerCompiler(const CompilationSettings& settings, const Pooling2DLayer& l) : LayerCompiler(settings), l(l) {}

      inline bool isInplace() const override
      {
        return l.strides[0] >= l.kernelSize[0] && l.strides[1] >= l.kernelSize[1];
      }

      void initialize() override;
      void compile(X86Assembler& a, ActivationFunctionHandler& afHandler, const float* const input, const float* const output) const override;

    private:
      mutable bool helperRegInitialized = false;

      void pool(X86Assembler& a, ActivationFunctionHandler& afHandler, const unsigned int paddingHorizontal, const unsigned int paddingVertical) const;
      unsigned int poolRow(X86Assembler& a, ActivationFunctionHandler& afHandler, const unsigned int paddingLeft, const unsigned int paddingVertical) const;
    };
  }
}
