/**
 * @author Felix Thielke
 */

#pragma once

#include "../CompiledNNImplBase.h"

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    struct ZeroPadding2DCompiler : public SISOOperationCompiler
    {
      struct Parameters
      {
        std::array<unsigned int, 4> padding;
      };
      const Parameters p;

      ZeroPadding2DCompiler(const CompilationSettings& settings, const Parameters& p) : SISOOperationCompiler(settings), p(p) {}

      inline bool canBeInplace() const override { return false; }
      void initialize() override {}
      void compile(x86::Assembler& a, ActivationFunctionHandler& afHandler, const TensorPointerXf& input, const TensorPointerXf& output) const override;

      inline std::vector<unsigned int> calcOutputDimensions(const std::vector<unsigned int>& inputDimensions) const override
      {
        ASSERT(inputDimensions.size() == 3);
        return {{
            inputDimensions[0] + p.padding[ZeroPadding2DLayer::TOP] + p.padding[ZeroPadding2DLayer::BOTTOM],
            inputDimensions[1] + p.padding[ZeroPadding2DLayer::LEFT] + p.padding[ZeroPadding2DLayer::RIGHT],
            inputDimensions[2]
          }};
      }
    };
  }
}
