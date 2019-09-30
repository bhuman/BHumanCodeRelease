/**
 * @author Felix Thielke
 */

#pragma once

#include "../CompiledNNImplBase.h"

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    struct Cropping2DCompiler : public SISOOperationCompiler
    {
      struct Parameters
      {
        std::array<unsigned int, 4> cropping;
      };
      const Parameters p;

      Cropping2DCompiler(const CompilationSettings& settings, const Parameters& p) : SISOOperationCompiler(settings), p(p) {}

      inline bool canBeInplace() const override { return true; }
      void initialize() override {}
      void compile(x86::Assembler& a, ActivationFunctionHandler& afHandler, const TensorPointerXf& input, const TensorPointerXf& output) const override;

      inline std::vector<unsigned int> calcOutputDimensions(const std::vector<unsigned int>& inputDimensions) const override
      {
        ASSERT(inputDimensions.size() == 3);
        return {{
            inputDimensions[0] - (p.cropping[Cropping2DLayer::TOP] + p.cropping[Cropping2DLayer::BOTTOM]),
            inputDimensions[1] - (p.cropping[Cropping2DLayer::LEFT] + p.cropping[Cropping2DLayer::RIGHT]),
            inputDimensions[2]
          }};
      }
    };
  }
}
