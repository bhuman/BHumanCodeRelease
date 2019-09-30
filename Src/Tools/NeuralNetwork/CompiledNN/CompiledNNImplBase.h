/**
 * @author Felix Thielke
 */

#pragma once

#include "Tools/NeuralNetwork/Model.h"
#include "CompilationSettings.h"
#include "TensorPointer.h"
#include <asmjit/asmjit.h>
#include <vector>

#ifdef NDEBUG
#define CHECK_ASMJIT_ERROR static_cast<void>(0)
#else
#define CHECK_ASMJIT_ERROR \
  do { \
    const ErrorCode _errcode = static_cast<ErrorCode>(a.getLastError()); \
    ASSERT(_errcode == ErrorCode::kErrorOk); \
  } while(false)
#endif

namespace NeuralNetwork
{
  class CompiledNN;

  namespace CompiledNNImpl
  {
    using namespace asmjit;

    class ActivationFunctionHandler;

    struct NetworkConstants
    {
      Label label;
      std::vector<float> data;
    };

    struct OperationCompiler
    {
      const CompilationSettings& settings;
      std::vector<NetworkConstants> constants;

      OperationCompiler(const CompilationSettings& settings) : settings(settings) {}
      virtual ~OperationCompiler() = default;


      virtual void initialize() = 0;
      virtual void compile(x86::Assembler& a, ActivationFunctionHandler& afHandler, const std::vector<TensorPointerXf>& input, const std::vector<TensorPointerXf>& output) const = 0;
      virtual std::vector<std::vector<unsigned int>> calcOutputDimensions(const std::vector<std::vector<unsigned int>>& inputDimensions) const = 0;
      virtual std::vector<std::size_t> routeIO(const std::vector<std::size_t>& indices, const std::vector<std::vector<unsigned int>>& inputDimensions) const = 0;

    private:
      // This reference count only means how often its constants are used, i.e. refCount==0 means that the constants do not have to be declared.
      // refCount==0 does not mean that the compiler can be freed since there still may be references from other compilers to the parameters of this compiler.
      mutable std::size_t refCount = 1;
      friend class NeuralNetwork::CompiledNN;
    };

    struct SISOOperationCompiler : public OperationCompiler
    {
      SISOOperationCompiler(const CompilationSettings& settings) : OperationCompiler(settings) {}

      void compile(x86::Assembler& a, ActivationFunctionHandler& afHandler, const std::vector<TensorPointerXf>& input, const std::vector<TensorPointerXf>& output) const override
      {
        ASSERT(input.size() == 1);
        ASSERT(output.size() == 1);
        compile(a, afHandler, input[0], output[0]);
      }

      std::vector<std::vector<unsigned int>> calcOutputDimensions(const std::vector<std::vector<unsigned int>>& inputDimensions) const override
      {
        ASSERT(inputDimensions.size() == 1);
        return {calcOutputDimensions(inputDimensions[0])};
      }

      std::vector<std::size_t> routeIO(const std::vector<std::size_t>& indices, const std::vector<std::vector<unsigned int>>& inputDimensions) const override
      {
        if(!indices.empty() && canBeInplace())
          return {indices[0]};
        return {};
      }

      virtual void compile(x86::Assembler& a, ActivationFunctionHandler& afHandler, const TensorPointerXf& input, const TensorPointerXf& output) const = 0;
      virtual std::vector<unsigned int> calcOutputDimensions(const std::vector<unsigned int>& inputDimensions) const
      {
        return inputDimensions;
      }
      virtual bool canBeInplace() const = 0;
    };
  }
}
