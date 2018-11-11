/**
 * @author Felix Thielke
 */

#pragma once

#include "Tools/NeuralNetwork/Model.h"
#include "CompilationSettings.h"
#include <asmjit/asmjit.h>
#include <vector>

#ifdef NDEBUG
#define CHECK_ASMJIT_ERROR ((void)0)
#else
#define CHECK_ASMJIT_ERROR \
  do { \
    const ErrorCode _errcode = static_cast<ErrorCode>(a.getLastError()); \
    ASSERT(_errcode == ErrorCode::kErrorOk); \
  } while(false)
#endif

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    using namespace asmjit;

    class ActivationFunctionHandler;

    struct NetworkConstants
    {
      Label label;
      std::vector<float> data;
    };

    struct LayerCompiler
    {
      const CompilationSettings& settings;
      std::vector<NetworkConstants> constants;

      LayerCompiler(const CompilationSettings& settings) : settings(settings) {}
      virtual ~LayerCompiler() = default;

      virtual bool isInplace() const = 0;

      virtual void initialize() = 0;
      virtual void compile(X86Assembler& a, ActivationFunctionHandler& afHandler, const float* const input, const float* const output = nullptr) const = 0;
    };

    struct InPlaceLayerCompiler : public LayerCompiler
    {
      InPlaceLayerCompiler(const CompilationSettings& settings) : LayerCompiler(settings) {}

      inline bool isInplace() const override { return true; }
      void compile(X86Assembler& a, ActivationFunctionHandler& afHandler, const float* const input, const float* const output) const override { compile(a, afHandler, input); }

      virtual void compile(X86Assembler& a, ActivationFunctionHandler& afHandler, const float* const data) const = 0;
    };
  }
}
