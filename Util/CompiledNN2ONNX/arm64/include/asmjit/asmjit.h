/**
 * This is a wrapper for ONNX runtime to mimic the behavior of CompiledNN.
 * It is used because CompiledNN is currently not supported on ARM Macs.
 * CompiledNN is based on asmjit. Since the ONNX runtime does not require
 * asmjit, this file simply defines a dummy.
 * @author Thomas Röfer
 */
#pragma once

namespace asmjit
{
  inline namespace _abi_1_9
  {
    class JitRuntime
    {
    };
  }
}
