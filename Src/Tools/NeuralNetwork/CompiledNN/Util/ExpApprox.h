/**
 * Utility functions for approximating exp(x) by exploiting the floating point
 * format as shown by to Schraudolph.
 * (https://nic.schraudolph.org/bib2html/b2hd-Schraudolph99.html)
 *
 * For the neural network use cases, this method has a mean absolute error of
 * about 0.02.
 *
 * @author Felix Thielke
 */

#pragma once
#include <asmjit/asmjit.h>
#include <cmath>
#include <vector>

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    namespace ExpApprox
    {
      using namespace asmjit;

      inline float factor()
      {
        return float(1 << 23) / std::log(2.f);
      }

      inline int offset()
      {
        return (127 << 23) - static_cast<int>(std::round((1 << 23) * std::log(3.f / (8.f * std::log(2.f)) + 0.5f) / std::log(2.f) - 0.5f));
      }

      void apply(X86Assembler& a, const std::vector<X86Xmm>& values, const X86Xmm factor, const X86Xmm offset);
    }
  }
}
