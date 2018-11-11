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

#include "ExpApprox.h"

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    namespace ExpApprox
    {
      void apply(X86Assembler& a, const std::vector<X86Xmm>& values, const X86Xmm factor, const X86Xmm offset)
      {
        for(const X86Xmm& value : values)
          a.mulps(value, factor);
        for(const X86Xmm& value : values)
          a.cvtps2dq(value, value);        // evil floating point bit level hacking
        for(const X86Xmm& value : values)
          a.paddd(value, offset);          // what the fuck?
      }
    }
  }
}
