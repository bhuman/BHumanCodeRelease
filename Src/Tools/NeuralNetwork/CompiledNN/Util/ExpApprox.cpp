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
      template<bool single, typename FactorType, typename OffsetType>
      void apply(x86::Assembler& a, const std::vector<x86::Xmm>& values, const FactorType factor, const OffsetType offset)
      {
        for(const x86::Xmm& value : values)
        {
          if(single)
            a.mulss(value, factor);
          else
            a.mulps(value, factor);
        }
        for(const x86::Xmm& value : values)
          a.cvtps2dq(value, value);        // evil floating point bit level hacking
        for(const x86::Xmm& value : values)
          a.paddd(value, offset);          // what the fuck?
      }

      template void apply<false>(x86::Assembler& a, const std::vector<x86::Xmm>& values, const x86::Xmm factor, const x86::Xmm offset);
      template void apply<true>(x86::Assembler& a, const std::vector<x86::Xmm>& values, const x86::Xmm factor, const x86::Xmm offset);
      template void apply<false>(x86::Assembler& a, const std::vector<x86::Xmm>& values, const x86::Mem factor, const x86::Xmm offset);
      template void apply<true>(x86::Assembler& a, const std::vector<x86::Xmm>& values, const x86::Mem factor, const x86::Xmm offset);
      template void apply<false>(x86::Assembler& a, const std::vector<x86::Xmm>& values, const x86::Xmm factor, const x86::Mem offset);
      template void apply<true>(x86::Assembler& a, const std::vector<x86::Xmm>& values, const x86::Xmm factor, const x86::Mem offset);
      template void apply<false>(x86::Assembler& a, const std::vector<x86::Xmm>& values, const x86::Mem factor, const x86::Mem offset);
      template void apply<true>(x86::Assembler& a, const std::vector<x86::Xmm>& values, const x86::Mem factor, const x86::Mem offset);
    }
  }
}
