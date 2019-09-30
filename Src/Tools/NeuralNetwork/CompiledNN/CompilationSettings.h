/**
 * @author Felix Thielke
 */

#pragma once

#include "Tools/Range.h"

namespace NeuralNetwork
{
  struct CompilationSettings
  {
    // CPU features
    bool useX64 = true;    /**< use x64 features (additional XMM registers) */
    bool useSSE42 = true;  /**< use SSE features up to 4.2 as supported by NAO V6 (else SSSE3 is used as the max version) */
    bool useAVX2 = true;   /**< use AVX and AVX2 features (not supported by NAOs) */

    // Optimizations
    bool useExpApproxInSigmoid = true;  /**< use a less accurate but faster approximation of sigmoid */
    bool useExpApproxInTanh = true;     /**< use a less accurate but faster approximation of tanh */

    // Debugging
    bool debug = false; /**< activate breakpoints */

    /**
     * Returns the number of XMM regs available on the current processor.
     */
    constexpr unsigned int xmmRegs() const
    {
      return useX64 ? 16 : 8;
    }

    /**
     * Constricts the settings to valid values and CPU features supported by the
     * current processor.
     */
    void constrict();

    /**
     * Returns a copy of the settings that has the settings constricted to valid
     * values and CPU features supported by the current processor.
     */
    inline CompilationSettings constricted() const
    {
      CompilationSettings s(*this);
      s.constrict();
      return s;
    }
  };
}
