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

    // Quantization
    bool useQuantization = false;              /**< quantize floats to fixed-point integers for calculations */
    unsigned char quantizationResolution = 8;  /**< number of bits to use for the quantized representation -- can be up to 8 */
    Rangef inputRange = Rangef(0.f, 255.f);    /**< range of values the input to the network can take, NEEDED BY QUANTIZATION TO WORK CORRECTLY */

    // Input
    bool uint8Input = false;  /**< interpret the values in the input tensor as unsigned chars instead of floats */

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
