/**
 * Utility functions for quantizing floats to integers.
 * (cf. https://github.com/google/gemmlowp/blob/master/doc/quantization.md)
 *
 * @author Felix Thielke
 */

#pragma once

#include "Tools/Range.h"

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    namespace Quantization
    {
      /**
       * Struct holding the parameters of a quantization:
       * floatValue = scale * (quantizedValue - zeroPoint)
       */
      struct QuantizationParams
      {
        float scale;             /**< scale of the quantization; always positive and in most cases <= 1 */
        unsigned char zeroPoint;  /**< the quantized value that stands for zero */

        QuantizationParams() = default;
        QuantizationParams(const float scale, unsigned char zeroPoint) : scale(scale), zeroPoint(zeroPoint) {}
      };

      /**
       * Chooses "optimal" quantization parameters for the given range of float values.
       *
       * @param range range of the input values. Will be modified to include zero.
       * @param resolution number of bits to use for the quantized representation.
       */
      QuantizationParams chooseQuantizationParams(Rangef& range, unsigned char resolution);
    }
  }
}
