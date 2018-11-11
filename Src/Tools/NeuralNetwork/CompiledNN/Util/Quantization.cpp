/**
 * Utility functions for quantizing floats to integers.
 * (cf. https://github.com/google/gemmlowp/blob/master/doc/quantization.md)
 *
 * @author Felix Thielke
 */

#include "Quantization.h"
#include <cmath>

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    namespace Quantization
    {
      QuantizationParams chooseQuantizationParams(Rangef& range, unsigned char resolution)
      {
        // Code shamelessly ~~stolen from~~ inspired by https://github.com/google/gemmlowp/blob/master/doc/quantization_example.cc

        // Extend range to include zero
        range.add(0.f);

        // Define the quantized range
        const Rangef qRange(0.f, static_cast<float>((1 << resolution) - 1));

        // Determine the scale
        const float scale = range.getSize() / qRange.getSize();

        // Compute the zero-point
        const float initial_zero_point = qRange.min - range.min / scale;

        // Nudge the zero point to be an integer within the quantized range and return result
        return QuantizationParams(
                 scale,
                 static_cast<unsigned char>(std::round(qRange.limit(initial_zero_point))) // nudge nudge
               );
      }
    }
  }
}
