/**
 * Contains functions to apply neural networks on input data without
 * optimizations. Do not use this in production; these are only meant to
 * verify the results of optimized implementations.
 *
 * @author Felix Thielke
 */

#pragma once

#include "Tensor.h"
#include "Model.h"
#include <string>

namespace NeuralNetwork
{
  namespace SimpleNN
  {
    /**
     * Applies the net described by the given specification on the given tensor
     * and returns the result. The input tensor may be modified in the process.
     */
    void apply(Tensor3& input, Tensor3& output, const Model& specification);

    /**
     * Applies a net consisting of only the given layer on the given tensor and
     * returns the result.
     */
    void apply(const Tensor3& input, Tensor3& output, const Layer& layer);

    /**
     * Applies the net from the given file on the given tensor and returns the
     * result. The input tensor may be modified in the process.
     */
    inline void apply(Tensor3& input, Tensor3& output, const std::string& filename) { return apply(input, output, Model(filename)); }
  };
}
