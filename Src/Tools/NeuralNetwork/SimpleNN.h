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
#include <functional>
#include <string>
#include <vector>

namespace NeuralNetwork
{
  namespace SimpleNN
  {
    using NodeCallback = std::function<void(const Node&, const std::vector<const TensorXf*>&, const std::vector<TensorXf*>&)>;

    /**
     * Applies the net described by the given specification on the given tensors
     * and returns the result. The input tensors may be modified in the process.
     * Optionally, a callback is called after execution of each node.
     */
    void apply(std::vector<TensorXf>& input, std::vector<TensorXf>& output, const Model& specification, const NodeCallback& nodeCallback = NodeCallback());

    /**
     * Applies a net consisting of only the given node on the given tensors and
     * returns the result.
     */
    void apply(const std::vector<TensorXf>& input, std::vector<TensorXf>& output, const Node& node);

    /**
     * Applies the net from the given file on the given tensors and returns the
     * result. The input tensors may be modified in the process.
     */
    inline void apply(std::vector<TensorXf>& input, std::vector<TensorXf>& output, const std::string& filename) { return apply(input, output, Model(filename)); }
  };
}
