/**
 * This is a wrapper for ONNX runtime to mimic the behavior of CompiledNN.
 * It is used because CompiledNN is currently not supported on ARM Macs.
 * This file replaces the CompiledNN header Model.h.
 * @author Thomas Röfer
 */

#pragma once

#include <string>
#include <unordered_set>

namespace NeuralNetworkONNX
{
  /**
   * The model. Technically, the class only stores the filename and which
   * inputs are encoded as unsigned chars instead of floats.
   */
  class Model
  {
    std::string filename; /**< The path to the .onnx file. */
    std::unordered_set<size_t> isUint8; /**< Which inputs are encoded as unsigned chars? */

  public:
    /**
     * Constructor.
     * @param filename The path to the .h5 file. There must be an .onnx
     *                 file with the same base name located in the same
     *                 directory.
     */
    Model(const std::string& filename)
    {
      size_t p = filename.find_last_of('.');
      this->filename = filename.substr(0, p) + ".onnx";
    }

    /**
     * Defines that a certain input is encoded as unsigned chars.
     * @param index The index of the input.
     */
    void setInputUInt8(size_t index) {isUint8.insert(index);}

    friend class CompiledNN; /**< CompiledNN needs access to private members. */
  };
}
