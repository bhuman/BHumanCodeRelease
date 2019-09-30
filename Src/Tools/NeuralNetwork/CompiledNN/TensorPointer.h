/**
 * Declares a class for referencing a n-dimensional tensor.
 *
 * @author Felix Thielke
 * @author Arne Hasselbring
 */

#pragma once

#include "Platform/BHAssert.h"
#include "Tools/NeuralNetwork/Tensor.h"
#include <vector>

namespace NeuralNetwork
{
  /**
   * A class for referencing a n-dimensional tensor.
   * Actually only a container for a data pointer and its dimensions.
   */
  template<typename T = float>
  class TensorPointer
  {
  private:
    std::vector<unsigned int> dimensions;
    T* dataPointer = nullptr;

  public:
    TensorPointer() = default;

    template<size_t alignment>
    TensorPointer(Tensor<T, alignment>& other) :
      dimensions(other.dims()),
      dataPointer(other.data())
    {}

    inline const T* data() const { return dataPointer; }
    inline T* data() { return dataPointer; }

    inline const std::vector<unsigned int>& dims() const { return dimensions; }
    inline unsigned int dims(const std::size_t i) const { return dimensions[i]; }

    inline std::size_t rank() const { return dimensions.size(); }

    inline constexpr std::size_t size() const
    {
      auto it = dimensions.cbegin();
      size_t size = *it;
      for(it++; it != dimensions.cend(); it++)
        size *= *it;
      return size;
    }

  };

  using TensorPointerXf = TensorPointer<float>;
}
