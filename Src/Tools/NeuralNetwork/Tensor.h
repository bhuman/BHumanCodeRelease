/**
 * Declares a class for storing a n-dimensional tensor.
 *
 * @author Felix Thielke
 */

#pragma once

#include "Platform/BHAssert.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Math/NeumaierSum.h"
#include <algorithm>
#include <functional>
#include <numeric>
#include <type_traits>
#include <vector>

namespace NeuralNetwork
{
  /**
   * A class for storing a n-dimensional tensor.
   */
  template<typename T = float, std::size_t alignment = 16>
  class Tensor
  {
    static_assert(alignment == 1 || alignment == 2 || alignment == 4 || alignment == 8 || alignment == 16 || alignment == 32 || alignment == 64, "Alignment must be 2^n, n in [0..6].");

  private:
    std::vector<unsigned int> dimensions;
    std::vector<unsigned char> buffer;
    T* dataOffset;
    static constexpr std::size_t alignmentShift = std::max(sizeof(T), alignment) - 1;

    inline constexpr unsigned int computeIndex(const std::vector<unsigned int>& indices) const
    {
      ASSERT(indices.size() == dimensions.size());

      unsigned int index = indices[0];

      for(unsigned int i = 1; i < indices.size(); i++)
        index = index * dimensions[i] + indices[i];

      return index;
    }

  public:
    Tensor() = default;
    Tensor(const std::vector<unsigned int>& dimensions, const std::size_t minCapacity = 0)
    {
      reshape(dimensions, minCapacity);
    }
    Tensor(const Tensor& other) : Tensor(other.dimensions)
    {
      std::copy_n(other.data(), other.size(), data());
    }

    inline Tensor& operator=(const Tensor& other)
    {
      reshape(other.dimensions);
      std::copy_n(other.data(), other.size(), data());
      return *this;
    }

    inline Tensor& copyFrom(const Tensor& other)
    {
      const std::size_t size = this->size();
      ASSERT(size == other.size());
      std::copy_n(other.data(), size, data());
      return *this;
    }

    inline void reshape(const std::vector<unsigned int>& dimensions, const std::size_t minCapacity = 0)
    {
      ASSERT(!dimensions.empty());
      this->dimensions = dimensions;
      const std::size_t size = std::max(minCapacity, this->size()) * sizeof(T) + alignmentShift;
      if(buffer.size() < size)
      {
        buffer.resize(size);
        dataOffset = reinterpret_cast<T*>((reinterpret_cast<uintptr_t>(buffer.data()) + alignmentShift) & (~alignmentShift));
      }
    }
    template<typename... Indices> inline void reshape(const Indices... indices) { reshape({ { static_cast<unsigned int>(indices)... } }); }

    inline void reshapeDim(const std::size_t dim, const unsigned int size)
    {
      if(dim >= dimensions.size())
        return;

      const unsigned int oldSize = dimensions[dim];
      dimensions[dim] = size;
      if(size > oldSize)
        reserve(this->size());
    }

    inline void reserve(const std::size_t minCapacity)
    {
      const std::size_t size = minCapacity * sizeof(T) + alignmentShift;
      if(buffer.size() < size)
      {
        buffer.resize(size);
        dataOffset = reinterpret_cast<T*>((reinterpret_cast<uintptr_t>(buffer.data()) + alignmentShift) & (~alignmentShift));
      }
    }

    inline T absError(const Tensor<T>& other, const bool l2) const
    {
      ASSERT(dimensions == other.dimensions);

      using SumType = typename std::conditional<std::is_floating_point<T>::value, NeumaierSum<T>, T>::type;
      SumType sum;

      const T* p0 = begin();
      const T* p1 = other.begin();
      if(l2)
      {
        for(std::size_t size = this->size(); size; --size)
          sum += sqr(*(p0++) - *(p1++));
      }
      else
      {
        for(std::size_t size = this->size(); size; --size)
          sum += std::abs(*(p0++) - *(p1++));
      }

      return l2 ? std::sqrt(static_cast<T>(sum)) : static_cast<T>(sum);
    }

    inline T relError(const Tensor<T>& other, const bool l2) const
    {
      ASSERT(dimensions == other.dimensions);

      using SumType = typename std::conditional<std::is_floating_point<T>::value, NeumaierSum<T>, T>::type;
      SumType sum;

      const T* p0 = begin();
      const T* p1 = other.begin();
      if(l2)
      {
        for(std::size_t size = this->size(); size; --size)
        {
          const T val0 = *(p0++);
          const T val1 = *(p1++);
          if(val0 != T(0) && val1 != T(0))
            sum += sqr(T(1) - val1 / val0);
        }
      }
      else
      {
        for(std::size_t size = this->size(); size; --size)
        {
          const T val0 = *(p0++);
          const T val1 = *(p1++);
          if(val0 != T(0) && val1 != T(0))
            sum += std::abs(T(1) - val1 / val0);
        }
      }

      return l2 ? std::sqrt(static_cast<T>(sum)) : static_cast<T>(sum);
    }

    inline T sad(const Tensor<T>& other) const
    {
      return absError(other, false);
    }

    inline const T* data() const { return dataOffset; }
    inline T* data() { return dataOffset; }

    inline const T* begin() const { return data(); }
    inline T* begin() { return data(); }
    inline const T* end() const { return data() + size(); }
    inline T* end() { return data() + size(); }

    inline const std::vector<unsigned int>& dims() const { return dimensions; }
    inline unsigned int dims(const std::size_t i) const { return dimensions[i]; }

    inline std::size_t rank() const { return dimensions.size(); }

    inline constexpr std::size_t size() const
    {
      ASSERT(!dimensions.empty());
      return std::accumulate(std::next(dimensions.cbegin()), dimensions.cend(), dimensions.front(), std::multiplies<>());
    }

    inline std::size_t capacity() const
    {
      return (buffer.capacity() - alignmentShift) / sizeof(T);
    }

    inline const T& operator[](const std::size_t index) const { return *(dataOffset + index); }
    inline T& operator[](const std::size_t index) { return *(dataOffset + index); }

    inline const T& operator()(const std::vector<unsigned int>& indices) const { return (*this)[computeIndex(indices)]; }
    inline T& operator()(const std::vector<unsigned int>& indices) { return (*this)[computeIndex(indices)]; }

    template<typename... Indices> inline const T& operator()(const Indices... indices) const { return (*this)({ {static_cast<unsigned int>(indices)...} }); }
    template<typename... Indices> inline T& operator()(const Indices... indices) { return (*this)({ {static_cast<unsigned int>(indices)...} }); }
  };

  using TensorXf = Tensor<float>;
}
