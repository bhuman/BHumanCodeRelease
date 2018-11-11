/**
 * Declares a class for storing a n-dimensional tensor.
 *
 * @author Felix Thielke
 */

#pragma once

#include "Platform/BHAssert.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Math/NeumaierSum.h"
#include <array>
#include <vector>
#include <algorithm>
#include <type_traits>

namespace NeuralNetwork
{
  /**
   * A class for storing a n-dimensional tensor.
   */
  template<unsigned int rank, typename T = float, size_t alignment = 16>
  class Tensor
  {
    static_assert(rank >= 1, "");
    static_assert(alignment == 1 || alignment == 2 || alignment == 4 || alignment == 8 || alignment == 16 || alignment == 32 || alignment == 64, "Alignment must be 2^n, n in [0..6].");

  private:
    std::array<unsigned int, rank> dimensions;
    std::vector<unsigned char> buffer;
    T* dataOffset;
    static constexpr size_t alignmentShift = std::max(sizeof(T), alignment) - 1;

    inline constexpr unsigned int computeIndex(const std::array<unsigned int, rank>& indices) const
    {
      unsigned int index = indices[0];

      for(unsigned int i = 1; i < rank; i++)
        index = index * dimensions[i] + indices[i];

      return index;
    }

  public:
    Tensor() = default;
    Tensor(const std::array<unsigned int, rank>& dimensions, const size_t minCapacity = 0)
    {
      reshape(dimensions, minCapacity);
    }
    Tensor(const Tensor<rank>& other) : Tensor(other.dimensions)
    {
      std::copy_n(other.data(), other.size(), data());
    }

    inline Tensor& operator=(const Tensor<rank>& other)
    {
      reshape(other.dimensions);
      std::copy_n(other.data(), other.size(), data());
      return *this;
    }

    inline Tensor& copyFrom(const Tensor& other)
    {
      const size_t size = this->size();
      ASSERT(size == other.size());
      std::copy_n(other.data(), size, data());
      return *this;
    }

    inline void reshape(const std::array<unsigned int, rank>& dimensions, const size_t minCapacity = 0)
    {
      this->dimensions = dimensions;
      const size_t size = std::max(minCapacity, this->size()) * sizeof(T) + alignmentShift;
      if(buffer.size() < size)
      {
        buffer.resize(size);
        dataOffset = reinterpret_cast<T*>((reinterpret_cast<uintptr_t>(buffer.data()) + alignmentShift) & (~alignmentShift));
      }
    }
    template<typename... Indices> inline void reshape(const Indices... indices) { reshape({ { static_cast<unsigned int>(indices)... } }); }

    inline void reshapeDim(const size_t dim, const unsigned int size)
    {
      if(dim >= rank)
        return;

      const unsigned int oldSize = dimensions[dim];
      dimensions[dim] = size;
      if(size > oldSize)
        reserve(this->size());
    }

    inline void reserve(const size_t minCapacity)
    {
      const size_t size = minCapacity * sizeof(T) + alignmentShift;
      if(buffer.size() < size)
      {
        buffer.resize(size);
        dataOffset = reinterpret_cast<T*>((reinterpret_cast<uintptr_t>(buffer.data()) + alignmentShift) & (~alignmentShift));
      }
    }

    inline T absError(const Tensor<rank, T>& other, const bool l2)
    {
      ASSERT(dimensions == other.dimensions);

      using SumType = typename std::conditional<std::is_floating_point<T>::value, NeumaierSum<T>, T>::type;
      SumType sum;

      const T* p0 = begin();
      const T* p1 = other.begin();
      if(l2)
      {
        for(size_t size = this->size(); size; --size)
          sum += sqr(*(p0++) - *(p1++));
      }
      else
      {
        for(size_t size = this->size(); size; --size)
          sum += std::abs(*(p0++) - *(p1++));
      }

      return l2 ? std::sqrt(static_cast<T>(sum)) : static_cast<T>(sum);
    }

    inline T relError(const Tensor<rank, T>& other, const bool l2)
    {
      ASSERT(dimensions == other.dimensions);

      using SumType = typename std::conditional<std::is_floating_point<T>::value, NeumaierSum<T>, T>::type;
      SumType sum;

      const T* p0 = begin();
      const T* p1 = other.begin();
      if(l2)
      {
        for(size_t size = this->size(); size; --size)
        {
          const T val0 = *(p0++);
          const T val1 = *(p1++);
          if(val0 != T(0) && val1 != T(0))
            sum += sqr(T(1) - val1 / val0);
        }
      }
      else
      {
        for(size_t size = this->size(); size; --size)
        {
          const T val0 = *(p0++);
          const T val1 = *(p1++);
          if(val0 != T(0) && val1 != T(0))
            sum += std::abs(T(1) - val1 / val0);
        }
      }

      return l2 ? std::sqrt(static_cast<T>(sum)) : static_cast<T>(sum);
    }

    inline T sad(const Tensor<rank, T>& other) const
    {
      return absError(other, false);
    }

    inline const T* data() const { return dataOffset; }
    inline T* data() { return dataOffset; }

    inline const T* begin() const { return data(); }
    inline T* begin() { return data(); }
    inline const T* end() const { return data() + size(); }
    inline T* end() { return data() + size(); }

    inline const std::array<unsigned int, rank>& dims() const { return dimensions; }
    inline unsigned int dims(const size_t i) const { return dimensions[i]; }

    inline constexpr size_t size() const
    {
      auto it = dimensions.cbegin();
      size_t size = *it;
      for(it++; it != dimensions.cend(); it++)
        size *= *it;
      return size;
    }

    inline const T& operator[](const size_t index) const { return *(dataOffset + index); }
    inline T& operator[](const size_t index) { return *(dataOffset + index); }

    inline const T& operator()(const std::array<unsigned int, rank>& indices) const { return (*this)[computeIndex(indices)]; }
    inline T& operator()(const std::array<unsigned int, rank>& indices) { return (*this)[computeIndex(indices)]; }

    template<typename... Indices> inline const T& operator()(const Indices... indices) const { return (*this)({ {static_cast<unsigned int>(indices)...} }); }
    template<typename... Indices> inline T& operator()(const Indices... indices) { return (*this)({ {static_cast<unsigned int>(indices)...} }); }
  };

  using Tensor2 = Tensor<2>;
  using Tensor3 = Tensor<3>;
  using Tensor4 = Tensor<4>;
}
