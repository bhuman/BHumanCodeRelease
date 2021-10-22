/**
 * @file SampleSet.h
 *
 * The file contains the definition of the class SampleSet.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Platform/BHAssert.h"

/**
 * @class SampleSet
 * A container for samples. Two independent sets are maintained.
 * As the sample set can be used by different modules that require
 * a different number of samples, the size of the set can be changed
 * at runtime.
 */
template<typename T> class SampleSet
{
private:
  int num;    /**< The number of samples. */
  T* current, /**< The actual sample set. */
   * other; /**< The secondary sample set. */

public:
  SampleSet(int num_)
  {
    ASSERT(num_ > 0);
    num = num_;
    current = new T[num];
    other = new T[num];
  }

  ~SampleSet()
  {
    delete[] current;
    delete[] other;
  }

  /**
   * The function returns the number of samples in the set.
   * @return The number of samples.
   */
  int size() const {return num;}

  /**
   * Access operator.
   * @param index The index of the sample to access.
   */
  T& at(int index) {ASSERT(index < num); return current[index];}

  /**
   * Constant access operator.
   * @param index The index of the sample to access.
   */
  const T& at(int index) const {ASSERT(index < num); return current[index];}

  /**
   * The function swaps the primary and secondary sample set.
   * @return The address of the previous sample set;
   */
  T* swap()
  {
    T* temp = current;
    current = other;
    other = temp;
    return other;
  }
};
