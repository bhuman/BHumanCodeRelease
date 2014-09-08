/**
 * @file RingBufferWithSum.h
 *
 * Declaration of template class RingBufferWithSum
 *
 * @author Matthias Jüngel
 * @author Tobias Oberlies
 */

#pragma once

#include "RingBuffer.h"

/**
 * @class RingBufferWithSum
 *
 * Template class for cyclic buffering of the last n values of the type C
 * and with a function that returns the sum of all entries in constant time.
 */
template <class C, int n> class RingBufferWithSum : public RingBuffer<C, n>
{
public:
  /** Constructor */
  RingBufferWithSum() {init();}

  /**
   * initializes the RingBufferWithSum
   */
  void init()
  {
    RingBuffer<C, n>::init();
    sum = oldSum = C();
  }

  /**
   * adds an entry to the buffer
   * \param value value to be added
   */
  void add(const C& value)
  {
    if(RingBuffer<C, n>::getNumberOfEntries() == n)
      oldSum -= (*this)[RingBuffer<C, n>::getNumberOfEntries() - 1];

    sum += value;
    RingBuffer<C, n>::add(value);

    // Prevent propagating errors from one round to another
    if(RingBuffer<C, n>::current == 0)
    {
      oldSum = sum;
      sum = C();
    }
  }

  /**
   * returns the sum of all entries
   */
  C getSum() const
  {
    return oldSum + sum;
  }

  /**
   * returns the smallest entry
   * \return the smallest entry
   */
  C getMinimum() const
  {
    // Return 0 if buffer is empty
    if(RingBuffer<C, n>::getNumberOfEntries() == 0)
      return C();

    C min = (*this)[0];
    for(int i = 1; i < RingBuffer<C, n>::getNumberOfEntries(); ++i)
      if((*this)[i] < min)
        min = (*this)[i];

    return min;
  }

  /**
   * \return the biggest entry
   */
  C getMaximum() const
  {
    // Return 0 if buffer is empty
    if(RingBuffer<C, n>::getNumberOfEntries() == 0)
      return C();

    C max = (*this)[0];
    for(int i = 1; i < RingBuffer<C, n>::getNumberOfEntries(); ++i)
      if((*this)[i] > max)
        max = (*this)[i];

    return max;
  }
  
  /**
   * returns the average value of all entries
   * \return the average value
   */
  C getAverage() const
  {
    // Return 0 if buffer is empty
    if(RingBuffer<C, n>::getNumberOfEntries() == 0)
      return C();
    else
      return getSum() / RingBuffer<C, n>::getNumberOfEntries();
  }

  /**
   * returns the average value of all entries
   * \return the average value
   */
  C getAverageFloat() const
  {
    // Return 0 if buffer is empty
    if(RingBuffer<C, n>::getNumberOfEntries() == 0)
      return C();
    else
      return getSum() / static_cast<float>(RingBuffer<C, n>::getNumberOfEntries());
  }

private:
  C sum; /**< Sum of current round since index 0. */
  C oldSum; /** Sum of previous round. */
};
