/**
 * @file RingBuffer.h
 *
 * Declaration of class RingBuffer
 *
 * @author Max Risler
 */

#pragma once

/**
 * @class RingBuffer
 *
 * template class for cyclic buffering of the last n values of Type V
 */
template <class V, int n> class RingBuffer
{
public:
  /** Constructor */
  RingBuffer() {init();}

  /**
   * initializes the Ringbuffer
   */
  void init() {current = n - 1; numberOfEntries = 0;}

  /**
   * adds an entry to the buffer
   * \param v value to be added
   */
  void add(const V& v)
  {
    add();
    buffer[current] = v;
  }

  /**
   * adds an entry to the buffer.
   * The new head is not initialized, but can be changed afterwards.
   */
  void add()
  {
    current++;
    current %= n;
    if(++numberOfEntries >= n) numberOfEntries = n;
  }

  /**
   * removes the first added entry to the buffer
   */
  void removeFirst()
  {
    --numberOfEntries;
  }

  /**
   * returns an entry
   * \param i index of entry counting from last added (last=0,...)
   * \return a reference to the buffer entry
   */
  V& getEntry(int i)
  {
    return buffer[(n + current - i) % n];
  }

  /**
   * returns an const entry
   * \param i index of entry counting from last added (last=0,...)
   * \return a reference to the buffer entry
   */
  const V& getEntry(int i) const
  {
    return buffer[(n + current - i) % n];
  }

  /**
   * returns an entry
   * \param i index of entry counting from last added (last=0,...)
   * \return a reference to the buffer entry
   */
  V& operator[](int i)
  {
    return buffer[(n + current - i) % n];
  }

  /**
   * returns a constant entry.
   * \param i index of entry counting from last added (last=0,...)
   * \return a reference to the buffer entry
   */
  const V& operator[](int i) const
  {
    return buffer[(n + current - i) % n];
  }

  /** Returns the number of elements that are currently in the ring buffer
  * \return The number
  */
  int getNumberOfEntries() const
  {
    return numberOfEntries;
  }

  /**
  * Returns the maximum entry count.
  * \return The maximum entry count.
  */
  int getMaxEntries() const
  {
    return n;
  }

  /**
  * Determines whether maximum entry count equals actual number of entries.
  * @return true iff getMaxEntries == getNumberOfEntries.
  */
  bool isFilled() const
  {
    return getMaxEntries() == getNumberOfEntries();
  }

  /**
   * Determines whether the buffer is empty.
   * \return True, if the number of entries is 0.
   */
  bool isEmpty() const
  {
    return !numberOfEntries;
  }

protected:
  int current;
  
private:
  int numberOfEntries;
  V buffer[n];
};
