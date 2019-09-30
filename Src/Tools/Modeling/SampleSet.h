/**
 * @file SampleSet.h
 *
 * The file contains the definition of the class SampleSet and SampleSetProxy.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/Math/Pose2f.h"
#include "Platform/BHAssert.h"

/**
 * @class SelfLocatorSample
 * A sample class for the selflocator
 */
class SelfLocatorSample
{
public:
  Vector2f translation = Vector2f::Zero(); /**< The position in mm. */
  Vector2i rotation = Vector2i::Zero();    /**< Cosinus and sinus of the rotation multiplied by 1024. */
  float angle,                             /**< The rotation in radians. */
        weighting;                         /**< The weighting of a sample*/
  int cluster;                             /**< The number of the cluster this sample belongs to*/
  SelfLocatorSample* next;                 /**< The next robot sample when clustering samples using binning. */

  /** Default constructor*/
  SelfLocatorSample() : rotation(1024, 0), angle(0), weighting(1.0f), cluster(0), next(0) {}

  /**
   * Conversion to general pose
   * @return a pose
   */
  Pose2f toPose() const
  {
    return Pose2f(angle, translation);
  }
};

/**
 * @class SampleSetProxyBase
 * The base of class SampleSetProxy.
 */
class SampleSetProxyBase
{
protected:
  const char* data;
  const char* dataOld;
  int numberOfSamples,
      sizeOfEntries;

public:
  SampleSetProxyBase() {numberOfSamples = 0;}

  /**
   * The function returns the number of samples in the set.
   * @return The number of samples.
   */
  int size() const {return numberOfSamples;}

  /**
   * The function links the proxy to a sample set.
   */
  void link(const char* start, const char* startOld, int number, int size)
  {
    data = start;
    dataOld = startOld;
    numberOfSamples = number;
    sizeOfEntries = size;
  }
};

/**
 * @class SampleSetProxy
 * The SampleSetProxy can be linked to instances of the class SampleSet.
 */
template<typename T> class SampleSetProxy : public SampleSetProxyBase
{
public:
  /**
   * Constant access operator.
   * @param index The index of the sample to access.
   */
  const T& operator[](int index) const {return *reinterpret_cast<const T*>(data + index * sizeOfEntries);}

  /**
   * Constant access to the second set (old set after swapping).
   * @param index The index of the sample to access.
   */
  const T& getSampleFromOldSet(int index) const {return *reinterpret_cast<const T*>(dataOld + index * sizeOfEntries);}
};

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
   * The function links a proxy to this sample set.
   * @param sampleSetProxy The proxy.
   */
  void link(SampleSetProxyBase& sampleSetProxy) const
  {
    sampleSetProxy.link(static_cast<const char*>(current), static_cast<const char*>(other), num, sizeof(T));
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
