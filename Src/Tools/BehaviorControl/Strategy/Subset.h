/**
 * @file Subset.h
 *
 * This file defines some functions to iterate through all
 * k-combinations of an N-element (multi)set.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include <vector>

class Subset
{
public:
  /**
   * Initializes an index vector to represent the lexicographically smallest set of k elements.
   * @param subset The vector to initialize (must already have the correct size k).
   */
  static void first(std::vector<std::size_t>& subset)
  {
    for(std::size_t i = 0; i < subset.size(); ++i)
      subset[i] = i;
  }

  /**
   * Creates the next subset index vector of a set with a given number of elements.
   * @param N The number of elements in the total set.
   * @param subset A set of indices in [0,N-1].
   * @return Whether the calculated subset is not the lexicographically smallest one.
   */
  static bool next(std::size_t N, std::vector<std::size_t>& subset)
  {
    for(std::size_t index = subset.size() - 1; index < subset.size(); --index)
    {
      if(subset[index] < N - (subset.size() - index))
      {
        ++subset[index];
        for(std::size_t i = index + 1; i < subset.size(); ++i)
          subset[i] = subset[index] + i - index;
        return true;
      }
    }
    first(subset);
    return false;
  }

  /**
   * Initializes an index vector to represent the lexicographically smallest set of k elements.
   * @param multiset A multiset of indices which must be sorted in ascending order.
   * @param subset The vector to initialize (must already have the correct size k).
   */
  static void first(const std::vector<std::size_t>& multiset, std::vector<std::size_t>& subset)
  {
    subset.assign(multiset.begin(), std::next(multiset.begin(), subset.size()));
  }

  /**
   * Creates the next subset index vector of a given index multiset.
   * @param multiset A multiset of indices which must be sorted in ascending order.
   * @param subset A set of indices which are in the multiset.
   * @return Whether the calculated subset is not the lexicographically smallest one.
   */
  static bool next(const std::vector<std::size_t>& multiset, std::vector<std::size_t>& subset)
  {
    for(std::size_t index = subset.size() - 1; index < subset.size(); --index)
    {
      if(subset[index] < multiset[multiset.size() - (subset.size() - index)])
      {
        std::size_t j;
        for(j = 0; multiset[j] <= subset[index]; ++j);
        for(std::size_t i = index; i < subset.size(); ++i, ++j)
          subset[i] = multiset[j];
        return true;
      }
    }
    first(multiset, subset);
    return false;
  }
};
