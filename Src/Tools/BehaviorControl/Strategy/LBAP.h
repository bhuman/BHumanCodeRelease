/**
 * @file LBAP.h
 *
 * This file defines a function to solve a linear bottleneck assignment problem.
 *
 * @author Claude Sonnet 5
 */

#pragma once

#include "Math/Eigen.h"
#include "Platform/BHAssert.h"
#include <cstdint>
#include <set>
#include <vector>

class LBAP
{
private:
  /**
   * Finds an augmenting path in a graph from a given row.
   * @param row The row from which to find an augmenting path.
   * @param adj The adjacency list from each row to its neighboring columns.
   * @param matchCol The column to match.
   * @param visited Tracks for each column whether it was visited.
   * @return Whether an augmenting path could be found.
   */
  static bool tryAugment(const std::vector<std::vector<std::size_t>>& adj, std::size_t row,
                         std::vector<int>& matchCol, std::vector<bool>& visited)
  {
    for(std::size_t col : adj[row])
    {
      if(visited[col])
        continue;
      visited[col] = true;
      if(matchCol[col] == -1 || tryAugment(adj, static_cast<std::size_t>(matchCol[col]), matchCol, visited))
      {
        matchCol[col] = static_cast<int>(row);
        return true;
      }
    }
    return false;
  }

  /**
   * Calculates a perfect matching in the given bipartite graph (with same number of vertices in both partitions).
   * @param adj The adjacency list from each row (first partition) to its neighboring columns (second partition).
   * @return A vector that maps each row index to the matched column index.
   */
  static std::vector<int> perfectMatching(const std::vector<std::vector<std::size_t>>& adj)
  {
    // Mapping from column (index) to matched row (index).
    std::vector<int> matchCol(adj.size(), -1);
    for(std::size_t row = 0; row < adj.size(); ++row)
    {
      std::vector<bool> visited(adj.size(), false);
      if(!tryAugment(adj, row, matchCol, visited))
        return {};
    }
    // Convert to mapping from row (index) to matched column (index).
    std::vector<int> matchRow(adj.size(), -1);
    for(std::size_t col = 0; col < adj.size(); ++col)
    {
      ASSERT(matchCol[col] >= 0);
      matchRow[matchCol[col]] = static_cast<int>(col);
    }
    return matchRow;
  }

public:
  /**
   * Solves the LBAP for a given cost matrix. Each row in the cost matrix is matched with one column from \c candidateColumns.
   * @param costMatrix The cost matrix.
   * @param candidateColumns The column indices that may be used.
   * @param isAllowed A function that checks whether assigning a given row to a given column is allowed.
   * @return A vector with one entry per row with the assigned column in the entire matrix.
   */
  static std::vector<std::size_t> solve(const Eigen::MatrixXf& costMatrix,
                                        const std::vector<std::size_t>& candidateColumns,
                                        const std::function<bool(std::size_t row, std::size_t col)>& isAllowed)
  {
    const std::size_t n = candidateColumns.size();
    ASSERT(static_cast<std::size_t>(costMatrix.rows()) == n);

    // Construct initial adjacency list. All rows are adjacent to all columns, as long as isAllowed says so.
    // The algorithm will successively remove edges with descending cost.
    std::vector<std::vector<std::size_t>> adj(n);
    using Index = std::pair<size_t, size_t>;
    std::vector<Index> costIndices;
    for(std::size_t i = 0; i < n; ++i)
      for(std::size_t j = 0; j < n; ++j)
        if(isAllowed(i, candidateColumns[j]))
        {
          adj[i].push_back(j);
          costIndices.emplace_back(i, candidateColumns[j]);
        }

    // Sort indices by descending cost order.
    std::sort(costIndices.begin(), costIndices.end(), [&costMatrix](const Index& a, const Index& b)
    {
      return costMatrix(a.first, a.second) > costMatrix(b.first, b.second);
    });

    std::vector<int> bestMatching = perfectMatching(adj);
    if(bestMatching.empty())
      return {}; // infeasible even with all allowed edges

    // Cost indices are sorted in descending cost order.
    // If the same costs occur multiple times, it is possible that some of their assignments are
    // dropped while others are kept.
    // Unsolved: The sequence in which assignments with the same costs are processed might
    // influence the result, i.e. a sub-optimal solution might be found.
    for(const Index& threshold : costIndices)
    {
      std::vector<std::vector<std::size_t>> reduced(n);
      for(std::size_t i = 0; i < n; ++i)
        for(std::size_t j : adj[i])
          if(i != threshold.first || candidateColumns[j] != threshold.second)
            reduced[i].push_back(j);

      if(std::vector<int> reducedMatching = perfectMatching(reduced); !reducedMatching.empty())
      {
        // There is a matching that doesn't use this cost level -> drop it.
        adj = std::move(reduced);
        bestMatching = std::move(reducedMatching);
      }
      // else: At least one occurrence of 'threshold' is unavoidable -> keep it, move to next (lower) level.
    }

    // Convert matching back to assignment by mapping column indices.
    std::vector<std::size_t> assignment(n);
    for(std::size_t i = 0; i < n; ++i)
      assignment[i] = candidateColumns[static_cast<std::size_t>(bestMatching[i])];
    return assignment;
  }
};
