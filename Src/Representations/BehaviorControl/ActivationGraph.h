/**
 * @file ActivationGraph
 * The graph of executed options and states.
 *
 * @author Max Risler
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(ActivationGraph,
{
public:
  STREAMABLE(Node,
  {
  public:
    inline Node(const std::string& option,
                int depth,
                const std::string& state,
                int optionTime,
                int stateTime),

    (std::string) option,
    (int)(0) depth,
    (std::string) state,
    (int)(0) optionTime,
    (int)(0) stateTime,
  }),

  (std::vector<Node>) graph,

  // Initialization
  graph.reserve(100);
});

ActivationGraph::Node::Node(const std::string& option,
                            int depth,
                            const std::string& state,
                            int optionTime,
                            int stateTime)
: option(option),
  depth(depth),
  state(state),
  optionTime(optionTime),
  stateTime(stateTime) {}
