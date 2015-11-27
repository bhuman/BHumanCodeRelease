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
  STREAMABLE(Node,
  {
    Node() = default;
    Node(const std::string& option, int depth,
         const std::string& state, int optionTime,
         int stateTime, const std::vector<std::string>& parameters),

    (std::string) option,
    (int)(0) depth,
    (std::string) state,
    (int)(0) optionTime,
    (int)(0) stateTime,
    (std::vector<std::string>) parameters,
  });

  ActivationGraph()
  {
    graph.reserve(100);
  },

  (std::vector<Node>) graph,
});

inline ActivationGraph::Node::Node(const std::string& option, int depth,
                                   const std::string& state, int optionTime,
                                   int stateTime, const std::vector<std::string>& parameters) :
  option(option),
  depth(depth),
  state(state),
  optionTime(optionTime),
  stateTime(stateTime),
  parameters(parameters)
{}
