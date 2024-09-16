/**
 * @file ActivationGraph.h
 *
 * The graph of executed options and states.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Platform/BHAssert.h"
#include "Streaming/AutoStreamable.h"

STREAMABLE(ActivationGraph,
{
  STREAMABLE(Node,
  {
    Node() = default;
    Node(const std::string& option, int depth,
         const std::string& state, int optionTime,
         int stateTime, const std::vector<std::string>& arguments),

    (std::string) option,
    (int)(0) depth,
    (std::string) state,
    (int)(0) optionTime,
    (int)(0) stateTime,
    (std::vector<std::string>) arguments,
  });

  ActivationGraph()
  {
    graph.reserve(100);
  },

  (std::vector<Node>) graph,
});

inline ActivationGraph::Node::Node(const std::string& option, int depth,
                                   const std::string& state, int optionTime,
                                   int stateTime, const std::vector<std::string>& arguments) :
  option(option),
  depth(depth),
  state(state),
  optionTime(optionTime),
  stateTime(stateTime),
  arguments(arguments)
{}
