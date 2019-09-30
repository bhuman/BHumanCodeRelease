/**
 * @file Tools/ModuleGraphCreator/ComplexDataExchange.cpp
 *
 * This file implements tests for more complex data exchange.
 *
 * @author Jan Fiedler
 */

#include "Utils/Tests/ModuleGraphCreator/ModuleGraphCreatorTest.h"

#include <gtest/gtest.h>

/*
 * Require:
 * Ac: -> A
 * Bc: -> B
 * Cc: A -> B
 * Dc: -> C
 * Ec: A,B -> C
 * Am: C -> D
 * Bm: B -> A
 * At: aA -> C
 * Bt: aA,bA -> D
 * Ct: B -> aA
 */

TEST_P(ModuleGraphCreatorComplex, sharedTest)
{
  calcShared(GetParam().config);
  EXPECT_EQ(GetParam().received, received());
  EXPECT_EQ(GetParam().sent, sent());
  calcShared(GetParam().config2);
  EXPECT_EQ(GetParam().received2, received());
  EXPECT_EQ(GetParam().sent2, sent());
  ASSERT_DEATH(exit(-1), "^$");
}

// Simple tests with more than 2 threads.
INSTANTIATE_TEST_CASE_P(MultipleThreads, ModuleGraphCreatorComplex, testing::Values(
  Parameters // 0
  {
    createConfig({{{"A", "Ac"}}, {{"B", "Bc"}}, {{"C", "Dc"}}}),
    {{{}, {}, {}}, {{}, {}, {}}, {{}, {}, {}}}, {{{}, {}, {}}, {{}, {}, {}}, {{}, {}, {}}},
    createConfig({{{"A", "Ac"}}, {{"A", "Ac"}}, {{"A", "Ac"}}}),
    {{{}, {}, {}}, {{}, {}, {}}, {{}, {}, {}}}, {{{}, {}, {}}, {{}, {}, {}}, {{}, {}, {}}}
  },
  Parameters // 1
  {
    createConfig({{{"A", "Ac"}}, {{"A", "Ac"}}, {{"A", "Ac"}}, {{"A", "Ac"}}, {{"A", "Ac"}}}),
    {{{}, {}, {}, {}, {}}, {{}, {}, {}, {}, {}}, {{}, {}, {}, {}, {}}, {{}, {}, {}, {}, {}}, {{}, {}, {}, {}, {}}}, // received
    {{{}, {}, {}, {}, {}}, {{}, {}, {}, {}, {}}, {{}, {}, {}, {}, {}}, {{}, {}, {}, {}, {}}, {{}, {}, {}, {}, {}}}, // sent
    createConfig({{{"A", "Ac"}}, {{"B", "Bc"}}, {{"A", "Ac"}}, {{"A", "Ac"}}, {{"A", "Ac"}}}),
    {{{}, {}, {}, {}, {}}, {{}, {}, {}, {}, {}}, {{}, {}, {}, {}, {}}, {{}, {}, {}, {}, {}}, {{}, {}, {}, {}, {}}}, // received
    {{{}, {}, {}, {}, {}}, {{}, {}, {}, {}, {}}, {{}, {}, {}, {}, {}}, {{}, {}, {}, {}, {}}, {{}, {}, {}, {}, {}}}, // sent
  }
));

// Test communication between multiple threads.
INSTANTIATE_TEST_CASE_P(MultipleThreadsCommunication, ModuleGraphCreatorComplex, testing::Values(
  Parameters // 0
  {
    createConfig({{{"A", "Ac"}}, {{"B", "Bc"}}, {{"C", "Ec"}}}),
    {{{}, {}, {}}, {{}, {}, {}}, {{"A"}, {"B"}, {}}}, // received
    {{{}, {}, {"A"}}, {{}, {}, {"B"}}, {{}, {}, {}}}, // sent
    createConfig({{{"A", "Ac"}, {"D", "Am"}}, {{"B", "Bc"}}, {{"C", "Ec"}}}),
    {{{}, {}, {"C"}}, {{}, {}, {}}, {{"A"}, {"B"}, {}}}, // received
    {{{}, {}, {"A"}}, {{}, {}, {"B"}}, {{"C"}, {}, {}}} // sent
  }
));

// Require same representation from more than one thread.
INSTANTIATE_TEST_CASE_P(RepresentationMultipleTimes, ModuleGraphCreatorComplex, testing::Values(
  // 1->2
  Parameters // 0
  {
    createConfig({{{"A", "Bm"}}, {{"B", "Bc"}}, {{"C", "At"}}}),
    {{{}, {"B"}, {}}, {{}, {}, {}}, {{"aA"}, {}, {}}}, // received
    {{{}, {}, {"A"}}, {{"B"}, {}, {}}, {{}, {}, {}}}, // sent
    createConfig({{{"A", "Bm"}}, {{"A", "Ac"}, {"B", "Bc"}}, {{"C", "At"}}}),
    {{{}, {"B"}, {}}, {{}, {}, {}}, {{"aA"}, {}, {}}}, // received
    {{{}, {}, {"A"}}, {{"B"}, {}, {}}, {{}, {}, {}}} // sent
  },
  // 2->1
  Parameters // 1
  {
    createConfig({{{"A", "Bm"}}, {{"A", "Ac"}, {"B", "Bc"}}, {{"D", "Bt"}}}),
    {{{}, {"B"}, {}}, {{}, {}, {}}, {{"aA"}, {"bA"}, {}}}, // received
    {{{}, {}, {"A"}}, {{"B"}, {}, {"A"}}, {{}, {}, {}}}, // sent
    createConfig({{}, {{"B", "Bc"}}, {{"D", "Bt"}}}, {"A"}),
    {{{}, {}, {}}, {{}, {}, {}}, {{}, {}, {}}}, // received
    {{{}, {}, {}}, {{}, {}, {}}, {{}, {}, {}}} // sent
  },
  Parameters // 2
  {
    createConfig({{}, {{"B", "Bc"}}, {{"D", "Bt"}}}, {"A"}),
    {{{}, {}, {}}, {{}, {}, {}}, {{}, {}, {}}}, // received
    {{{}, {}, {}}, {{}, {}, {}}, {{}, {}, {}}}, // sent
    createConfig({{{"A", "Bm"}}, {{"A", "Ac"}, {"B", "Bc"}}, {{"D", "Bt"}}}),
    {{{}, {"B"}, {}}, {{}, {}, {}}, {{"aA"}, {"bA"}, {}}}, // received
    {{{}, {}, {"A"}}, {{"B"}, {}, {"A"}}, {{}, {}, {}}} // sent
  },
  // Same name as a possible alias.
  Parameters // 3
  {
    createConfig({{{"A", "Ac"}}, {{"B", "Bc"}}, {{"C", "At"}}}),
    {{{}, {}, {}}, {{}, {}, {}}, {{"aA"}, {}, {}}}, // received
    {{{}, {}, {"A"}}, {{}, {}, {}}, {{}, {}, {}}}, // sent
    createConfig({{{"aA", "Ct"}}, {{"B", "Bc"}}, {{"C", "At"}}}),
    {{{}, {"B"}, {}}, {{}, {}, {}}, {{"aA"}, {}, {}}}, // received
    {{{}, {}, {"aA"}}, {{"B"}, {}, {}}, {{}, {}, {}}} // sent
  },
  // Provide possible alias and then really as alias.
  Parameters // 4
  {
    createConfig({{}, {{"B", "Bc"}, {"aA", "Ct"}}, {{"C", "At"}}}),
    {{{}, {}, {}}, {{}, {}, {}}, {{}, {"aA"}, {}}}, // received
    {{{}, {}, {}}, {{}, {}, {"aA"}}, {{}, {}, {}}}, // sent
    createConfig({{{"A", "Ac"}}, {}, {{"C", "At"}}}),
    {{{}, {}, {}}, {{}, {}, {}}, {{"aA"}, {}, {}}}, // received
    {{{}, {}, {"A"}}, {{}, {}, {}}, {{}, {}, {}}} // sent
  }
));
