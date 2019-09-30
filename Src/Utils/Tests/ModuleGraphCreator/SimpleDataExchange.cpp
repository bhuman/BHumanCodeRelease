/**
 * @file Tools/ModuleGraphCreator/SimpleDataExchange.cpp
 *
 * This file implements tests for simple data exchange.
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
 * Cm: A,C -> A,B
 *
 * It is assumed that the module graoh creator behaves in the same way for each thread.
 */

// Test shared
TEST_P(ModuleGraphCreatorSimple, sharedTest)
{
  calcShared(GetParam().config);
  EXPECT_EQ(GetParam().received, received());
  EXPECT_EQ(GetParam().sent, sent());
  calcShared(GetParam().config2);
  EXPECT_EQ(GetParam().received2, received());
  EXPECT_EQ(GetParam().sent2, sent());
}

// Test all things together
TEST_P(ModuleGraphCreatorSimple, updateTest)
{
  calcShared(GetParam().config);
  EXPECT_EQ(GetParam().received, received());
  EXPECT_EQ(GetParam().sent, sent());
  update(GetParam().config2);
  EXPECT_EQ(GetParam().received2, received());
  EXPECT_EQ(GetParam().sent2, sent());
  // Assert no errormessage present
  ASSERT_DEATH(exit(-1), "^$");
}

// No communication, change a module.
INSTANTIATE_TEST_CASE_P(ChangeModule, ModuleGraphCreatorSimple, testing::Values(
  // Provide the same.
  Parameters // 0
  {
    createConfig({{{"A", "Ac"}, {"B", "Bc"}}}),
    {{{}}}, {{{}}},
    createConfig({{{"A", "Ac"}, {"B", "Cc"}}}),
    {{{}}}, {{{}}}
  },
  // Provide something different.
  Parameters // 1
  {
    createConfig({{{"A", "Ac"}, {"B", "Bc"}}}),
    {{{}}}, {{{}}},
    createConfig({{{"A", "Ac"}, {"C", "Dc"}}}),
    {{{}}}, {{{}}}
  },
  // Change provider of send data.
  Parameters // 2
  {
    createConfig({{{"A", "Ac"}, {"B", "Bc"}}, {{"A", "Bm"}}}),
    {{{}, {}}, {{"B"}, {}}}, // received
    {{{}, {"B"}}, {{}, {}}}, // sent
    createConfig({{{"A", "Ac"}, {"B", "Cc"}}, {{"A", "Bm"}}}),
    {{{}, {}}, {{"B"}, {}}}, // received
    {{{}, {"B"}}, {{}, {}}} // sent
  },
  Parameters // 3
  {
    createConfig({{{"A", "Bm"}}, {{"A", "Ac"}, {"B", "Bc"}}}),
    {{{}, {"B"}}, {{}, {}}}, // received
    {{{}, {}}, {{"B"}, {}}}, // sent
    createConfig({{{"A", "Bm"}}, {{"A", "Ac"}, {"B", "Cc"}}}),
    {{{}, {"B"}}, {{}, {}}}, // received
    {{{}, {}}, {{"B"}, {}}} // sent
  }
));

// Receive/sent changes.
INSTANTIATE_TEST_CASE_P(ChangeTrafficAmount, ModuleGraphCreatorSimple, testing::Values(
  // Receive/sent 0->1/1->0
  Parameters // 0
  {
    createConfig({{{"B", "Bc"}, {"C", "Ec"}}, {{"A", "Bm"}}}),
    {{{}, {"A"}}, {{"B"}, {}}}, // received
    {{{}, {"B"}}, {{"A"}, {}}}, // sent
    createConfig({{{"B", "Bc"}}, {{"A", "Bm"}}}),
    {{{}, {}}, {{"B"}, {}}}, // received
    {{{}, {"B"}}, {{}, {}}} // sent
  },
  Parameters // 1
  {
    createConfig({{{"B", "Bc"}}, {{"A", "Bm"}}}),
    {{{}, {}}, {{"B"}, {}}}, // received
    {{{}, {"B"}}, {{}, {}}}, // sent
    createConfig({{{"B", "Bc"}, {"C", "Ec"}}, {{"A", "Bm"}}}),
    {{{}, {"A"}}, {{"B"}, {}}}, // received
    {{{}, {"B"}}, {{"A"}, {}}} // sent
  },
  // Receive/sent 1->2/2->1
  Parameters // 2
  {
    createConfig({{{"B", "Cc"}, {"C", "Ec"}}, {{"D", "Am"}, {"A", "Bm"}}}),
    {{{}, {"A"}}, {{"C", "B"}, {}}}, // received
    {{{}, {"C", "B"}}, {{"A"}, {}}}, // sent
    createConfig({{{"B", "Cc"}, {"C", "Ec"}}, {{"A", "Bm"}}}),
    {{{}, {"A"}}, {{"B"}, {}}}, // received
    {{{}, {"B"}}, {{"A"}, {}}} // sent
  },
  Parameters // 3
  {
    createConfig({{{"B", "Cc"}, {"C", "Ec"}}, {{"A", "Bm"}}}),
    {{{}, {"A"}}, {{"B"}, {}}}, // received
    {{{}, {"B"}}, {{"A"}, {}}}, // sent
    createConfig({{{"B", "Cc"}, {"C", "Ec"}}, {{"A", "Bm"}, {"D", "Am"}}}),
    {{{}, {"A"}}, {{"B", "C"}, {}}}, // received
    {{{}, {"B", "C"}}, {{"A"}, {}}} // sent
  }
));

// Representations are provided in both threads and should not be shared.
INSTANTIATE_TEST_CASE_P(ProvidedInBoth, ModuleGraphCreatorSimple, testing::Values(
  // Switch shared
  Parameters // 0
  {
    createConfig({{{"A", "Ac"}, {"B", "Bc"}}, {{"A", "Bm"}}}),
    {{{}, {}}, {{"B"}, {}}}, // received
    {{{}, {"B"}}, {{}, {}}}, // sent
    createConfig({{{"A", "Ac"}, {"B", "Cc"}}, {{"A", "Bm"}}}),
    {{{}, {}}, {{"B"}, {}}}, // received
    {{{}, {"B"}}, {{}, {}}} // sent
  },
  // Not send/received anymore
  Parameters // 1
  {
    createConfig({{{"B", "Bc"}, {"C", "Ec"}}, {{"A", "Bm"}}}),
    {{{}, {"A"}}, {{"B"}, {}}}, // received
    {{{}, {"B"}}, {{"A"}, {}}}, // sent
    createConfig({{{"A", "Ac"}, {"B", "Bc"}, {"C", "Ec"}}, {{"A", "Bm"}}}),
    {{{}, {}}, {{"B"}, {}}}, // received
    {{{}, {"B"}}, {{}, {}}} // sent
  },
  Parameters // 2
  {
    createConfig({{{"A", "Ac"}, {"B", "Bc"}, {"C", "Ec"}}, {{"A", "Bm"}}}),
    {{{}, {}}, {{"B"}, {}}}, // received
    {{{}, {"B"}}, {{}, {}}}, // sent
    createConfig({{{"B", "Bc"}, {"C", "Ec"}}, {{"A", "Bm"}}}),
    {{{}, {"A"}}, {{"B"}, {}}}, // received
    {{{}, {"B"}}, {{"A"}, {}}} // sent
  }
));

// Module require and provide same representation
INSTANTIATE_TEST_CASE_P(RequireProvideSame, ModuleGraphCreatorSimple, testing::Values(
  Parameters // 0
  {
    createConfig({{{"C", "Dc"}}, {{"A", "Cm"}, {"B", "Cm"}}}),
    {{{}, {}}, {{"C"}, {}}}, // received
    {{{}, {"C"}}, {{}, {}}}, // sent
    createConfig({{{"C", "Dc"}, {"A", "Ac"}}, {{"A", "Cm"}, {"B", "Cm"}}}),
    {{{}, {}}, {{"C"}, {}}}, // received
    {{{}, {"C"}}, {{}, {}}} // sent
  }
));

// Two different configs.
INSTANTIATE_TEST_CASE_P(SwitchConfig, ModuleGraphCreatorSimple, testing::Values(
  Parameters // 0
  {
    createConfig({{{"B", "Bc"}}, {{"A", "Bm"}}}),
    {{{}, {}}, {{"B"}, {}}}, // received
    {{{}, {"B"}}, {{}, {}}}, // sent
    createConfig({{{"C", "Dc"}}, {{"D", "Am"}}}),
    {{{}, {}}, {{"C"}, {}}}, // received
    {{{}, {"C"}}, {{}, {}}} // sent
  },
  Parameters // 1
  {
    createConfig({{{"B", "Bc"}, {"C", "Ec"}}, {{"D", "Am"}, {"A", "Bm"}}}),
    {{{}, {"A"}}, {{"C", "B"}, {}}}, // received
    {{{}, {"C", "B"}}, {{"A"}, {}}}, // sent
    createConfig({{{"A", "Ac"}, {"C", "Dc"}}, {{"B", "Cm"}}}),
    {{{}, {}}, {{"A", "C"}, {}}}, // received
    {{{}, {"A", "C"}}, {{}, {}}} // sent
  }
));

INSTANTIATE_TEST_CASE_P(Default, ModuleGraphCreatorSimple, testing::Values(
  Parameters // 0
  {
    createConfig({{}, {}}),
    {{{}, {}}, {{}, {}}}, {{{}, {}}, {{}, {}}},
    createConfig({{{"B", "Cc"}}, {}}, {"A"}),
    {{{}, {}}, {{}, {}}}, {{{}, {}}, {{}, {}}}
  }
));
