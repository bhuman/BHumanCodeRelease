/**
 * @file Tools/ModuleGraphCreator/ErrorTests.cpp
 *
 * This file implements Tests for all error cases.
 *
 * @author Jan Fiedler
 */

#include "Tools/FunctionList.h"
#include "Utils/Tests/ModuleGraphCreator/ModuleGraphCreatorTest.h"

#include <gtest/gtest.h>
#include <ostream>

/*
 * Require:
 * Ac: -> A
 * Bc: -> B
 * Cc: A -> B
 * Dc: -> C
 * Bm: B -> A
 * At: aA -> C
 * Bt: aA,bA -> D
 * Ct: B -> aA
 * Dt: cA -> B
 * Et: aB -> C
 *
 * Require not:
 * XYZ: -> a
 * ABC: -> b
 */

struct Errors
{
  Configuration config;
  std::string error;

  friend std::ostream& operator<<(std::ostream& os, const Errors& obj)
  {
    return os
           << "config: threads=" << obj.config().size()
           << "; error: " << obj.error;
  }
};

class ModuleGraphCreatorDeathTest : public testing::TestWithParam<Errors> {};

TEST_P(ModuleGraphCreatorDeathTest,)
{
  FunctionList::execute();
  ModuleGraphCreator graphCreator(GetParam().config);
  OutBinaryMemory out(100);
  out << GetParam().config;
  InBinaryMemory in(out.data());
  // Comma operator to avoid the error being written to the console.
  ASSERT_DEATH((graphCreator.update(in), exit(-1)), GetParam().error);
}

// OUTPUT_ERROR(config()[index].name << ": Module " << rp.provider << " is unknown!");
INSTANTIATE_TEST_CASE_P(UnknownModule, ModuleGraphCreatorDeathTest, testing::Values(
  // No module.
  Errors { createConfig({{{"", ""}}}),
    "a: Module  is unknown!\n$"},
  // No existing module.
  Errors {createConfig({{{"", "XYZ"}}}),
    "a: Module XYZ is unknown!\n$"},
  Errors {createConfig({{{"", "XYZ"}, {"A", "ABC"}}}),
    "a: Module .* is unknown!\n$"},
  // Not existing module.
  Errors {createConfig({{{"A", "Ac"}, {"", "XYZ"}}}),
    "a: Module XYZ is unknown!\n$"}
));

// OUTPUT_ERROR("Default representation " << rrepresentation << " is not required anywhere!");
INSTANTIATE_TEST_CASE_P(UnknownRepresentation, ModuleGraphCreatorDeathTest, testing::Values(
  // No existing representation.
  Errors { createConfig({}, {"a"}),
    "Default representation a is not required anywhere!\n$"},
  Errors { createConfig({}, {"a", "b"}),
    "Default representation a is not required anywhere!\n$"},
  // Not existing representation.
  Errors { createConfig({{{"A", "Ac"}}}, {"a"}),
    "Default representation a is not required anywhere!\n$"},
  // Existing but not used representation.
  Errors { createConfig({}, {"A"}),
    "Default representation A is not required anywhere!\n$"}

));

// OUTPUT_ERROR(config()[index].name << ": " << rp.provider << " does not provide " << rp.representation << "!");
INSTANTIATE_TEST_CASE_P(WrongProvider, ModuleGraphCreatorDeathTest, testing::Values(
  // Provide not existing representation.
  Errors { createConfig({{{"", "Ac"}}}),
    "a: Ac does not provide !\n$"},
  Errors { createConfig({{{"a", "Ac"}}}),
    "a: Ac does not provide a!\n$"},
  // Wrong provider for representation.
  Errors { createConfig({{{"B", "Ac"}}}),
    "a: Ac does not provide B!\n$"}
));

// OUTPUT_ERROR("No provider for required representation " << requirement.representation << " required by " << module.module->name);
INSTANTIATE_TEST_CASE_P(NoProvider, ModuleGraphCreatorDeathTest, testing::Values(
  // No provider for representation.
  Errors {createConfig({{  {"B", "Cc"}}}),
    "a: No provider for required representation A required by Cc\n$" }
));

// OUTPUT_ERROR(thread.name << ": " << a->representation << " is provided by more than one module!");
INSTANTIATE_TEST_CASE_P(MultipleProvider, ModuleGraphCreatorDeathTest, testing::Values(
  // More than one provider for representation.
  Errors {createConfig({{{"B", "Bc"}, {"B", "Cc"}}}),
    "a: B is provided by more than one module!\n$"},
  Errors{createConfig({{{"B", "Bc"}, {"B", "Bc"}}}),
    "a: B is provided by more than one module!\n$"}
));

// OUTPUT_ERROR("Requirements missing for providers for " << text2 << "!");
// OUTPUT_ERROR("Only found consistent providers for " << text << ".\nRequirements missing for providers for " << text2 << "!");
INSTANTIATE_TEST_CASE_P(CircleProvider, ModuleGraphCreatorDeathTest, testing::Values(
  // Full circle
  Errors {createConfig({{{"B", "Cc"}, {"A", "Bm"}}}),
    "a: Requirements missing for providers for A, B!\n$"},
  // Parts are a circle
  Errors {createConfig({{{"B", "Cc"}, {"A", "Bm"}, {"C", "Dc"}}}),
    "a: Only found consistent providers for C.\nRequirements missing for providers for A, B!\n$"}
));

// Uses received2.front().front().front() to save the error message.
TEST_P(ExpModuleGraphCreatorDeathTest,)
{
  ASSERT_DEATH((update(GetParam().config2), exit(-1)), GetParam().received2.front().front().front());
}

// Test again, because the error can be thrown at 2 places.
INSTANTIATE_TEST_CASE_P(UnknownModule, ExpModuleGraphCreatorDeathTest, testing::Values(
  // No module.
  Parameters {createConfig({{}}), {}, {},
    createConfig({{{"", ""}}}), {{{"a: Module  is unknown!\n$"}}}, {}},
  // No existing module.
  Parameters {createConfig({{}}), {}, {},
    createConfig({{{"", "XYZ"}}}), {{{"a: Module XYZ is unknown!\n$"}}}, {}},
  Parameters {createConfig({{}}), {}, {},
    createConfig({{{"", "XYZ"}, {"A", "ABC"}}}), {{{"a: Module XYZ is unknown!\n$"}}}, {}},
  // Not existing module.
  Parameters {createConfig({{}}), {}, {},
    createConfig({{{"A", "Ac"}, {"", "XYZ"}}}), {{{"a: Module XYZ is unknown!\n$"}}}, {}}
));

// The number of threads musst be even.
GTEST_TEST(ModuleGraphCreator, unevenDeathTest)
{
  ModuleGraphCreator moduleGraphCreator(createConfig({{{"A", "Ac"}}}));

  OutBinaryMemory out(100);
  out << createConfig({{{"A", "Ac"}}});
  InBinaryMemory in(out.data());
  moduleGraphCreator.update(in);

  OutBinaryMemory out2(100);
  out2 << createConfig({{{"A", "Ac"}}, {{"A", "Ac"}}});
  InBinaryMemory in2(out2.data());
  ASSERT_DEATH(moduleGraphCreator.update(in2),
               "ASSERT\\(prevConfig\\(\\).empty\\(\\) \\|\\| config\\(\\).empty\\(\\) "
                        "\\|\\| prevConfig\\(\\).size\\(\\) == config\\(\\).size\\(\\)\\) failed");
}

// Invalid default representations.
// OUTPUT_ERROR(config()[index].name << ": " << representation << " is also provided by default!");
INSTANTIATE_TEST_CASE_P(Default, ModuleGraphCreatorDeathTest, testing::Values(
  // Default and thread provide representation
  Errors {createConfig({{{"A", "Ac"}}, {{"B", "Cc"}}}, {"A"}),
    "a: A is also provided by default!\n$"},
  Errors{ createConfig({{{"A", "Bm"}}, {{"B", "Bc"}, {"D", "Bt"}}, {{"C", "Dc"}}}, {"D"}),
    "b: D is also provided by default!\n$"}
));

// Same representation requestet from multiple threads.
INSTANTIATE_TEST_CASE_P(ReprFromMultiThreads, ModuleGraphCreatorDeathTest, testing::Values(
  // OUTPUT_ERROR(config()[index].name << ": Representation " << rp.representation << " is provided by multiple threads!");
  Errors {createConfig({{{"A", "Ac"}}, {{"B", "Bc"}, {"A", "Bm"}}, {{"B", "Cc"}}}, /*threads*/ {{}, {}, {"a", "b"}}),
    "c: Representation A is provided by multiple threads!\n$"},
  //OUTPUT_ERROR(config()[index].name << ": The representation " << rp.substr(config()[aliasThreadIndex].name.size(), rp.size())
  //             << " that is expected from thread " << config()[aliasThreadIndex].name << " is not provided in this thread!");
  Errors {createConfig({{{"B", "Bc"}}, {{"C", "At"}}}, /*threads*/ {{}, {"a"}}),
    "b: The representation A that is expected from thread a is not provided in this thread!\n$"},
  //  OUTPUT_ERROR(config()[index].name << ": The representations " << rp.representation << " and the alias " << requirement.representation << " are not equal!");
  Errors {createConfig({{}, {{"B", "Dt"}}, {{"A", "Ac"}}}, /*threads*/ {{}, {"c"}, {}}),
    "b: The representations A and the alias cA are not equal!\n$"},
  Errors {createConfig({{{"B", "Bc"}}, {{"C", "Et"}}}, /*threads*/ {{}, {"a"}}),
    "b: The representations B and the alias aB are not equal!\n$"},
  // OUTPUT_ERROR(config()[index].name << ": Representation " << requirement.representation << " is also provided as alias!");
  Errors {createConfig({{{"A", "Ac"}}, {{"B", "Bc"}, {"aA", "Ct"}, {"C", "At"}}}, /*threads*/ {{}, {"a"}}),
    "b: Representation aA is also provided as alias!"},
  Errors {createConfig({{{"A", "Ac"}}, {{"B", "Bc"}, {"aA", "Ct"}}, {{"C", "At"}}}, /*threads*/ {{}, {}, {"a", "b"}}),
    "c: Representation aA is also provided as alias!"},
  Errors{ createConfig({{{"A", "Ac"}, {"B", "Bc"}, {"aA", "Ct"}}, {{"C", "At"}}}, /*threads*/ {{}, {"a"}}),
    "b: Representation aA is also provided as alias!" }
));
