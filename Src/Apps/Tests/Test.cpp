#include "Platform/SystemCall.h"
#include "Streaming/FunctionList.h"
#include <gtest/gtest.h>

int main(int argc, char** argv)
{
  FunctionList::execute();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

SystemCall::Mode SystemCall::getMode()
{
  return simulatedRobot;
}
