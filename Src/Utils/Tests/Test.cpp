#include "gtest/gtest.h"
#include "Platform/SystemCall.h"

#include <iostream>

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  std::cin.get();
  return result;
}

SystemCall::Mode SystemCall::getMode()
{
  return simulatedRobot;
}
