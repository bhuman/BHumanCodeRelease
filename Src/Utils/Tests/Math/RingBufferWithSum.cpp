#include "Tools/RingBufferWithSum.h"

#include "gtest/gtest.h"

GTEST_TEST(RingBufferWithSum, NotFullBufferSum)
{
  RingBufferWithSum<int> ring(static_cast<size_t>(8));

  ring.push_front(1);
  ring.push_front(2);
  ring.push_front(3);
  ring.push_front(4);

  for(int i = 5; i < 20; ++i)
  {
    int expectedSum = (i - 1) + (i - 2) + (i - 3) + (i - 4);
    EXPECT_EQ(expectedSum, ring.sum());
    ring.pop_back();
    ring.push_front(i);
  }
}
