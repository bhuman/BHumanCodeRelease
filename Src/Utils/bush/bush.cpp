#include "Utils/bush/Initializer.h"

int main(int argc, char** argv)
{
  Initializer initializer(argc, argv);
  return initializer.start();
}
