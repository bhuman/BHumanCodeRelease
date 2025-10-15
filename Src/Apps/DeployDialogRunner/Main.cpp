#include "Platform/File.h"
#include <cstdlib>
#include <filesystem>

#define STR(x) STR2(x)
#define STR2(x) #x

int main()
{
  std::filesystem::current_path("Make/Common");
  std::system("bash ./deployDialog " STR(CONFIGURATION));
  return 0;
}
