#include <iomanip>
#include <sstream>

int main(int argc, char* argv[])
{
  std::stringstream commandLine;
  commandLine << "bash -c ";
  //short circuit if only one argument is given
  if(2 == argc)
  {
    commandLine << std::quoted(argv[1]);
  }
  else
  {
    //join all arguments
    std::stringstream commandLineArguments;
    for(int i = 1; i < argc; ++i)
    {
      //join arguments with space
      if(1 < i)
      {
        commandLineArguments << ' ';
      }
      //quote every argument
      commandLineArguments << std::quoted(argv[i]);
    }
    //quote final argument
    commandLine << std::quoted(commandLineArguments.str());
  }
  //execute command and return exit code
  return std::system(commandLine.str().c_str());
}
