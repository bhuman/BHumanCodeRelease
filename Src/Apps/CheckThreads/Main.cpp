#include "Framework/Configuration.h"
#include "Framework/ModuleGraphCreator.h"
#include "Platform/SystemCall.h"
#include "Streaming/FunctionList.h"
#include "Streaming/InStreams.h"
#include "Streaming/Output.h"
#include <cstdlib>

int main(int argc, char** argv)
{
  if(argc != 2)
  {
    OUTPUT_ERROR("Usage: CheckThreads <threads.cfg>");
    return EXIT_FAILURE;
  }
  FunctionList::execute();
  Configuration config;
  {
    InMapFile stream(argv[1]);
    if(!stream.exists())
    {
      OUTPUT_ERROR(argv[1] << ": Could not open the file or the file is empty.");
      return EXIT_FAILURE;
    }
    stream >> config;
    if(config().empty())
    {
      OUTPUT_ERROR(argv[1] << ": Could not parse the file to a configuration.");
      return EXIT_FAILURE;
    }
  }
  InMapFile stream(argv[1]);
  ModuleGraphCreator moduleGraphCreator(config);
  if(!stream.exists() || !moduleGraphCreator.update(stream))
  {
    OUTPUT_ERROR(argv[1] << ": Invalid threads configuration for this scenario. See error above.");
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}

SystemCall::Mode SystemCall::getMode()
{
  return SystemCall::logFileReplay;
}
