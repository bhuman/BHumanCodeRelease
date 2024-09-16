/**
 * @file SimulatedNao/Views/ModuleGraphView.cpp
 * Implementation of a class to represent a view displaying the module layout of the thread.
 * @author Thomas Röfer
 * @author Colin Graf
 */

#include "ModuleGraphView.h"
#include "Framework/Module.h"
#include "SimulatedNao/RobotConsole.h"

#include <sstream>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>

ModuleGraphViewObject::ModuleGraphViewObject(const QString& fullName, RobotConsole& console, const std::string& threadName) :
  DotViewObject(fullName), console(console), threadName(threadName)
{}

bool ModuleGraphViewObject::hasChanged()
{
  SYNC_WITH(console);
  return lastModuleInfoTimestamp != console.moduleInfo.timestamp;
}

QString ModuleGraphViewObject::generateDotFileContent()
{
  SYNC_WITH(console);
  lastModuleInfoTimestamp = console.moduleInfo.timestamp;

  const std::vector<Configuration::Thread>& threads = console.moduleInfo.config();

  // Representations can be provided by "default".
  std::unordered_set<std::string> defaultRepresentations(console.moduleInfo.config.defaultRepresentations.begin(),
                                                         console.moduleInfo.config.defaultRepresentations.end());

  // To mark missing requirements, it must be known which representations are currently provided.
  // Possible names are "<representation>" and "<thread name><representation>".
  std::unordered_set<std::string> providedRepresentations;
  for(const Configuration::Thread& thread : threads)
  {
    for(const Configuration::RepresentationProvider& representationProvider : thread.representationProviders)
    {
      providedRepresentations.insert(representationProvider.representation);
      providedRepresentations.insert(thread.name + representationProvider.representation);
    }
    for(const std::string& representation : console.moduleInfo.config.defaultRepresentations)
      defaultRepresentations.insert(thread.name + representation);
  }

  const auto thread = std::find(threads.begin(), threads.end(), threadName);
  if(thread != threads.end())
  {
    // Setup some data structures for quicker searching.
    std::unordered_set<std::string> providersInThread;
    std::unordered_map<std::string, std::string> representationProvidersInThread;
    for(const Configuration::RepresentationProvider& representationProvider : thread->representationProviders)
    {
      providersInThread.insert(representationProvider.provider);
      representationProvidersInThread[representationProvider.representation] = representationProvider.provider;
    }

    // Create graph.
    std::stringstream stream;
    stream << "digraph G {" << std::endl;
    stream << "node [style=filled,fillcolor=lightyellow,fontname=Arial,fontsize=9,height=0.2];" << std::endl;
    stream << "concentrate = true;" << std::endl;

    bool defaultCreated = false;
    for(const ModuleInfo::Module& module : console.moduleInfo.modules)
      if(providersInThread.contains(module.name))
      {
        // Node for module.
        stream << module.name << "[shape=box];" << std::endl;

        // Outgoing edges to all representations that are actually provided by this module and their nodes.
        for(const std::string& representation : module.representations)
        {
          const auto representationProviderInThread = representationProvidersInThread.find(representation);
          if(representationProviderInThread != representationProvidersInThread.end()
             && representationProviderInThread->second == module.name)
          {
            stream << representation << " [style=filled,fillcolor=\"lightblue\"];" << std::endl;
            stream << module.name << "->" << representation.c_str() << " [len=2];" << std::endl;
          }
        }

        // Ingoing edges from all requirements of this module.
        for(const std::string& requirement : module.requirements)
        {
          // Requirement is provided by "default" -> create representation node.
          if(defaultRepresentations.contains(requirement))
          {
            // Create node for "default" if it does not exist yet.
            if(!defaultCreated)
            {
              stream << "default[shape=box,fontcolor=red];" << std::endl;
              defaultCreated = true;
            }
            stream << requirement << " [style=filled,fillcolor=\"lightblue\"];" << std::endl;
            stream << "default->" << requirement << " [len=2];" << std::endl;
          }

          // Create node for representation without a provider (illegal configuration).
          else if(!providedRepresentations.contains(requirement))
            stream << requirement << " [style=\"filled,dashed\",color=red,fontcolor=red];" << std::endl;

          // Create node for representation provided in a different thread.
          else if(!representationProvidersInThread.contains(requirement))
            stream << requirement << " [style=\"filled,dashed\",fillcolor=\"#ffdec4\"];" << std::endl;

          // Create edge from requirement to this module.
          stream << requirement << "->" << module.name << " [len=2];" << std::endl;
        }
      }

    stream << "}" << std::endl;
    return stream.str().c_str();
  }
  else
    return "";
}
