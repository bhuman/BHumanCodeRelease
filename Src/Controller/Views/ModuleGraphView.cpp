/**
* @file Controller/Views/ModuleGraphView.cpp
* Implementation of a class to represent a view displaying the module layout of the process.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
* @author Colin Graf
*/

#include "ModuleGraphView.h"
#include "Controller/RobotConsole.h"

#include <sstream>
#include <algorithm>

ModuleGraphViewObject::ModuleGraphViewObject(const QString& fullName, RobotConsole& console, char processIdentifier, const std::string& category) :
  DotViewObject(fullName), console(console), processIdentifier(processIdentifier), category(category), lastModulInfoTimeStamp(0) {}

bool ModuleGraphViewObject::hasChanged()
{
  SYNC_WITH(console);
  return lastModulInfoTimeStamp != console.moduleInfo.timeStamp;
}

QString ModuleGraphViewObject::generateDotFileContent()
{
  SYNC_WITH(console);
  lastModulInfoTimeStamp = console.moduleInfo.timeStamp;
  const ModuleInfo& m = console.moduleInfo;
  bool success = false;
  bool defaultCreated = false;
  std::stringstream stream;
  stream << "digraph G {" << std::endl;
  stream << "node [style=filled,fillcolor=lightyellow,fontname=Arial"
#ifdef OSX
            "MT"
#endif
            ",fontsize=9,height=0.2];" << std::endl;
  stream << "concentrate = true;" << std::endl;

  for(const auto& i : m.modules)
    if(i.processIdentifier == processIdentifier && (category == "" || i.category == category))
    {
      bool used = false;
      for(const auto& j : i.representations)
      {
        for(const auto& k : m.config.representationProviders)
          if(k.representation == j && k.provider == i.name)
          {
            used = true;
            stream << j << " [style=filled,fillcolor=\"lightblue\"];" << std::endl;
            stream << i.name << "->" << j.c_str() << " [len=2];" << std::endl;
          }
      }
      if(used)
      {
        for(const auto& j : i.requirements)
        {
          for(const auto& k : m.config.representationProviders)
            if(k.representation == j)
            {
              if(k.provider == "default")
              {
                if(!defaultCreated)
                {
                  stream << "default[shape=box,fontcolor=red];" << std::endl;
                  defaultCreated = true;
                }
                stream << j << " [style=filled,fillcolor=\"lightblue\"];" << std::endl;
                stream << "default->" << j << " [len=2];" << std::endl;
                goto providerFound;
              }
              else
              {
                const auto& l = std::find(m.modules.begin(), m.modules.end(), k.provider);
                if(l != m.modules.end())
                {
                  if(l->processIdentifier != i.processIdentifier)
                    stream << j << " [style=\"filled,dashed\",fillcolor=\"#ffdec4\"];" << std::endl;
                  goto providerFound;
                }
              }
            }
          // no provider
          stream << j << " [style=\"filled,dashed\",color=red,fontcolor=red];" << std::endl;
        providerFound:
          stream << j << "->" << i.name << " [len=2];" << std::endl;
        }
        stream << i.name << "[shape=box];" << std::endl;
        success = true;
      }
    }

  stream << "}" << std::endl;
  return success ? QString(stream.str().c_str()) : QString();
}

std::string ModuleGraphViewObject::compress(const std::string& s) const
{
  std::string s2(s);
  if(!isalpha(s2[0]))
    s2[0] = '_';
  for(unsigned i = 1; i < s2.size(); ++i)
    if(!isalpha(s2[i]) && !isdigit(s2[i]))
      s2[i] = '_';
  return s2;
}
