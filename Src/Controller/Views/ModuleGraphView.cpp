/**
 * @file Controller/Views/ModuleGraphView.cpp
 * Implementation of a class to represent a view displaying the module layout of the thread.
 * @author Thomas Röfer
 * @author Colin Graf
 */

#include "ModuleGraphView.h"
#include "Controller/RobotConsole.h"

#include <sstream>
#include <algorithm>

ModuleGraphViewObject::ModuleGraphViewObject(const QString& fullName, RobotConsole& console,  std::unordered_set<ModuleBase::Category> categories) :
  DotViewObject(fullName), console(console), categories(categories)
{}

bool ModuleGraphViewObject::hasChanged()
{
  SYNC_WITH(console);
  return lastModulInfoTimestamp != console.moduleInfo.timestamp;
}

QString ModuleGraphViewObject::generateDotFileContent()
{
  SYNC_WITH(console);
  lastModulInfoTimestamp = console.moduleInfo.timestamp;
  const ModuleInfo& m = console.moduleInfo;
  bool success = false;
  bool defaultCreated = false;
  std::stringstream stream;
  stream << "digraph G {" << std::endl;
  stream << "node [style=filled,fillcolor=lightyellow,fontname=Arial"
#ifdef MACOS
         "MT"
#endif
         ",fontsize=9,height=0.2];" << std::endl;
  stream << "concentrate = true;" << std::endl;

  for(const ModuleInfo::Module& i : m.modules)
    if(categories.find(static_cast<ModuleBase::Category>(ModuleBase::numOfCategories)) != categories.end() || categories.find(i.category) != categories.end())
    {
      bool used = false;
      size_t index = -1;
      for(const auto& j : i.representations)
      {
        for(size_t k = 0; k < m.config().size(); k++)
          for(const auto& l : m.config()[k].representationProviders)
            if(l.representation == j && l.provider == i.name)
            {
              index = k;
              used = true;
              stream << j << " [style=filled,fillcolor=\"lightblue\"];" << std::endl;
              stream << i.name << "->" << j.c_str() << " [len=2];" << std::endl;
            }
      }
      if(used)
      {
        for(const auto& j : i.requirements)
        {
          bool found = false;
          bool foundInSameThread = false;
          bool foundDefault = false;
          for(const std::string& representation : m.config.defaultRepresentations)
            if(representation == j)
            {
              foundDefault = true;
              goto exitTwoLoops;
            }
          for(const auto& k : m.config()[index].representationProviders)
            if(k.representation == j)
              for(const auto& l : m.modules)
              {
                found = foundInSameThread |= l == k.provider;
                if(foundInSameThread)
                  goto exitTwoLoops;
              }
          for(const Configuration::Thread& thread : m.config())
            if(thread.name != m.config()[index].name)
              for(const auto& rp : thread.representationProviders)
                if(rp.representation == j)
                  for(const auto& l : m.modules)
                  {
                    found |= l == rp.provider;
                    if(found)
                      goto exitTwoLoops;
                  }
        exitTwoLoops:

          if(!foundInSameThread)
          {
            if(foundDefault)
            {
              if(!defaultCreated)
              {
                stream << "default[shape=box,fontcolor=red];" << std::endl;
                defaultCreated = true;
              }
              stream << j << " [style=filled,fillcolor=\"lightblue\"];" << std::endl;
              stream << "default->" << j << " [len=2];" << std::endl;
            }
            else if(!found)
              stream << j << " [style=\"filled,dashed\",color=red,fontcolor=red];" << std::endl;
            else
              stream << j << " [style=\"filled,dashed\",fillcolor=\"#ffdec4\"];" << std::endl;
          }
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
