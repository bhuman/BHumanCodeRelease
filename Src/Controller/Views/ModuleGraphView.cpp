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
  std::stringstream stream;
  stream << "digraph G {" << std::endl;
  stream << "node [style=filled,fillcolor=lightyellow,fontname=Arial,fontsize=9,height=0.2];" << std::endl;
  stream << "concentrate = true;" << std::endl;

  for(std::list<ModuleInfo::Module>::const_iterator i = m.modules.begin(); i != m.modules.end(); ++i)
    if(i->processIdentifier == processIdentifier && (category == "" || i->category == category))
    {
      bool used = false;
      for(std::vector<std::string>::const_iterator j = i->representations.begin(); j != i->representations.end(); ++j)
      {
        std::list<ModuleInfo::Provider>::const_iterator k = std::find(m.providers.begin(), m.providers.end(), *j);
        while(k != m.providers.end() && k->selected != i->name)
          k = std::find(++k, m.providers.end(), *j);
        if(k != m.providers.end())
        {
          used = true;
          stream << j->c_str() << " [style=filled,fillcolor=\"lightblue\"];" << std::endl;
          stream << i->name << "->" << j->c_str() << " [len=2];" << std::endl;
        }
      }
      if(used)
      {
        for(std::vector<std::string>::const_iterator j = i->requirements.begin(); j != i->requirements.end(); ++j)
        {
          std::list<ModuleInfo::Provider>::const_iterator k = std::find(m.providers.begin(), m.providers.end(), *j);
          while(k != m.providers.end() && (k->selected == "" || k->processIdentifier != processIdentifier))
            k = std::find(++k, m.providers.end(), *j);
          if(k == m.providers.end())
          {
            // no provider in same process
            k = std::find(m.providers.begin(), m.providers.end(), *j);
            while(k != m.providers.end() && k->selected == "")
              k = std::find(++k, m.providers.end(), *j);
            if(k == m.providers.end())
              stream << j->c_str() << " [style=\"filled,dashed\",color=red,fontcolor=red];" << std::endl;
            else
              stream << j->c_str() << " [style=\"filled,dashed\",fillcolor=\"#ffdec4\"];" << std::endl;
          }
          stream << j->c_str() << "->" << i->name << " [len=2];" << std::endl;
        }
        if(i->name == "default")
          stream << i->name.c_str() << "[shape=box,fontcolor=red];" << std::endl;
        else
          stream << i->name.c_str() << "[shape=box];" << std::endl;
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
