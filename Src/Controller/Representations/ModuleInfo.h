/**
 * @file Controller/Representations/ModuleInfo.h
 *
 * Declaration of class ModuleInfo
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Tools/MessageQueue/InMessage.h"
#include "Tools/Module/ModuleManager.h"
#include <list>
#include <set>
#include <string>
#include <vector>

/**
 * @class ModuleInfo
 *
 * A class to represent modules and selected providers.
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */
class ModuleInfo
{
public:
  struct Module
  {
    std::string name; /**< The name of the module. */
    ModuleBase::Category category; /**< The category of the module. */
    std::vector<std::string> requirements; /**< The requirements of this module. */
    std::vector<std::string> representations; /**< The representations provided by this module. */
    char processIdentifier; /** The process in which this module is defined. */

    /**
     * Comparison operator. Only uses the module name for comparison.
     * @param other The module name this one is compared to.
     * @return Are the module names the same?
     */
    bool operator==(const std::string& other) const { return name == other; }

    /**
     * Comparison operator. Uses the process identifier, the category and the
     * module name for comparison.
     * @param other The module this one is compared to.
     * @return Is this module "smaller" that the other?
     */
    bool operator<(const Module& other) const
    {
      return processIdentifier != other.processIdentifier ? processIdentifier < other.processIdentifier :
             category != other.category ? category < other.category : name < other.name;
    }
  };

  std::list<Module> modules; /**< All available modules. */
  std::set<std::string> representations; /**< All available representations. */
  ModuleManager::Configuration config; /**< The current module configuration. */
  unsigned timeStamp = 0; /**< The time when the module information was last changed. */

  /**
   * The method clears all tables.
   */
  void clear();

  /**
   * The function handles a module table message.
   * @param message The message.
   * @param processIdentifier The identifier of the process sending the module table.
   * @return Was it a module table message?
   */
  bool handleMessage(InMessage& message, char processIdentifier);

  /**
   * The method writes a module request to a stream.
   * @param stream The stream the request is written to.
   * @param sort Should the provider and representationlist be sorted?
   */
  void sendRequest(Out& stream, bool sort = false);
};
