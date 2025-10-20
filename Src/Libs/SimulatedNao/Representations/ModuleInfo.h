/**
 * @file Representations/ModuleInfo.h
 *
 * Declaration of class ModuleInfo
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Framework/Configuration.h"
#include "Framework/Module.h"
#include "Streaming/MessageQueue.h"
#include <list>
#include <set>
#include <string>
#include <vector>

/**
 * @class ModuleInfo
 *
 * A class to represent modules and selected providers.
 *
 * @author Thomas Röfer
 */
class ModuleInfo
{
public:
  struct Module
  {
    std::string name; /**< The name of the module. */
    std::vector<std::string> requirements; /**< The requirements of this module. */
    std::vector<std::string> representations; /**< The representations provided by this module. */

    /**
     * Comparison operator. Only uses the module name for comparison.
     * @param other The module name this one is compared to.
     * @return Are the module names the same?
     */
    bool operator==(const std::string& other) const { return name == other; }

    /**
     * Comparison operator. Uses the module name for comparison.
     * @param other The module this one is compared to.
     * @return Is this module "smaller" that the other?
     */
    bool operator<(const Module& other) const
    {
      return name < other.name;
    }
  };

  std::list<Module> modules; /**< All available modules. */
  std::set<std::string> representations; /**< All available representations. */
  Configuration config; /**< The current module configuration. */
  unsigned timestamp = 0; /**< The time when the module information was last changed. */

  /**
   * The method clears all tables.
   */
  void clear();

  /**
   * The function handles a module table message.
   * @param message The message.
   * @return Was it a module table message?
   */
  bool handleMessage(MessageQueue::Message message);

  /**
   * The method writes a module request to a stream.
   * @param stream The stream the request is written to.
   * @param sort Should the provider and representation list be sorted?
   */
  void sendRequest(Out& stream, bool sort = false);
};
