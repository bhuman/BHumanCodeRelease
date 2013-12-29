/**
* @file Controller/Representations/ModuleInfo.h
*
* Declaration of class ModuleInfo
*
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

#include "Tools/MessageQueue/InMessage.h"
#include <list>
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
  class Module
  {
  public:
    std::string name; /**< The name of the module. */
    std::string category; /**< The category of the module. */
    std::vector<std::string> requirements; /**< The requirements of this module. */
    std::vector<std::string> representations; /**< The representations provided by this module. */
    char processIdentifier; /** The process in which this module is defined. */

    /**
    * Comparison operator. Only uses the module name for comparison.
    * @param other The module name this one is compared to.
    * @return Are the module names the same?
    */
    bool operator==(const std::string& other) const {return name == other;}

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

  class Provider
  {
  public:
    std::string representation; /**< The name of the representation provided. */
    std::string selected; /**< The selected module. Empty if no provider is selected for this representation. */
    std::vector<std::string> modules; /**< The names of the modules providing it. */
    char processIdentifier; /** The process in which this represenation is provided. */

    /**
    * Comparison operator. Only uses the representaion name for comparison.
    * @param other The representaion name this one is compared to.
    * @return Are the representation names the same?
    */
    bool operator==(const std::string& other) const {return representation == other;}

    /**
    * Comparison operator. Only uses the representaion name and the process identifier for comparison.
    * @param other The other provider this one is compared to.
    * @return Are the representation names and processIdentifiers the same?
    */
    bool operator==(const Provider& other) const {return representation == other.representation && processIdentifier == other.processIdentifier;}

    /**
    * Comparison operator. Uses the process identifier and the representation name for comparison.
    * @param other The other provider this one is compared to.
    * @return Is this module "provider" that the other?
    */
    bool operator<(const Provider& other) const
    {
      return processIdentifier != other.processIdentifier ? processIdentifier < other.processIdentifier :
             representation < other.representation;
    }
  };

  std::list<Module> modules; /**< All available modules. */
  std::list<Provider> providers; /**< All available providers. */
  unsigned timeStamp; /**< The time when the module information was last changed. */

  /**
  * Default constructor.
  */
  ModuleInfo() : timeStamp(0) {}

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
  * @return An error message. If empty, everything was ok.
  */
  std::string sendRequest(Out& stream, bool sort = false) const;
};
