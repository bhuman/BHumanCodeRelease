/**
* @file Simulation/Reader.h
* Declaration of class Reader
* @author Colin Graf
*/

#pragma once

#include <string>
#include <unordered_map>

/**
* @class Reader
* A c++ wrapper of a libxml2 xml file reader
*/
class Reader
{
protected:
  typedef std::unordered_map<std::string, std::pair<std::string, int> > Attributes;

  int line; /**< The current line number */
  int column; /**< The current column */
  std::string fileName; /**< The current file name */

  /** Destructor */
  ~Reader();

  /**
  * Reads a xml file. This function can be called recursively .
  * @param fileName The name of the file
  * @return Whether the file was read without errors
  */
  bool readFile(const std::string& fileName);

  /**
  * Reads the subordinate nodes of a node (if any)
  * @param callHandler Whether the handler function (\c handleNode) should be called for each subordinate node. Set it to \c false to pass the children of unexepcted elements.
  * @return Whether an error occured
  */
  bool readSubNodes(bool callHandler);

  /**
  * A handler called for syntax errors
  * @param msg An error message
  */
  virtual void handleError(const std::string& msg) = 0;

  /**
  * A handler called for each parsed node. Call readSubNodes() within in the handler to parse subordinate nodes.
  * @param node The name of the node
  * @param attributes The attributes of the node
  */
  virtual void handleNode(const std::string& name, Attributes& attributes) = 0;

  /**
  * A handler called when text was parsed
  * @param text The text read
  */
  virtual void handleText(std::string& text) = 0;

private:
  void* reader; /**< A libxml2 reader instance */
  std::string nodeName; /**< The current node */
  Attributes attributes; /**< A storage for the attributes of an element */
};
