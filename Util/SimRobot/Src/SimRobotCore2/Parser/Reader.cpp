/**
* @file Simulation/Reader.cpp
* Implementation of class Reader
* @author Colin Graf
*/

#include <libxml/xmlreader.h>

#include "Reader.h"
#include "Platform/Assert.h"

Reader::~Reader()
{
  xmlCleanupParser();
  xmlMemoryDump();
}

bool Reader::readFile(const std::string& fileName)
{
  // declare an error handler function
  class Handler
  {
  public:
    static void xmlErrorHandler(Reader* reader, const char * msg, xmlParserSeverities severity, xmlTextReaderLocatorPtr locator)
    {
      reader->line = xmlTextReaderLocatorLineNumber(locator);
      reader->column = 0;
      reader->handleError(msg);
    }
  };

  // open file
  xmlTextReaderPtr reader = xmlReaderForFile(fileName.c_str(), "UTF-8" /* "ISO-8859-1" */, 0); // using UTF-8 since it is the native format used by libxml2
  if(!reader)
    return false;
  xmlTextReaderSetErrorHandler((xmlTextReaderPtr)reader, (xmlTextReaderErrorFunc)&Handler::xmlErrorHandler, this);

  // backup reader state (except node name and attributes)
  std::string oldFileName = fileName;
  int oldLine = line;
  int oldColumn = column;
  xmlTextReaderPtr oldReader = (xmlTextReaderPtr)this->reader;

  // set new reader state
  this->fileName.swap(oldFileName);
  this->reader = reader;

  // read root node
  bool result = readSubNodes(true);

  // close reader
  xmlTextReaderClose((xmlTextReaderPtr)reader);
  xmlFreeTextReader((xmlTextReaderPtr)reader);

  // restore old reader state
  if(!oldFileName.empty())
  {
    this->fileName.swap(oldFileName);
    line = oldLine;
    column = oldColumn;
  }
  this->reader = oldReader;

  return result;
}

bool Reader::readSubNodes(bool callHandler)
{
  if(!reader) // the last element was "empty"
    return true;

  int ret;
  for(;;)
  {
    ret = xmlTextReaderRead((xmlTextReaderPtr)reader);
    if(ret != 1) // 1 = alright, 0 = eof, -1 = error
      return ret == 0;;

    switch(xmlTextReaderNodeType((xmlTextReaderPtr)reader))
    {
      case XML_READER_TYPE_ELEMENT:
      {
        bool isEmptyElement = xmlTextReaderIsEmptyElement((xmlTextReaderPtr)reader) != 0;

        if(!isEmptyElement && !callHandler)
          readSubNodes(false);
        else
        {
          line = xmlTextReaderGetParserLineNumber((xmlTextReaderPtr)reader);
          column = xmlTextReaderGetParserColumnNumber((xmlTextReaderPtr)reader);
          const xmlChar* xmlString = xmlTextReaderConstName((xmlTextReaderPtr)reader);
          nodeName = (const char*)xmlString;

          attributes.clear();
          int numberOfAttributes = xmlTextReaderAttributeCount((xmlTextReaderPtr)reader);
          if(numberOfAttributes > 32)
            numberOfAttributes = 32;

          xmlTextReaderMoveToFirstAttribute((xmlTextReaderPtr)reader);
          for(int i = 0; i < numberOfAttributes; i++)
          {
            xmlString = xmlTextReaderConstName((xmlTextReaderPtr)reader);
            std::string attName((const char*)xmlString);
            xmlString = xmlTextReaderConstValue((xmlTextReaderPtr)reader);
            std::string attValue((const char*)xmlString);
            attributes[attName] = std::make_pair(attValue, i);
            xmlTextReaderMoveToNextAttribute((xmlTextReaderPtr)reader);
          }

          if(isEmptyElement)
          {
            xmlTextReaderPtr reader = (xmlTextReaderPtr)this->reader;
            this->reader = 0;
            handleNode(nodeName, attributes);
            this->reader = reader;
          }
          else
            handleNode(nodeName, attributes);
        }
      }
      break;
      case XML_READER_TYPE_END_ELEMENT:
        line = xmlTextReaderGetParserLineNumber((xmlTextReaderPtr)reader);
        column = xmlTextReaderGetParserColumnNumber((xmlTextReaderPtr)reader);
        return true;
      case XML_READER_TYPE_TEXT:
        if(callHandler)
        {
          line = xmlTextReaderGetParserLineNumber((xmlTextReaderPtr)reader);
          column = xmlTextReaderGetParserColumnNumber((xmlTextReaderPtr)reader);
          xmlChar* xmlString = xmlTextReaderReadString((xmlTextReaderPtr)reader);
          std::string text = (const char*)xmlString;
          handleText(text);
          xmlFree(xmlString);
        }
        break;
    }
  }
  ASSERT(false);
  return false;
}

