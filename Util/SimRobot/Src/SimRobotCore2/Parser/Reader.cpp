/**
* @file Simulation/Reader.cpp
* Implementation of class Reader
*
* It accepts the following language:
*
* file := elements
* element := '<' name attributes { whitespace } ( '/>' | '>' elements '</' name { whitespace } '>' ) |
*            '<!--' comment '-->'
* attributes := { whitespace } { whitespace attribute { whitespace } }
* attribute := name { whitespace } '=' { whitespace } string
* elements := { whitespace } { element { whitespace } | data }
* name := ( letter | '_' | ':' ) { ( letter | digit | '.' | '-' | '_' | ':' ) }
* string := '"' { stringchar } '"'
*
* whitespace := ' ' | '\n' | '\r' | '\t'
* comment := Any character sequence that does not contain the string -->
* data := Any character sequence that does not contain the character '<'
* letter := ( 'A' - 'Z' ) | ( 'a' - 'z' )
* digit := '0' - '9'
* stringchar := Any character, double quote and backslash must be escaped by a backslash
*
* @author Colin Graf
* @author Arne Hasselbring
*/

#include <cctype>
#include <fstream>
#include <iostream>

#include "Reader.h"
#include "Platform/Assert.h"

bool Reader::readFile(const std::string& fileName)
{
  // open file
  std::ifstream reader(fileName);
  if(!reader.is_open())
    return false;

  // backup reader state
  std::string oldFileName = fileName;
  const Location oldLocation = location;
  const Location oldNextLocation = nextLocation;

  // set new reader state
  this->reader.swap(reader);
  this->fileName.swap(oldFileName);
  nextLocation = location = Location(1, 1);
  ASSERT(nextToken == invalidToken);

  // read root elements
  bool result = readElements(true, true);

  // close reader
  this->reader.close();

  // restore old reader state
  if(!oldFileName.empty())
  {
    this->fileName.swap(oldFileName);
    location = oldLocation;
    nextLocation = oldNextLocation;
  }
  nextToken = invalidToken;
  this->reader.swap(reader);

  return result;
}

bool Reader::readElements(bool callHandler, bool isRoot)
{
  if(!reader.is_open()) // the last element was "empty"
    return true;

  Token token;
  while(tokenIsSpace(token = readToken()));

  for(;;)
  {
    if(token == tagStart || token == commentStart)
    {
      undoReadToken(token);
      if(!readElement(callHandler))
        return false;
      while(tokenIsSpace(token = readToken()));
    }
    else if(token == endTagStart)
    {
      if(isRoot)
      {
        handleError("Unexpected end tag without matching opening tag", location);
        return false;
      }
      undoReadToken(endTagStart);
      return true;
    }
    else if(token == endOfInput)
    {
      if(!isRoot)
        handleError("Unexpected end of file", location);
      return isRoot;
    }
    else
    {
      undoReadToken(token);
      if(!readData(callHandler))
        return false;
      token = readToken();
    }
  }
}

bool Reader::readElement(bool callHandler)
{
  Token token = readToken();
  if(token == tagStart)
  {
    Location nameLocation;
    if(!readName(nameLocation))
      return false;
    const std::string name = tmpString;
    attributes.clear();
    if(!readAttributes())
      return false;
    token = readToken();
    if(token == emptyTagEnd)
    {
      if(callHandler)
      {
        // This signals further calls to readElements that there is nothing to read
        std::ifstream f;
        reader.swap(f);
        handleElement(name, attributes, nameLocation);
        reader.swap(f);
      }
      return true;
    }
    else if(token == tagEnd)
    {
      bool result;
      if(callHandler)
      {
        // handleElement is responsible for calling readElements.
        result = handleElement(name, attributes, nameLocation);
      }
      else
        result = readElements(false);
      if(!result)
        return false;
      token = readToken();
      if(token == endTagStart)
      {
        Location endNameLocation;
        if(!readName(endNameLocation))
          return false;
        const std::string& endName = tmpString;
        while(tokenIsSpace(token = readToken()));
        if(token == tagEnd)
        {
          if(name != endName)
          {
            handleError("End tag does not match", endNameLocation);
            handleError("Note: Matching tag is here", nameLocation);
            return false;
          }
          return true;
        }
        else
        {
          handleError("Expected tag end", location);
          return false;
        }
      }
      // readElements would not have returned true if the next token was not endTagStart.
      ASSERT(false);
      return false;
    }
    else
    {
      handleError("Expected tag end", location);
      return false;
    }
  }
  else if(token == commentStart)
  {
    const Location startCommentLocation = location;
    while((token = readToken()) != commentEnd)
    {
      if(token == endOfInput)
      {
        handleError("Unterminated comment", startCommentLocation);
        return false;
      }
    }
    return true;
  }
  // This function is only called if one of the above conditions is true.
  ASSERT(false);
  return false;
}

bool Reader::readAttributes()
{
  for(;;)
  {
    bool readWhitespace = false;
    Token token;
    while(tokenIsSpace(token = readToken()) && (readWhitespace = true));
    undoReadToken(token);
    if(token == tagEnd || token == emptyTagEnd)
      return true;
    else if(token == endOfInput)
    {
      handleError("Unexpected end of file", location);
      return false;
    }
    else if(!readWhitespace)
    {
      handleError("Attributes must be separated by whitespace characters", location);
      return false;
    }
    else if(!readAttribute())
      return false;
  }
}

bool Reader::readAttribute()
{
  Location nameLocation;
  if(!readName(nameLocation))
    return false;
  const std::string name = tmpString;
  Token token;
  while(tokenIsSpace(token = readToken()));
  if(token == equals)
  {
    while(tokenIsSpace(token = readToken()));
    undoReadToken(token);
    Location valueLocation;
    if(!readString(valueLocation))
      return false;
    const std::string& value = tmpString;
    attributes.emplace(name, Attribute(value, static_cast<int>(attributes.size()), nameLocation, valueLocation));
    return true;
  }
  else
  {
    if(token == endOfInput)
      handleError("Unexpected end of file", location);
    else
      handleError("Expected '=' after attribute name", location);
    return false;
  }
}

bool Reader::readName(Location& location)
{
  Token token = readToken();
  if(token >= firstNonCharToken || (!std::isalpha(static_cast<unsigned char>(token)) && token != underscore && token != colon))
  {
    if(token == endOfInput)
      handleError("Unexpected end of file", this->location);
    else if(tokenIsSpace(token))
      handleError("Unexpected whitespace", this->location);
    else
      handleError("A name must begin with an alphabetic letter, underscore or colon", this->location);
    return false;
  }
  location = this->location;
  tmpString.clear();
  tmpString += static_cast<char>(token);
  for(;;)
  {
    token = readToken();
    if(token >= firstNonCharToken || (!std::isalnum(static_cast<unsigned char>(token)) && token != dot && token != dash && token != underscore && token != colon))
    {
      undoReadToken(token);
      return true;
    }
    tmpString += static_cast<char>(token);
  }
}

bool Reader::readString(Location& location)
{
  Token token = readToken();
  if(token != doubleQuote)
  {
    if(token == endOfInput)
      handleError("Unexpected end of file", this->location);
    else if(tokenIsSpace(token))
      handleError("Unexpected whitespace", this->location);
    else
      handleError("A string must begin with a double quote character", this->location);
    return false;
  }
  const Location startQuoteLocation = this->location;
  bool wroteLocation = false;
  tmpString.clear();
  bool escaped = false;
  while((token = readToken()) != doubleQuote && !escaped)
  {
    if(token == endOfInput)
    {
      handleError("Unterminated string (there must be a closing double quote somewhere)", startQuoteLocation);
      return false;
    }
    if(!wroteLocation)
    {
      location = this->location;
      wroteLocation = true;
    }
    if(!escaped && token == backslash)
    {
      escaped = true;
      continue;
    }
    if(static_cast<int>(token) < firstNonCharToken)
      tmpString += static_cast<char>(token);
    else
      tmpString += nonCharTokenToString(token);
  }
  if(!wroteLocation)
    location = this->location;
  return true;
}

bool Reader::readData(bool callHandler)
{
  tmpString.clear();
  Token token = readToken();
  // This function is only called if the next token is not one of the ones below.
  ASSERT(token != endOfInput && token != tagStart && token != endTagStart);
  const Location dataLocation = location;
  if(static_cast<int>(token) < firstNonCharToken)
    tmpString += static_cast<char>(token);
  else
    tmpString += nonCharTokenToString(token);
  while((token = readToken()) != endTagStart && token != tagStart)
  {
    if(token == endOfInput)
    {
      handleError("Unterminated data block (there must be a tag somewhere)", dataLocation);
      return false;
    }
    if(static_cast<int>(token) < firstNonCharToken)
      tmpString += static_cast<char>(token);
    else
      tmpString += nonCharTokenToString(token);
  }
  undoReadToken(token);

  if(callHandler)
    handleText(tmpString, dataLocation);
  return true;
}

Reader::Token Reader::readToken()
{
  prevLocation = location;
  location = nextLocation;
  if(nextToken != invalidToken)
  {
    const Token token = nextToken;
    nextToken = invalidToken;
    nextLocation = nextNextLocation;
    return token;
  }
  const char c = static_cast<char>(reader.get());
  if(!reader.good())
    return endOfInput;
  ASSERT(c != EOF);
  if(c == '\n')
  {
    ++nextLocation.line;
    nextLocation.column = 1;
  }
  else if((c & 0xc0) != 0x80) // This handles UTF-8 continuation characters.
    ++nextLocation.column;
  if(c == '<')
  {
    char nextC = static_cast<char>(reader.peek());
    if(nextC == '/')
    {
      reader.get();
      ++nextLocation.column;
      return endTagStart;
    }
    else if(nextC == '!')
    {
      reader.get();
      if(reader.peek() == '-')
      {
        reader.get();
        if(reader.peek() == '-')
        {
          reader.get();
          nextLocation.column += 3;
          return commentStart;
        }
        reader.putback('-');
      }
      reader.putback('!');
    }
  }
  else if(c == '/')
  {
    if(reader.peek() == '>')
    {
      reader.get();
      ++nextLocation.column;
      return emptyTagEnd;
    }
  }
  else if(c == '-')
  {
    if(reader.peek() == '-')
    {
      reader.get();
      if(reader.peek() == '>')
      {
        reader.get();
        nextLocation.column += 2;
        return commentEnd;
      }
      reader.putback('-');
    }
  }
  return static_cast<Token>(c);
}

void Reader::undoReadToken(Token token)
{
  ASSERT(nextToken == invalidToken);
  nextToken = token;
  nextNextLocation = nextLocation;
  nextLocation = location;
  location = prevLocation;
}

std::string Reader::nonCharTokenToString(Token token)
{
  ASSERT(token >= firstNonCharToken);
  if(token == emptyTagEnd)
    return "/>";
  else if(token == endTagStart)
    return "</";
  else if(token == commentStart)
    return "<!--";
  else if(token == commentEnd)
    return "-->";
  ASSERT(false);
  return std::string();
}

bool Reader::tokenIsSpace(Token token)
{
  return token == ' ' || token == '\n' || token == '\r' || token == '\t';
}

void Reader::skipWhitespace(const char*& str, Location& loc)
{
  while(std::isspace(static_cast<unsigned char>(*str)))
  {
    if(*str == '\n')
    {
      ++loc.line;
      loc.column = 1;
    }
    else
      ++loc.column;
    ++str;
  }
}
