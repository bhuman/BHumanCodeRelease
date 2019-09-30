/**
* @file Simulation/Reader.h
* Declaration of class Reader
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

#pragma once

#include <fstream>
#include <string>
#include <unordered_map>

/**
* @class Reader
* A parser for the above language
*/
class Reader
{
protected:
  struct Location
  {
    Location(int line = 0, int column = 0) : line(line), column(column) {}
    int line; /**< The line number */
    int column; /**< The column */
  };

  struct Attribute
  {
    Attribute(const std::string& value, int index, const Location& nameLocation, const Location& valueLocation) :
      value(value), index(index), nameLocation(nameLocation), valueLocation(valueLocation) {}

    /**
     * Copies an attribute with a new index
     * @param other The other attribute
     * @param index The new index
     */
    Attribute(const Attribute& other, int index) :
      value(other.value), index(index), nameLocation(other.nameLocation), valueLocation(other.valueLocation) {}

    std::string value; /**< The value of the attribute (i.e. what is in between the quotes) */
    int index; /**< The index in the list of attributes of its containing tag */
    Location nameLocation; /**< The location of the attribute name */
    Location valueLocation; /**< The location of the attribute value (first character after quote) */
  };

  using Attributes = std::unordered_map<std::string, Attribute>;

  std::string fileName; /**< The current file name */

  /**
   * Reads a file and calls the handlers of the derived class
   * This function is reentrant from handleElement.
   * @param fileName The name of the file
   * @return Whether the file was read without errors
   */
  bool readFile(const std::string& fileName);

  /**
   * Reads the subordinate elements of the current element (if any)
   * @param callHandler Whether handler functions (\c handleElement or \c handleText) should be called. Set it to \c false to pass the children of unexpected elements.
   * @param isRoot Whether this is the root syntax tree node of the file
   * @return Whether an error occured
   */
  bool readElements(bool callHandler, bool isRoot = false);

  /**
   * A handler called for syntax errors
   * @param msg An error message
   * @param location The location of the error in the current file
   */
  virtual void handleError(const std::string& msg, const Location& location) = 0;

  /**
   * A handler called for each parsed element
   * Call \c readElements within the handler to parse subordinate elements.
   * @param name The name of the element
   * @param attributes The attributes of the element
   * @param location The location of the element in the current file (first character after <)
   * @return The result of \c readElements that has been called inside
   */
  virtual bool handleElement(const std::string& name, Attributes& attributes, const Location& location) = 0;

  /**
   * A handler called when text was parsed
   * @param text The text read
   * @param location The location of the text in the current file
   */
  virtual void handleText(std::string& text, const Location& location) = 0;

  /**
   * Advances a string pointer and accordingly a location until a non-whitespace character is found
   * @param str The string pointer
   * @param location The location
   */
  static void skipWhitespace(const char*& str, Location& location);

private:
  /**
   * Represents a (very low level) token of the language
   * The values from 0 to 255 have their normal UTF-8 character meaning.
   * Values above 255 represent tokens that do not correspond to a single character.
   */
  enum Token : int
  {
    invalidToken = -1,
    endOfInput = 0,
    doubleQuote = '"',
    dash = '-',
    dot = '.',
    colon = ':',
    tagStart = '<',
    equals = '=',
    tagEnd = '>',
    backslash = '\\',
    underscore = '_',
    firstNonCharToken = 256,
    emptyTagEnd = firstNonCharToken, // />
    endTagStart, // </
    commentStart, // <!--
    commentEnd // -->
  };

  /**
   * Reads an element from the stream
   * @param callHandler Whether handlers for subordinate elements should be called
   * @return Whether an element could be read
   */
  bool readElement(bool callHandler);

  /**
   * Reads all attributes within a tag from the stream and stores them in \c attributes
   * @return Whether the attributes could be read
   */
  bool readAttributes();

  /**
   * Reads an attribute from the stream and stores it in \c attributes
   * @return Whether an attribute could be read
   */
  bool readAttribute();

  /**
   * Reads a name from the stream and writes it to \c tmpString
   * @param location Is filled with the location of the name
   * @return Whether a name could be read
   */
  bool readName(Location& location);

  /**
   * Reads a string from the stream and writes it to \c tmpString
   * @param location Is filled with the location of the first character after the quote
   * @return Whether a string could be read
   */
  bool readString(Location& location);

  /**
   * Reads arbitrary text until the next tag is encountered
   * @param callHandler Whether the handler of the derived class should be called
   * @return Whether data could be read
   */
  bool readData(bool callHandler);

  /**
   * Reads a token from the input stream and sets the current location accordingly
   * @return The token that has been read
   */
  Token readToken();

  /**
   * Puts an already read token back
   * readToken must be called in between calls to this method and before this method is called after readFile has been entered.
   * @param token The read token
   */
  void undoReadToken(Token token);

  /**
   * Returns the character sequence that corresponds to a token
   * @param token The token to convert
   * @return The converted string
   */
  static std::string nonCharTokenToString(Token token);

  /**
   * Custom version of isspace that is independent of the C locale and supports values that do not fit into an unsigned char.
   * @param token The token to test
   * @return Whether the token represents a whitespace character
   */
  static bool tokenIsSpace(Token token);

  std::ifstream reader; /**< The stream from which to read */
  Location location; /**< The location of the last token returned by readToken */
  Location nextLocation; /**< The location where the next token starts */
  Token nextToken = invalidToken; /**< The token that was put back (not saved over reentrant \c readFile calls) */
  Location prevLocation; /**< The location before the last call to readToken (needed by \c undoReadToken) */
  Location nextNextLocation; /**< The next location after the next call to readToken (filled by \c undoReadToken) */
  std::string tmpString; /**< A string used by readName, readString and readData (not saved over reentrant \c readFile calls) */
  Attributes attributes; /**< A storage for the attributes of an element (not saved over reentrant \c readFile calls) */
};
