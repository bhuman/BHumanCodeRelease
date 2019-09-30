/**
 * This file declares a simple backend for the stream classes that work on
 * the "ConfigMap" format. It parses the following format and provides the data
 * as a simple syntax tree:
 *
 * map ::= record
 * record ::= field ';' { field ';' }
 * field ::= literal '=' ( literal | '{' record '}' | array )
 * array ::= '[' [ ( literal | '{' record '}' ) { ',' ( literal | '{' record '}' ) } [ ',' ] ']'
 * literal ::= '"' { anychar1 } '"' | { anychar2 }
 *
 * It can also parse a JSON-like format, which has the following grammar:
 *
 * map ::= record
 * record ::= '{' field { ',' field } [ ',' ] '}'
 * field ::= literal ':' ( literal | record | array )
 * array ::= '[' [ ( literal | record | array ) { ',' ( literal | record | array ) } [ ',' ] ']'
 * literal ::= '"' { anychar1 } '"' | { anychar2 }
 *
 * anychar1 must escape double quotes and backslash with a backslash
 * anychar2 cannot contain whitespace and other characters used by the grammar.
 *
 * @author Thomas Röfer
 */

#pragma once

#include <unordered_map>
#include "InOut.h"
#include "Enum.h"

/**
 * The class is simple backend for the stream classes that work on
 * the "ConfigMap" format.
 */
class SimpleMap
{
public:
  /**< Base class for syntax tree nodes. */
  class Value
  {
  public:
    virtual ~Value() = default;
  };

  /** A class representing a literal. */
  class Literal : public Value
  {
  private:
    std::string literal; /**< The literal. */
    mutable In* stream = nullptr; /**< A stream that can parse the literal. */

  public:
    Literal(const std::string& literal) : literal(literal) {}

    ~Literal()
    {
      if(stream)
        delete stream;
    }

    operator In&() const; /**< Returns a stream that can parse the literal. */
  };

  /** A class representing a record of attributes, i.e. a mapping of names to values. */
  class Record : public Value, public std::unordered_map<std::string, Value*>
  {
  public:
    ~Record();
  };

  /**< A class representing an array of values, i.e. a mapping of indices to values. */
  class Array : public Value, public std::vector<Value*>
  {
  public:
    ~Array();
  };

  /**
   * Constructor. Parses the stream.
   * @param stream The stream that is parsed according to the grammar given above.
   * @param name The name of the file if the stream is a file. Used for error messages.
   * @param jsonMode Whether the parser should be in JSON(-like) mode.
   */
  SimpleMap(In& stream, const std::string& name = "", bool jsonMode = false);

  /**
   * Destructor.
   */
  ~SimpleMap();

  operator const Value*() const {return root;} /**< Returns the root of the syntax tree. 0 if parsing failed. */

private:
  /** Lexicographical symbols. */
  ENUM(Symbol,
  {,
    literal, equals,
    comma, semicolon,
    lBracket, rBracket,
    lBrace, rBrace,
    eof,
  });

  In& stream; /**< The stream from which is read. */
  char c; /**< The current character. 0 means EOF reached. */
  int row; /**< The current row in the stream. */
  int column; /**< The current column in the stream. */
  Symbol symbol; /**< The current lexicographical symbol. */
  std::string string; /**< The string if the current symbol is "literal". */
  Value* root; /**< The root of the syntax tree. 0 if parsing failed. */
  const bool jsonMode; /**< Whether the parser should be in JSON(-like) mode. */

  void nextChar(); /**< Read the next character into "c". */
  void nextSymbol(); /**< Read the next symbol into "symbol". */
  void unexpectedSymbol(); /**< Throw an exception for an unexpected symbol. */

  /**
   * Read next symbol and check whether it is the expected one.
   * Throw an exception if not.
   * @param expected The symbol expected.
   */
  void expectSymbol(Symbol expected);
  Record* parseRecord(); /**< Parse a record. */
  Array* parseArray(); /**< Parse an array. */
};
