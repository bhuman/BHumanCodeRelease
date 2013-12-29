/**
* @file ConfigMapParser.h
*
* Declaration of a Lexer and a Parser which can read a ConfigMap from a file
* or an std InStream.
*
* @author <a href="mailto::ojlr@informatik.uni-bremen.de">Ole Jan Lars Riemann</a>
*/

#pragma once
#include <iosfwd>
#include <string>

#if defined WIN32
#define PARSEEXCEPTION_THROW ...
#else
#define PARSEEXCEPTION_THROW ParseException
#endif

class ConfigValue;
class PlainConfigValue;
class ListConfigValue;
class ConfigMap;

class ConfigMapLexer;
class ConfigMapParser;

class ConfigMapLexer
{
  std::istream* input;
  std::string* filename;

  int line;
  int column;

public:
  struct Token
  {
    enum Type
    {
      CMT_ERROR,
      CMT_EOF,
      CMT_KEY,
      CMT_VALUE,
      CMT_COMMENT,
      CMT_COMMA,     // ,
      CMT_EQ,        // =
      CMT_SEMICOLON, // ;
      CMT_ABL,       // [
      CMT_ABR,       // ]
      CMT_CBL,       // {
      CMT_CBR,       // }
    };
    static std::string type2str(Type t);
    Type type;
    int line;
    int column;
    std::string value;

    Token(Type t, int l, int c, std::string v)
      : type(t),
        line(l),
        column(c),
        value(v)
    { }
  };

private:
  Token* tkn;

public:
  ConfigMapLexer(std::istream* in)
    : input(in),
      filename(0),
      line(1),
      column(0),
      tkn(0)
  { }
  ConfigMapLexer(const std::string& filename);
  ~ConfigMapLexer();
  Token nextToken();
  std::string getFilename() const;
  bool isValid() const;
};

extern const ConfigMapLexer::Token TKN_EOF;

/**
 * @class ConfigMapParser
 *
 * Parses a ConfigMap with following grammar:
 *
 * file     ::= map
 *
 * value    ::= plain
 *            | '[' list ']'
 *            | '{' map '}'
 *
 * plain    ::= VALUE
 *
 * list     ::= [ comments value { ',' comments value } [ ',' ] ]
 *
 * map      ::= { comments key '=' value ';' }
 *
 * comments ::= { COMMENT }
 *
 */
class ConfigMapParser
{
public:
  class ParseException : public std::exception
  {
    unsigned line;
    unsigned column;
    std::string msg;

  public:
    ParseException(size_t line, size_t column, const std::string& msg) throw()
      : line(line),
        column(column),
        msg(msg)
    { }
    ~ParseException() throw() {};
    inline const char* what() const throw()
    {
      return msg.c_str();
    }
    void printError(const std::string& filename) const;

    friend class ConfigMapParser;
  };

private:
  ConfigMapLexer* lexer;
  ConfigMap* configMap;
  ConfigMapLexer::Token token;
  bool verbose;
  ParseException *error;
  void setError(const ParseException& e);
  void nextToken();
  void expect(ConfigMapLexer::Token::Type type) throw(PARSEEXCEPTION_THROW);
  int file(ConfigMap* configMap) throw(PARSEEXCEPTION_THROW);
  ListConfigValue* list() throw(PARSEEXCEPTION_THROW);
  ConfigMap* map() throw(PARSEEXCEPTION_THROW);
  PlainConfigValue* plain() throw(PARSEEXCEPTION_THROW);
  ConfigValue* value() throw(PARSEEXCEPTION_THROW);
  std::string* comments() throw(PARSEEXCEPTION_THROW);

public:
  ConfigMapParser()
    : lexer(0),
      configMap(0),
      token(TKN_EOF),
      verbose(false),
      error(0)
  { }
  ConfigMapParser(ConfigMap* map)
    : lexer(0),
      configMap(map),
      token(TKN_EOF),
      verbose(false),
      error(0)
  { }
  ConfigMapParser(ConfigMap* map, std::istream* in)
    : lexer(new ConfigMapLexer(in)),
      configMap(map),
      token(TKN_EOF),
      verbose(false),
      error(0)
  { }
  ConfigMapParser(ConfigMap* map, const std::string& file)
    : lexer(new ConfigMapLexer(file)),
      configMap(map),
      token(TKN_EOF),
      verbose(false),
      error(0)
  { }
  ~ConfigMapParser();

  inline ParseException* getError() const { return error; }

  /**
   * Toggles whether errors should be reported with OUTPUT_ERROR or not.
   * @param b Verbose or not
   */
  inline void setVerbose(bool b = true)
  {
    this->verbose = b;
  }

  /**
   * Reads a ConfigMap.
   * @param noThrow If set to true, no ParseException are thrown and errors are
   *                reported by the return value.
   * @throws A ConfigMapParser::ParseException if an error occures and noThrow
   *         is set to false.
   * @return The number of the readed keys or -1 if an error occures.
   */
  int parse(bool noThrow = true);
  int parse(std::istream* in, bool noThrow = true);
  int parse(const std::string& file, bool noThrow = true);
  int parse(ConfigMap* map, std::istream* in, bool noThrow = true);
  int parse(ConfigMap* map, const std::string& file, bool noThrow = true);
};
