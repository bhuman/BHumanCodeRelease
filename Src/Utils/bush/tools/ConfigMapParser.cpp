/**
* @file ConfigMapParser.cpp
*
* Implementation of a Lexer and a Parser which can read a ConfigMap from a file
* or an std InStream.
*
* @author <a href="mailto::ojlr@informatik.uni-bremen.de">Ole Jan Lars Riemann</a>
*/

#include <cstdio>
#include <cctype>
#include <iostream>
#include <sstream>
#include "ConfigMapParser.h"
#include "Tools/Debugging/Asserts.h"
#include "Utils/bush/tools/ConfigMap.h"
#include "Tools/Debugging/Debugging.h"

typedef ConfigMapLexer::Token T;
typedef ConfigMapLexer L;

const T TKN_EOF(T::CMT_EOF, 0, 0, "");

/*
 * ConfigMapLexer
 */

enum State_T
{
  NONE = 0,
  EOL_C,
  BLOCK_C,
  STRING,
  ESCAPE
};

#define MK_TOKEN(type, value) \
  ConfigMapLexer::Token(type, tknLine, tknColumn, value)


std::string T::type2str(Type t)
{
  switch(t)
  {
  case CMT_EOF:
    return "EOF";
  case CMT_KEY:
    return "KEY";
  case CMT_VALUE:
    return "VALUE";
  case CMT_COMMENT:
    return "COMMENT";
  case CMT_COMMA:
    return ",";
  case CMT_EQ:
    return "=";
  case CMT_SEMICOLON:
    return ";";
  case CMT_ABL:
    return "[";
  case CMT_ABR:
    return "]";
  case CMT_CBL:
    return "{";
  case CMT_CBR:
    return "}";
  default:
    return "UNKNOWN";
  }
}

ConfigMapLexer::ConfigMapLexer(const std::string& filename)
  : input(new std::ifstream(filename.c_str(), std::ifstream::in)),
    filename(new std::string(filename)),
    line(1),
    column(0),
    tkn(0)
{ }

ConfigMapLexer::~ConfigMapLexer()
{
  if(tkn) delete tkn;
  if(filename)
  {
    delete filename;
    if(input)
      delete input;
  }
}

size_t countNL(const std::string& str)
{
  size_t count = 0;
  for(std::string::const_iterator i = str.begin(); i != str.end(); ++i)
    if(*i == '\n') ++count;
  return count;
}

#define RETURN_TOKEN(t, type) \
  { \
    std::string s = value.str(); \
    value.str(""); \
    if (s.length() > 0) \
    { \
      ASSERT(!tkn); \
      tkn = new L::Token(t); \
      tkn->column += s.length(); \
      tkn->line += countNL(s); \
      return L::Token(type, tknLine, tknColumn, s);\
    } \
    else \
      return t; \
  }

ConfigMapLexer::Token ConfigMapLexer::nextToken()
{
  if(tkn)
  {
    Token t = *tkn;
    delete tkn;
    tkn = 0;
    return t;
  }

  std::stringstream value(std::stringstream::in | std::stringstream::out);
  int tknLine = line;
  int tknColumn = column + 1;

  State_T state = NONE;

  char l = EOF;

  while(true)
  {
    ++column;
    char c = (char) input->get();

    switch(c)
    {
    case EOF:
      if (state == EOL_C)
      {
        RETURN_TOKEN(MK_TOKEN(T::CMT_EOF, std::string(1, c)), T::CMT_COMMENT);
      }
      else
      {
        RETURN_TOKEN(MK_TOKEN(T::CMT_EOF, std::string(1, c)), T::CMT_VALUE);
      }
    case ' ':
      if(state == ESCAPE)
      {
        return MK_TOKEN(T::CMT_ERROR, std::string(1, c));
      }
      else if(state == NONE);
      else
      {
        value << c;
      }
      break;
    case '\r':
      break;
    case '\n':
      ++line;
      column = 1;
      switch(state)
      {
      case ESCAPE:
        return MK_TOKEN(T::CMT_ERROR, std::string(1, c));
      case EOL_C:                           // leave end of line comment
      {
        state = NONE;
        std::string s = value.str();
        value.str("");
        return MK_TOKEN(T::CMT_COMMENT, std::string(s));
      }
      default:
        value << c;
      case NONE:
        ;
      }
      break;
    case ',':
      if(state == STRING || state == EOL_C || state == BLOCK_C)
      {
        value << c;
        break;
      }
      else
        RETURN_TOKEN(MK_TOKEN(T::CMT_COMMA, std::string(1, c)), T::CMT_VALUE);
    case '=':
      if(state == STRING || state == EOL_C || state == BLOCK_C)
      {
        value << c;
        break;
      }
      else
        RETURN_TOKEN(MK_TOKEN(T::CMT_EQ, std::string(1, c)), T::CMT_KEY);
    case ';':
      if(state == STRING || state == EOL_C || state == BLOCK_C)
      {
        value << c;
        break;
      }
      else
        RETURN_TOKEN(MK_TOKEN(T::CMT_SEMICOLON, std::string(1, c)), T::CMT_VALUE);
    case '[':
      if(state == STRING || state == EOL_C || state == BLOCK_C)
      {
        value << c;
        break;
      }
      else
        return MK_TOKEN(T::CMT_ABL, std::string(1, c));
    case ']':
      if(state == STRING || state == EOL_C || state == BLOCK_C)
      {
        value << c;
        break;
      }
      else
        RETURN_TOKEN(MK_TOKEN(T::CMT_ABR, std::string(1, c)), T::CMT_VALUE);
    case '{':
      if(state == STRING || state == EOL_C || state == BLOCK_C)
      {
        value << c;
        break;
      }
      else
        return MK_TOKEN(T::CMT_CBL, std::string(1, c));
    case '}':
      if(state == STRING || state == EOL_C || state == BLOCK_C)
      {
        value << c;
        break;
      }
      else
        return MK_TOKEN(T::CMT_CBR, std::string(1, c));
    case '*':
      if(l == '/' && !state)                  // enter block comment
      {
        value.str("");
        state = BLOCK_C;
      }
      else
      {
        value << c;
      }
      break;
    case '/':
      if(l == '*' && (state == BLOCK_C))      // leave block comment
      {
        std::string s = value.str();
        value.str("");
        state = NONE;
        return MK_TOKEN(T::CMT_COMMENT, std::string(s, 0, s.length() - 1));
      }
      else if(l == '/' && state == NONE)      // enter end of line comment
      {
        value.str("");
        state = EOL_C;
      }
      else
      {
        value << c;
      }
      break;
    case '"':
      if(state == STRING)                     // leave quoted string
      {
        std::string s = value.str();
        value.str("");
        state = NONE;
        return MK_TOKEN(T::CMT_VALUE, std::string(s));
      }
      else if(state == NONE)                  // enter quoted string
      {
        tknLine = line;
        tknColumn = column;
        state = STRING;
      }
      else if(state == ESCAPE)
      {
        value << c;
        state = STRING;
      }
      else if (state == EOL_C || state == BLOCK_C)
      {
        value << c;
      }
      else
      {
        return MK_TOKEN(T::CMT_ERROR, std::string(1, c));
      }
      break;
    case '\\':
      if(state == STRING)                     // escape next character
      {
        state = ESCAPE;
      }
      else if(state == ESCAPE)
      {
        value << c;
        state = STRING;
      }
      else
      {
        return MK_TOKEN(T::CMT_ERROR, std::string(1, c));
      }
      break;
    default:
      if(state == STRING || state == EOL_C || state == BLOCK_C)
      {
        value << c;
      }
      else if(state == NONE)
      {
        if(isalnum(c) || c == '_' || c == '+' || c == '-' || c == '.')
        {
          value << c;
        }
        else
        {
          return MK_TOKEN(T::CMT_ERROR, std::string(1, c));
        }
      }
      else
      {
        return MK_TOKEN(T::CMT_ERROR, std::string(1, c));
      }
    }

    l = c;
  }
}

std::string ConfigMapLexer::getFilename() const
{
  return filename ? *filename : "(mem)";
}

bool ConfigMapLexer::isValid() const
{
  return !filename || input->good();
}

/*
 * ConfigMapParser
 */


ConfigMapParser::~ConfigMapParser()
{
  if(lexer) delete lexer;
  if (error) delete error;
}

void ConfigMapParser::setError(const ParseException& e)
{
  if (error) delete error;
  error = new ParseException(e);
}

void ConfigMapParser::nextToken()
{
  token = lexer->nextToken();
}

#define P_ERROR(msg) \
  if (token.type == T::CMT_ERROR) \
    throw ParseException(token.line, token.column, "Unexpected character: `" + token.value + "`");\
  else \
    throw ParseException(token.line, token.column, msg);

void ConfigMapParser::expect(T::Type type) throw(PARSEEXCEPTION_THROW)
{
  if(token.type == type)
    nextToken();
  else
    P_ERROR("Expected `" + T::type2str(type) + "` got `" + T::type2str(token.type) + "`");
}

bool prefixValue(const T& token)
{
  return token.type == T::CMT_VALUE
         || token.type == T::CMT_ABL
         || token.type == T::CMT_CBL;
}


int ConfigMapParser::file(ConfigMap* configMap) throw(PARSEEXCEPTION_THROW)
{
  size_t size = configMap->length();
  std::string* currComments = 0;
  if(token.type == T::CMT_COMMENT)
    currComments = comments();
  while(token.type == T::CMT_KEY)
  {
    size_t keyLine = token.line;
    size_t keyColumn = token.column;
    std::string key = token.value;
    nextToken();
    try
    {
      expect(T::CMT_EQ);
      if(prefixValue(token))
      {
        ConfigValue* cv = value();
        if(currComments)
          cv->setComment(*currComments);
        try
        {
          (*configMap)[key] = *cv;
        }
        catch(invalid_key& ik)
        {
          throw ParseException(keyLine, keyColumn, ik.what());
        }
        delete cv;
      }
      else
      {
        P_ERROR("Expected begin of a value (`{`, `[` or VALUE), got `" + T::type2str(token.type) + "`");
      }
    }
    catch(ParseException)
    {
      if(currComments) delete currComments;
      throw;
    }
    expect(T::CMT_SEMICOLON);

    if(currComments)
      delete currComments;
    currComments = 0;
    if(token.type == T::CMT_COMMENT)
      currComments = comments();
  }
  expect(T::CMT_EOF);
  if(currComments) delete currComments;
  return configMap->length() - size;
}

ConfigValue* ConfigMapParser::value() throw(PARSEEXCEPTION_THROW)
{
  if(token.type == T::CMT_CBL)
  {
    return map();
  }
  else if(token.type == T::CMT_ABL)
  {
    return list();
  }
  else
  {
    return plain();
  }
}

PlainConfigValue* ConfigMapParser::plain() throw(PARSEEXCEPTION_THROW)
{
  if(token.type == T::CMT_VALUE)
  {
    PlainConfigValue* pcv = new PlainConfigValue(token.value);
    nextToken();
    return pcv;
  }
  else
    P_ERROR("Expected a plain value, got `" + T::type2str(token.type) + "`");
}

ListConfigValue* ConfigMapParser::list() throw(PARSEEXCEPTION_THROW)
{
  expect(T::CMT_ABL);
  ListConfigValue* lcv = new ListConfigValue();
  std::string* currComments = 0;
  try
  {
    if(token.type == T::CMT_COMMENT)
      currComments = comments();
    if(prefixValue(token))
    {
      ConfigValue* cv = value();
      if(currComments)
      {
        cv->setComment(*currComments);
        delete currComments;
        currComments = 0;
      }
      lcv->append(*cv);
      delete cv;
      while(token.type == T::CMT_COMMA)
      {
        nextToken();
        if(token.type == T::CMT_COMMENT)
          currComments = comments();
        if(prefixValue(token))
        {
          ConfigValue* cv = value();
          if(currComments)
          {
            cv->setComment(*currComments);
            delete currComments;
            currComments = 0;
          }
          lcv->append(*cv);
          delete cv;
        }
        else
          break;
      }
    }
    expect(T::CMT_ABR);
  }
  catch(ParseException)
  {
    if(currComments)
      delete currComments;
    delete lcv;
    throw;
  }
  return lcv;
}

ConfigMap* ConfigMapParser::map() throw(PARSEEXCEPTION_THROW)
{
  expect(T::CMT_CBL);
  ConfigMap* cm = new ConfigMap();
  std::string* currComments = 0;
  if(token.type == T::CMT_COMMENT)
    currComments = comments();
  try
  {
    while(token.type == T::CMT_KEY)
    {
      std::string key = token.value;
      nextToken();
      try
      {
        expect(T::CMT_EQ);
        if(prefixValue(token))
        {
          ConfigValue* cv = value();
          if(currComments)
            cv->setComment(*currComments);
          try
          {
            (*cm)[key] = *cv;
          }
          catch(invalid_key& ik)
          {
            P_ERROR(ik.what());
          }
          delete cv;
        }
        else
        {
          P_ERROR("Expected begin of a value (`{`, `[` or VALUE), got `" + T::type2str(token.type) + "`");
        }
      }
      catch(ParseException)
      {
        if(currComments) delete currComments;
        throw;
      }
      expect(T::CMT_SEMICOLON);
    }
    expect(T::CMT_CBR);

    if(currComments)
      delete currComments;
    currComments = 0;
    if(token.type == T::CMT_COMMENT)
      currComments = comments();
  }
  catch(ParseException)
  {
    delete cm;
    if(currComments) delete currComments;
    throw;
  }
  if(currComments) delete currComments;
  return cm;
}

std::string* ConfigMapParser::comments() throw(PARSEEXCEPTION_THROW)
{
  std::stringstream comments;
  bool newline = false;
  while(token.type == T::CMT_COMMENT)
  {
    if(newline)
      comments << "\n";
    comments << token.value;
    newline = true;
    nextToken();
  }
  return new std::string(comments.str());
}

void ConfigMapParser::ParseException::printError(const std::string& filename) const
{
  OUTPUT_ERROR(filename << ":" << line << "," << column << ": " << msg);
}

int ConfigMapParser::parse(bool noThrow)
{
  int returnValue = 0;
  ASSERT(lexer);
  ASSERT(configMap);
  try
  {
    if(!lexer->isValid())
    {
      returnValue = ConfigMap::E_FILE;
      throw ParseException(0, 0, "Cannot read file.");
    }
    nextToken(); // get the first token
    return file(configMap);
  }
  catch(ParseException& pe)
  {
    // If the returnValue indicates not that the file connot be opened, there
    // has to be a syntax error.
    if (returnValue != ConfigMap::E_FILE)
      returnValue = ConfigMap::E_SYNTAX;

    if(verbose)
      pe.printError(lexer->getFilename());

    std::stringstream buf;
    buf << lexer->getFilename() << ":" << pe.line << "," << pe.column << ": " << pe.what();
    ParseException e = ParseException(pe.line, pe.column, buf.str());
    if (noThrow)
      setError(e);
    else
      throw e;
  }
  catch(invalid_key)
  {
    if(verbose)
      OUTPUT_ERROR("ConfigMapParser::parse catched a invalid_key exception.\n"
                   "This indicates, that there is a bug into the parser.");
    ASSERT(false);
  }
  catch(std::invalid_argument)
  {
    if(verbose)
      OUTPUT_ERROR("ConfigMapParser::parse catched a invalid_argument exception.\n"
                   "This indicates, that there is a bug into the parser.");
    ASSERT(false);
  }
  return returnValue;
}

int ConfigMapParser::parse(std::istream* in, bool noThrow)
{
  if(lexer) delete lexer;
  lexer = new ConfigMapLexer(in);
  return parse(noThrow);
}

int ConfigMapParser::parse(const std::string& file, bool noThrow)
{
  if(lexer) delete lexer;
  lexer = new ConfigMapLexer(file);
  return parse(noThrow);
}

int ConfigMapParser::parse(ConfigMap* map, const std::string& file, bool noThrow)
{
  configMap = map;
  if(lexer) delete lexer;
  lexer = new ConfigMapLexer(file);
  return parse(noThrow);
}

int ConfigMapParser::parse(ConfigMap* map, std::istream* in, bool noThrow)
{
  configMap = map;
  if(lexer) delete lexer;
  lexer = new ConfigMapLexer(in);
  return parse(noThrow);
}
