/**
 * @file InExpr.h
 *
 * This file declares a stream for configuration files that allows
 * arithmetic expressions (including predefined symbols) as values
 * (currently only for floats).
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Streaming/InStreams.h"
#include <unordered_map>

class InExprMemory : public InTextMemory
{
public:
  /**
   * Constructor.
   * @param mem The address of the memory block from which is read.
   * @param size The size of the memory block. It is only used to
   *             implement the function eof().
   * @param symbols The map of symbols. Must live longer than this stream.
   */
  InExprMemory(const void* mem, std::size_t size, const std::unordered_map<std::string, float>& symbols) :
    InTextMemory(mem, size),
    symbols(symbols)
  {}

protected:
  void inFloat(float& value) override;

private:
  /**
   * Reads an expression (which is a sum/difference of terms).
   * @param ptr Pointer to the start of the expression.
   * @return The value of the expression.
   */
  float readExpr(const char*& ptr);

  /**
   * Reads a term (which is a product/quotient of factors).
   * @param ptr Pointer to the start of the term.
   * @return The value of the term.
   */
  float readTerm(const char*& ptr);

  /**
   * Reads a factor (which is a potentially negated literal)
   * @param ptr Pointer to the start of the factor.
   * @return The value of the factor.
   */
  float readFactor(const char*& ptr);

  /**
   * Reads a literal (which can be a bracketed expression, a number or an identifier).
   * @param ptr Pointer to the start of the literal.
   * @return The value of the literal.
   */
  float readLiteral(const char*& ptr);

  const std::unordered_map<std::string, float>& symbols; /**< The map of symbols. Must live longer than this stream. */
};

class InExprMapFile : public InMapFile
{
public:
  /**
   * Constructor.
   * @param name The name of the config file to read.
   * @param symbols The map of known symbols.
   * @param errorMask The kinds of error messages to show if specification does not match.
   */
  InExprMapFile(const std::string& name, const std::unordered_map<std::string, float>& symbols, unsigned errorMask = 0u);

protected:
  /**
   * Returns a new stream from which a literal can be read.
   * @param literal The string from which to create the stream.
   * @return A pointer to the new stream (must be freed by the caller).
   */
  In* createLiteralStream(const std::string& literal) override;

private:
  std::unordered_map<std::string, float> symbols; /**< The map of known symbols for each literal. */
};
