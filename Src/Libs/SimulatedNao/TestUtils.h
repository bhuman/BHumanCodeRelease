/**
 * @file SimulatedNao/TestUtils.h
 *
 * @brief Provides utility functions for managing test-related stuff.
 *
 * @author Adam Cihasev
 */

#pragma once
#include <string>
#include <filesystem>
#include <vector>

/**
 * @namespace TestUtils
 *
 * @brief Utility functions for managing test-related stuff.
 */
namespace TestUtils
{
  /**
   * @brief Generates the test directory path from the given scene path.
   *
   * @param scenePath The base directory path.
   * @return The generated test directory path.
   */
  std::filesystem::path generateTestDirPath(const std::string& scenePath);

  /**
   * @brief Creates a CRC32 hash of the given string without using a lookup table.
   *
   * @param str The string to hash.
   * @return The CRC32 hash of the string.
   */
  uint32_t crc32(const std::string& str);

  /**
   * @brief Gets the process ID of the current process.
   *
   * @return The process ID of the current process.
   */
  int getProcessID();

  /**
   * @brief Converts a boolean array to a string.
   *
   * @param boolArray The boolean array to convert.
   * @return The string representation of the boolean array.
   */
  std::string boolArrayToString(const std::vector<bool>& boolArray);

  /**
   * @brief Evaluates logical expressions with preset variables.
   */
  class ExpressionEvaluator
  {
  public:

    ENUM(ExpressionType,
    {,
      undefined,
      logicalOr,
      logicalAnd,
      logicalNot,
      literal,
    });

    STREAMABLE(LogicalExpression,
    {
      LogicalExpression() = default;
      LogicalExpression(const ExpressionType type) : type(type) { },

      (ExpressionType)(undefined) type,            /**< The type of the logical expression */
      (std::vector<LogicalExpression>) children,  /**< The children of the logical expression */
      (int)(-1) index,                            /**< The index of the preset variable */
    });

    enum EvaluationResult
    {
      unknown,    // The result is indeterminate.
      trueResult, // The result is true.
      falseResult // The result is false.
    };

    /**
     * @brief Evaluates the logical expression with the given preset variables.
     *
     * @param expr The logical expression to evaluate.
     * @param presetVars The preset variables to evaluate the expression with.
     * @param ignoreFalse Whether to ignore variables that are false.
     * @return The result of the evaluation.
     */
    static EvaluationResult evaluate(const LogicalExpression& expr, const std::vector<bool>& presetVars, const bool ignoreFalse = true);
  };
}
