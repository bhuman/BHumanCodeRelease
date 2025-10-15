/**
 * @file SimulatedNao/TestCommandParser.cpp
 *
 * This file declares a class that parses commands for the test controller.
 *
 * @author Adam Cihasev
 */

#pragma once
#include "Streaming/InStreams.h"
#include "SimulatedNao/TestController.h"
#include "TestUtils.h"

enum TestCommandType
{
  help,
  stop,
  test,
  none,
};

class TestCommandParser
{
private:
  TestCommandType commandType = TestCommandType::none;
  TestController::TestParameters testParams;
  In& stream;
  std::string token;

  void parseGame();
  void parseSituation();
  void parseTeamNumbers();
  void parseTurn();
  void parseUserConfigPath();
  void parseFuzzingParams();
  void parseFlag();
  void parseTargets();

  TestController::TestTarget readTarget();
  TestController::TargetZoneType readTargetZoneType();
  TestController::TeamID readTeamID();
  TestController::TargetType readTargetType();
  SkillRequest::Type readSkillType();
  TestController::TargetZone readTargetZone();
  TestController::ComparisonOperator readComparisonOperator();
  TestController::ComparisonRule readComparisonRule();
  int readTeamNumber();

  TestUtils::ExpressionEvaluator::LogicalExpression readBaseExpression();
  TestUtils::ExpressionEvaluator::LogicalExpression readAndExpression();
  TestUtils::ExpressionEvaluator::LogicalExpression readOrExpression();

  int readPositiveInt();
  float readFloat();
  std::string readString();

  bool nextToken();
  void expectToken(const std::string& expected);
  void throwUnexpectedToken();

public:

  TestCommandParser(In& stream);
  ~TestCommandParser() = default;

  void parse();
  TestCommandType getCommandType() const;
  TestController::TestParameters getTestParameters() const;

  static bool isNumberString(const std::string& str);
  static bool isTargetType(const std::string& str);

  static std::string normalize(const std::string& cmd);
  static std::string padParentheses(const std::string& str);
  static std::string invertBudgetGreaterThan(const std::string& cmd);
};
