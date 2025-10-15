/**
 * @file SimulatedNao/TestCommandParser.cpp
 *
 * This file implements a class that parses commands for the test controller.
 *
 * @author Nico Holsten
 * @author Adam Cihasev
 */

#include "TestCommandParser.h"
#include "Streaming/InStreams.h"
#include "Streaming/OutStreams.h"
#include "Platform/File.h"
#include <string>
#include <map>

using TestUtils::ExpressionEvaluator;

TestCommandType TestCommandParser::getCommandType() const
{
  return commandType;
}

TestController::TestParameters TestCommandParser::getTestParameters() const
{
  return testParams;
}

int TestCommandParser::readPositiveInt()
{
  try
  {
    int value = std::stoi(token);
    if(value < 0)
    {
      throwUnexpectedToken();
    }
    nextToken();
    return value;
  }
  catch(const std::exception&)  // invalid_argument, out_of_range
  {
    throwUnexpectedToken();
    return 0;
  }
}

float TestCommandParser::readFloat()
{
  try
  {
    float value = std::stof(token);
    nextToken();
    return value;
  }
  catch(const std::exception&)
  {
    throwUnexpectedToken();
  }
  return 0;
}

std::string TestCommandParser::readString()
{
  if(token.empty())
  {
    throwUnexpectedToken();
  }
  std::string value = std::move(token);
  nextToken();
  return value;
}

TestController::TargetType TestCommandParser::readTargetType()
{
  const int rawType = TypeRegistry::getEnumValue(typeid(TestController::TargetType).name(), token);
  if(rawType < 0)
  {
    throwUnexpectedToken();
    return TestController::TargetType::noTarget;
  }
  else
  {
    nextToken();
    return static_cast<TestController::TargetType>(rawType);
  }
}

SkillRequest::Type TestCommandParser::readSkillType()
{
  const int rawType = TypeRegistry::getEnumValue(typeid(SkillRequest::Type).name(), token);
  if(rawType < 0)
  {
    throwUnexpectedToken();
    return SkillRequest::Type::none;
  }
  else
  {
    nextToken();
    return static_cast<SkillRequest::Type>(rawType);
  }
}

TestController::ComparisonOperator TestCommandParser::readComparisonOperator()
{
  static const std::map<std::string, TestController::ComparisonOperator> operatorMap =
  {
    {">",  TestController::greaterThan},
    {"<",  TestController::lessThan},
    {">=", TestController::greaterEqual},
    {"<=", TestController::lessEqual}
  };
  auto it = operatorMap.find(token);
  if(it != operatorMap.end())
  {
    nextToken();
    return it->second;
  }
  else
  {
    throwUnexpectedToken();
    return TestController::lessThan; // Unreachable, but avoids compiler warning
  }
}

TestController::ComparisonRule TestCommandParser::readComparisonRule()
{
  TestController::ComparisonRule rule;
  rule.op = readComparisonOperator();
  rule.value = readPositiveInt();
  return rule;
}

void TestCommandParser::parseTurn()
{
  expectToken("trn");
  testParams.turn = readTeamID();
}

void TestCommandParser::parseSituation()
{
  expectToken("situation");

  testParams.testType = TestController::situation;
  testParams.numOfTestRuns = readPositiveInt();
  testParams.secsToPlay = std::min(readPositiveInt() + 10, 600);
  parseTargets();

  // Optional/position-independent parameters
  while(!token.empty())
  {
    if(token == "tnm")
    {
      parseTeamNumbers();
    }
    else if(token == "trn")
    {
      parseTurn();
    }
    else if(token == "con")
    {
      parseUserConfigPath();
    }
    else if(token == "fuz")
    {
      parseFuzzingParams();
    }
    else
    {
      parseFlag();
    }
  }
}

void TestCommandParser::parseTeamNumbers()
{
  expectToken("tnm");

  testParams.teamNumbers[0] = readTeamNumber();
  testParams.teamNumbers[1] = readTeamNumber();
}

void TestCommandParser::parseGame()
{
  expectToken("game");

  testParams.testType = TestController::game;
  testParams.numOfTestRuns = readPositiveInt();
  testParams.secsToPlay = 600;
  testParams.targets.push_back(TestController::TestTarget());

  while(!token.empty())
  {
    if(token == "tnm")
    {
      parseTeamNumbers();
    }
    else if(token == "trn")
    {
      parseTurn();
    }
    else if(token == "con")
    {
      parseUserConfigPath();
    }
    else
    {
      parseFlag();
    }
  }
}

void TestCommandParser::parseUserConfigPath()
{
  expectToken("con");

  if(token.ends_with(".con"))
  {
    if(File::isAbsolute(token))
    {
      testParams.conFilePath = token;
    }
    else
    {
      testParams.conFilePath = std::string(File::getBHDir()) + "/Config/Scenes/" + token;
    }
    nextToken();
  }
  else
  {
    throwUnexpectedToken();
  }
}

void TestCommandParser::parseFuzzingParams()
{
  expectToken("fuz");

  testParams.fuzzing = true;
  TestController::FuzzingParams& fuzzingParams = testParams.fuzzingParams;
  int count = 0;

  while(true)
  {
    if(token == "all")
    {
      nextToken();
      float deviation = readFloat();
      fuzzingParams.pos.fill(deviation);
      fuzzingParams.rot.fill(deviation);
    }
    else if(token == "pos")
    {
      nextToken();
      fuzzingParams.pos.fill(readFloat());
    }
    else if(token == "rot")
    {
      nextToken();
      fuzzingParams.rot.fill(readFloat());
    }
    else if(token == "x")
    {
      nextToken();
      fuzzingParams.pos.x() = readFloat();
    }
    else if(token == "y")
    {
      nextToken();
      fuzzingParams.pos.y() = readFloat();
    }
    else if(token == "z")
    {
      nextToken();
      fuzzingParams.pos.z() = readFloat();
    }
    else if(token == "rotx")
    {
      nextToken();
      fuzzingParams.rot.x() = readFloat();
    }
    else if(token == "roty")
    {
      nextToken();
      fuzzingParams.rot.y() = readFloat();
    }
    else if(token == "rotz")
    {
      nextToken();
      fuzzingParams.rot.z() = readFloat();
    }
    else
    {
      break;  // Exit loop on begin of the next option
    }
    ++count;
  }

  // Ensure at least one parameter was set
  if(count == 0)
  {
    throwUnexpectedToken();
  }
}

void TestCommandParser::parseFlag()
{
  if(token == "rt")
  {
    testParams.realTime = true;
    nextToken();
  }
  else if(token == "q")
  {
    testParams.quit = true;
    nextToken();
  }
  else
  {
    throwUnexpectedToken();
  }
}

void TestCommandParser::parseTargets()
{
  testParams.targetsExpr = readOrExpression();

  // At least one target must be specified
  if(testParams.targetsExpr.type == ExpressionEvaluator::ExpressionType::undefined)
  {
    throwUnexpectedToken();
  }
}

ExpressionEvaluator::LogicalExpression TestCommandParser::readBaseExpression()
{
  ExpressionEvaluator::LogicalExpression expr;
  if(token == "(")
  {
    nextToken();
    expr = readOrExpression();
    expectToken(")");
  }
  else if(token == "not")
  {
    nextToken();
    expr.type = ExpressionEvaluator::ExpressionType::logicalNot;
    expr.children.push_back(readBaseExpression());
  }
  else if(isTargetType(token))
  {
    testParams.targets.push_back(readTarget());
    expr.type = ExpressionEvaluator::ExpressionType::literal;
    expr.index = static_cast<int>(testParams.targets.size()) - 1;
  }
  return expr;
}

ExpressionEvaluator::LogicalExpression TestCommandParser::readAndExpression()
{
  ExpressionEvaluator::LogicalExpression expr = readBaseExpression();
  while(token == "&")
  {
    nextToken();
    ExpressionEvaluator::LogicalExpression conj(ExpressionEvaluator::ExpressionType::logicalAnd);
    conj.children.push_back(expr);
    conj.children.push_back(readBaseExpression());
    expr = conj;
  }
  return expr;
}

ExpressionEvaluator::LogicalExpression TestCommandParser::readOrExpression()
{
  ExpressionEvaluator::LogicalExpression expr = readAndExpression();
  while(token == "|")
  {
    nextToken();
    ExpressionEvaluator::LogicalExpression disj(ExpressionEvaluator::ExpressionType::logicalOr);
    disj.children.push_back(expr);
    disj.children.push_back(readAndExpression());
    expr = disj;
  }
  return expr;
}

TestController::TestTarget TestCommandParser::readTarget()
{
  TestController::TestTarget target;
  target.type = readTargetType();
  switch(target.type)
  {
    case TestController::goalTeamB:
    case TestController::goalTeamA:
      break;
    case TestController::ballInZone:
      target.zone = readTargetZone();
      break;
    case TestController::robotInZone:
      target.zone = readTargetZone();
      target.robotName = "RoboCup.robots." + readString();
      break;
    case TestController::robotTouch:
      target.robotName = "RoboCup.robots." + readString();
      break;
    case TestController::penalized:
      target.robotName = "RoboCup.robots." + readString();
      break;
    case TestController::teamPossession:
    {
      target.teamID = readTeamID();
      break;
    }
    case TestController::robotInSkill:
      target.skill = readSkillType();
      target.robotName = "RoboCup.robots." + readString();
      break;
    case TestController::budget:
      target.teamID = readTeamID();
      target.budgetRule = readComparisonRule();
      break;
    default:
      throwUnexpectedToken();
  }
  return target;
}

TestController::TargetZoneType TestCommandParser::readTargetZoneType()
{
  const int rawType = TypeRegistry::getEnumValue(typeid(TestController::TargetZoneType).name(), token);
  if(rawType < 0)
  {
    throwUnexpectedToken();
    return TestController::TargetZoneType::customRect;
  }
  else
  {
    nextToken();
    return static_cast<TestController::TargetZoneType>(rawType);
  }
}

TestController::TeamID TestCommandParser::readTeamID()
{
  const int rawType = TypeRegistry::getEnumValue(typeid(TestController::TeamID).name(), token);

  // noTeam(0) is not a valid team ID
  if(rawType < 1)
  {
    throwUnexpectedToken();
    return TestController::TeamID::noTeam;
  }
  else
  {
    nextToken();
    return static_cast<TestController::TeamID>(rawType);
  }
}

TestController::TargetZone TestCommandParser::readTargetZone()
{
  TestController::TargetZone targetZone;
  if(isNumberString(token))
  {
    targetZone.type = TestController::TargetZoneType::customRect;
    targetZone.topLeft << readFloat(), readFloat();
    targetZone.bottomRight << readFloat(), readFloat();
  }
  else
  {
    if(token.ends_with("A"))
    {
      targetZone.side = TestController::TargetZoneSide::sideA;
      token.pop_back(); // Remove the trailing 'A'
    }
    else if(token.ends_with("B"))
    {
      targetZone.side = TestController::TargetZoneSide::sideB;
      token.pop_back();
    }
    else
    {
      targetZone.side = TestController::TargetZoneSide::any;
    }
    targetZone.type = readTargetZoneType();
  }
  return targetZone;
}

int TestCommandParser::readTeamNumber()
{
  try
  {
    int number = std::stoi(token);
    if(number < 0 || number > 255)
    {
      throwUnexpectedToken();
    }
    nextToken();
    return number;
  }
  catch(const std::exception&)
  {
    throwUnexpectedToken();
    return 0;
  }
}

bool TestCommandParser::nextToken()
{
  if(stream.eof())
  {
    if(!token.empty())
    {
      token = "";
    }
    return false;
  }
  stream >> token;
  return !token.empty();
}

void TestCommandParser::expectToken(const std::string& expected)
{
  if(token != expected)
  {
    throwUnexpectedToken();
  }
  nextToken();
}

void TestCommandParser::throwUnexpectedToken()
{
  throw std::runtime_error("Unexpected token: '" + (token.empty() ? "EOF" : token) + "'");
}

TestCommandParser::TestCommandParser(In& stream) : stream(stream) {}

void TestCommandParser::parse()
{
  nextToken();  // Read the first token
  expectToken("test");

  if(token == "help")
  {
    nextToken();
    commandType = TestCommandType::help;
  }
  else if(token == "stop")
  {
    nextToken();
    commandType = TestCommandType::stop;
  }
  else if(token == "situation")
  {
    commandType = TestCommandType::test;
    parseSituation();
  }
  else if(token == "game")
  {
    commandType = TestCommandType::test;
    parseGame();
  }
  else
  {
    throwUnexpectedToken();
  }
  expectToken("");
}

bool TestCommandParser::isNumberString(const std::string& str)
{
  if(str.empty())
  {
    return false;
  }
  size_t i = (str[0] == '-' || str[0] == '+') ? 1 : 0;  // Skip sign
  bool hasDecimal = false;
  bool hasDigits = false;

  for(; i < str.length(); i++)
  {
    if(std::isdigit(str[i]))
    {
      hasDigits = true;
    }
    else if(str[i] == '.' && !hasDecimal)
    {
      hasDecimal = true;
    }
    else
    {
      return false;
    }
  }
  return hasDigits;  // Ensure at least one digit is present
}

bool TestCommandParser::isTargetType(const std::string& str)
{
  return TypeRegistry::getEnumValue(typeid(TestController::TargetType).name(), str) >= 0;
}

std::string TestCommandParser::normalize(const std::string& cmd)
{
  return invertBudgetGreaterThan(padParentheses(cmd));
}

std::string TestCommandParser::padParentheses(const std::string& input)
{
  std::string result;
  result.reserve(input.size() + 10);  // Reserve some extra space for efficiency

  for(size_t i = 0; i < input.size(); ++i)
  {
    char c = input[i];
    if(c == '(')
    {
      // Add '(' first
      result.push_back(c);

      // If next char exists and is NOT whitespace, add a space
      if(i + 1 < input.size() && input[i + 1] != ' ' && input[i + 1] != '\t')
      {
        result.push_back(' ');
      }
    }
    else if(c == ')')
    {
      // If previous char exists and is NOT whitespace, add a space before ')'
      if(!result.empty() && result.back() != ' ' && result.back() != '\t')
      {
        result.push_back(' ');
      }
      result.push_back(c);
    }
    else
    {
      // Normal char, just copy
      result.push_back(c);
    }
  }
  return result;
}

std::string TestCommandParser::invertBudgetGreaterThan(const std::string& cmd)
{
  InConfigMemory in(cmd.c_str(), cmd.size());
  OutTextMemory out(cmd.size() + 64);

  std::string token;
  in >> token;

  const std::string budgetToken = TypeRegistry::getEnumName(TestController::TargetType::budget);

  while(!token.empty())
  {
    if((token == budgetToken))
    {
      std::string team, op, val;
      in >> team >> op >> val;

      std::string negOp;
      if((op == ">"))
      {
        negOp = "<=";
      }
      else if((op == ">="))
      {
        negOp = "<";
      }

      if(!negOp.empty())
      {
        out << "not" << "(" << budgetToken << team << negOp << val << ")";
      }
      else
      {
        out << budgetToken << team << op << val;
      }
    }
    else
    {
      out << token;
    }
    in >> token;
  }
  return out.data();
}
