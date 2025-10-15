#include "TestUtils.h"
#include <string>
#include <vector>
#include <filesystem>
#include <format>
#include <QCoreApplication>
#include <QFileInfo>

std::filesystem::path TestUtils::generateTestDirPath(const std::string& scenePath)
{
  std::filesystem::path baseDir = std::filesystem::temp_directory_path() / "b-human";
  std::string normalScenePath = std::filesystem::path(scenePath).lexically_normal().string();
  std::string testID = std::format("{}:{}", normalScenePath, TestUtils::getProcessID());
  std::string testDir = std::format("test_{:08x}", crc32(testID));
  return baseDir / testDir;
}

uint32_t TestUtils::crc32(const std::string& str)
{
  uint32_t crc = 0xFFFFFFFF;
  for(char c : str)
  {
    crc ^= c;
    for(int i = 0; i < 8; i++)
    {
      crc = (crc >> 1) ^ (0xEDB88320 & ~((crc & 1) - 1));
    }
  }
  return ~crc;
}

int TestUtils::getProcessID()
{
  return static_cast<int>(QCoreApplication::applicationPid());
}

std::string TestUtils::boolArrayToString(const std::vector<bool>& boolArray)
{
  std::string str;
  for(bool b : boolArray)
  {
    str += b ? '1' : '0';
  }
  return str;
}

TestUtils::ExpressionEvaluator::EvaluationResult TestUtils::ExpressionEvaluator::evaluate(
  const TestUtils::ExpressionEvaluator::LogicalExpression& expr,
  const std::vector<bool>& presetVars,
  const bool ignoreFalse)
{
  switch(expr.type)
  {
    case ExpressionType::literal:
      if(presetVars[expr.index])
      {
        return EvaluationResult::trueResult;
      }
      return ignoreFalse ? EvaluationResult::unknown : EvaluationResult::falseResult;

    case ExpressionType::logicalNot:
      switch(evaluate(expr.children[0], presetVars, ignoreFalse))
      {
        case EvaluationResult::trueResult:
          return EvaluationResult::falseResult;
        case EvaluationResult::falseResult:
          return EvaluationResult::trueResult;
        default:
          return EvaluationResult::unknown;
      }

    case ExpressionType::logicalAnd:
    {
      bool hasUnknown = false;
      for(const auto& child : expr.children)
      {
        EvaluationResult childResult = evaluate(child, presetVars, ignoreFalse);
        if(childResult == EvaluationResult::falseResult)
        {
          return EvaluationResult::falseResult;
        }
        hasUnknown |= childResult == EvaluationResult::unknown;
      }
      return hasUnknown ? EvaluationResult::unknown : EvaluationResult::trueResult;
    }

    case ExpressionType::logicalOr:
    {
      bool hasUnknown = false;
      for(const auto& child : expr.children)
      {
        EvaluationResult result = evaluate(child, presetVars, ignoreFalse);
        if(result == EvaluationResult::trueResult)
        {
          return EvaluationResult::trueResult;
        }
        hasUnknown |= result == EvaluationResult::unknown;
      }
      return hasUnknown ? EvaluationResult::unknown : EvaluationResult::falseResult;
    }

    default:
      return EvaluationResult::unknown;
  }
}
