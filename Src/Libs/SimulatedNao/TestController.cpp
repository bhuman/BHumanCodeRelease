/**
 * @file SimulatedNao/TestController.cpp
 *
 * This file implements a class that is responsible for
 * controlling the automatic test execution.
 *
 * @author Nico Holsten
 * @author Adam Cihasev
 */

#include "TestController.h"
#include "Framework/Settings.h"
#include "RoboCupCtrl.h"
#include "SimRobot.h"
#include "Streaming/Global.h"
#include "Streaming/InStreams.h"
#include "Streaming/OutStreams.h"
#include "TestUtils.h"
#include <format>
#include <fstream>
#include <regex>
#ifdef MACOS
#include <MainWindow.h>
#include <QApplication>
#include <QTimer>
#endif

using TestUtils::ExpressionEvaluator;

bool TestController::init(const QString& scenePath, TestController::TestParameters& inputParams)
{
  SYNC;
  if(!initTestDir())
  {
    return false;
  }

  // Copy current config file to the test directory and remove specific lines
  const std::filesystem::path testConPath = getPrimaryConfigPath();
  const std::filesystem::path origConPath = std::filesystem::path(scenePath.toStdString()).replace_extension("con");
  static std::regex matchSpaces("([^\\\\]) ");
  const std::string conPath = std::regex_replace(origConPath.parent_path().string(), matchSpaces, "$1\\ ");

  std::ifstream origConFile(origConPath, std::ios::in);
  std::ofstream testConFile(testConPath, std::ios::trunc);

  // Open the test configuration file, copy original, and append new configuration
  if(!origConFile.is_open() || !testConFile.is_open())
  {
    return false;
  }

  std::string line;
  while(std::getline(origConFile, line))
  {
    static std::regex matchCall("^[ \\t]*call[ \\t]*([^/\\\\])");
    testConFile << std::regex_replace(line, matchCall, "call " + conPath + "/$1") << std::endl;
  }

  if(inputParams.testType == situation)
  {
    testConFile << "gc playing" << std::endl;
  }
  else
  {
    testConFile << "gc ready" << std::endl;
  }

  testConFile << "dr representation:SkillRequest" << std::endl;

  if(!inputParams.realTime)
  {
    testConFile << "dt off" << std::endl;
  }

  TestState newState;
  newState.currTestRuns = 1;
  newState.status = TestStatus::resetting;

  OutMapFile(getStateFilePath(), true) << newState;
  OutMapFile(getParametersPath(), true) << inputParams;
  return true;
}

bool TestController::loadTest(const FieldDimensions& /*fieldDims*/)
{
  SYNC;
  InMapFile stateFile(getStateFilePath());
  InMapFile paramsFile(getParametersPath());

  if(!stateFile.exists() || !paramsFile.exists())
  {
    return false;
  }

  TestController::TestState loadedTestState;
  TestController::TestParameters loadedTestParams;

  stateFile >> loadedTestState;
  paramsFile >> loadedTestParams;

  if(loadedTestState.status != TestStatus::resetting)
  {
    // We only load states explicitly saved for resumption.
    // Other states are considered remnants from previous, unrelated tests.
    return false;
  }
  else
  {
    testState = loadedTestState;
    testParams = loadedTestParams;
    testState.status = TestStatus::running;
    saveTestState();
  }

  std::string modifiedString = "";

  // Write information into console
  if(testParams.testType == situation)
  {
    message += std::format("{} / {} test runs completed\n"
                           "{} / {} test runs successful\n",
                           testState.currTestRuns - 1, testParams.numOfTestRuns, testState.numOfSuccess, testParams.numOfTestRuns);
  }
  else if(testParams.testType == game)
  {
    message += std::format("{} / {} test runs completed\n", testState.currTestRuns - 1, testParams.numOfTestRuns);

    int winsTeamA = 0, winsTeamB = 0;
    int totalScoreA = 0, totalScoreB = 0;
    int draws = 0;

    for(const auto& halftime : testState.halftimes)
    {
      int scoreA = halftime.scores[0];
      int scoreB = halftime.scores[1];
      if(scoreA > scoreB)
      {
        ++winsTeamA;
      }
      else if(scoreA < scoreB)
      {
        ++winsTeamB;
      }
      else
      {
        ++draws;
      }
      totalScoreA += scoreA;
      totalScoreB += scoreB;
    }

    message += std::format("Total score: {}:{}\n"
                           "Wins Team A: {}\n"
                           "Wins Team B: {}\n"
                           "Draws: {}\n",
                           totalScoreA, totalScoreB, winsTeamA, winsTeamB, draws);
  }

  // Add fuzzing for the move parameters
  if(isFuzzingApplicable())
  {
    std::ifstream userConFile(testParams.conFilePath, std::ios::in);
    std::ofstream fuzzedConFile(getFuzzedConfigPath(), std::ios::trunc);

    if(!userConFile.is_open() || !fuzzedConFile.is_open())
    {
      return false;
    }

    auto isMvoLine = [](const std::string& line) -> bool
    {
      size_t pos = line.find_first_not_of(" \t");
      return pos != std::string::npos && line.compare(pos, 3, "mvo") == 0;
    };

    auto calcMaxDeviation = [](float pos, float maxDim, float deviation) -> float
    {
      return std::min(deviation, std::min(maxDim - pos, pos + maxDim));
    };

    std::string line;
    while(std::getline(userConFile, line))
    {
      // If the line does not start with "mvo", skip it
      if(!isMvoLine(line))
      {
        fuzzedConFile << line << std::endl;
        continue;
      }

      // Parse the command parameters
      InConfigMemory lineIn(line.c_str(), line.size());
      std::string mvo;
      std::string objectID;
      Vector3f position, rotation;

      lineIn >> mvo >> objectID;
      lineIn >> position.x() >> position.y() >> position.z();

      // Apply fuzzing
      FuzzingParams& fuzzingParams = testParams.fuzzingParams;

      // Calculate the maximum deviation for the position (Field dimensions: 9000 x 6000)
      // TODO: Uncomment when field dimensions availability is fixed
      // float maxPosXDev = calcMaxDeviation(position.x(), std::abs(fieldDims.xPosOwnGoalLine), fuzzingParams.pos.x());
      // float maxPosYDev = calcMaxDeviation(position.y(), std::abs(fieldDims.yPosLeftTouchline), fuzzingParams.pos.y());
      float maxPosXDev = calcMaxDeviation(position.x(), 4500.0f, fuzzingParams.pos.x());
      float maxPosYDev = calcMaxDeviation(position.y(), 3000.0f, fuzzingParams.pos.y());

      position.x() += Random::uniform(-maxPosXDev, maxPosXDev);
      position.y() += Random::uniform(-maxPosYDev, maxPosYDev);
      position.z() += Random::uniform(-fuzzingParams.pos.z(), fuzzingParams.pos.z());

      // Convert the position back to the string
      OutTextMemory modLineOut(256);
      modLineOut << mvo << objectID;
      modLineOut << position.x() << position.y() << position.z();

      // Handle optional rotation
      for(int i = 0; i < rotation.size() && !lineIn.eof(); ++i)
      {
        lineIn >> rotation[i];
        rotation[i] += Random::uniform(-fuzzingParams.rot[i], fuzzingParams.rot[i]);
        modLineOut << rotation[i];
      }

      // Write the modified line to the new con file
      fuzzedConFile << modLineOut.data() << std::endl;
    }
  }
  testCycle.completed = false;
  testCycle.targetHit = std::vector<bool>(testParams.targets.size(), false);
  return true;
}

bool TestController::checkGoal(const int side)
{
  int i = 0;
  for(const TestTarget& target : testParams.targets)
  {
    if((side == 1 && target.type == goalTeamB) || (side == 0 && target.type == goalTeamA))
    {
      testCycle.targetHit[i] = true;
      if(isTargetsExpressionFalse())
      {
        // Expression is always false, so we can skip the rest of the test
        return true;
      }
    }
    ++i;
  }
  return false;
}

bool TestController::checkTime(const int secs)
{
  return secondsOfHalf - secs >= testParams.secsToPlay;
}

bool TestController::checkBallInZone(const Vector2f& pos, const FieldDimensions& fieldDims)
{
  int i = 0;
  for(const TestTarget& target : testParams.targets)
  {
    if(target.type == ballInZone && checkPositionInZone(pos, target.zone.value(), fieldDims))
    {
      testCycle.targetHit[i] = true;
      if(isTargetsExpressionFalse())
      {
        return true;
      }
    }
    ++i;
  }
  return false;
}

std::vector<QString> TestController::getRobotNames() const
{
  std::vector<QString> names;
  for(const TestTarget& target : testParams.targets)
  {
    names.push_back(QString::fromStdString(target.robotName.value_or("")));
  }
  return names;
}

bool TestController::checkRobotInZone(const std::vector<Vector2f>& positions, const FieldDimensions& fieldDims)
{
  int i = 0;
  for(const TestTarget& target : testParams.targets)
  {
    if(target.type == robotInZone && checkPositionInZone(positions.at(i), target.zone.value(), fieldDims))
    {
      testCycle.targetHit[i] = true;
      if(isTargetsExpressionFalse())
      {
        return true;
      }
    }
    ++i;
  }
  return false;
}

bool TestController::checkRobotTouch(const QString& name)
{
  int i = 0;
  for(const TestTarget& target : testParams.targets)
  {
    if(target.type == robotTouch && name.toStdString() == target.robotName)
    {
      testCycle.targetHit[i] = true;
      if(isTargetsExpressionFalse())
      {
        return true;
      }
    }
    ++i;
  }
  return false;
}

bool TestController::checkPenalized(const QString& name)
{
  int i = 0;
  for(const TestTarget& target : testParams.targets)
  {
    if(target.type == penalized && name.toStdString() == target.robotName)
    {
      testCycle.targetHit[i] = true;
      if(isTargetsExpressionFalse())
      {
        return true;
      }
    }
    ++i;
  }
  return false;
}

bool TestController::checkTeamPossession(const QString& name)
{
  int i = 0;
  bool firstTeam = SimulatedRobot::isFirstTeam(RoboCupCtrl::application->resolveObject(name, RoboCupCtrl::controller->is2D ? static_cast<int>(SimRobotCore2D::body) : static_cast<int>(SimRobotCore3::body)));

  for(const TestTarget& target : testParams.targets)
  {
    if(target.type == teamPossession)
    {
      if((target.teamID == teamA && firstTeam) || (target.teamID == teamB && !firstTeam))
      {
        testCycle.targetHit[i] = true;
        if(isTargetsExpressionFalse())
        {
          return true;
        }
      }
    }
    ++i;
  }
  return false;
}

bool TestController::checkRobotInSkill(SkillRequest::Type skill)
{
  // Note: First team must have even team number, second team odd.
  std::string robot = Global::getSettings().teamNumber % 2 == 0 ? "RoboCup.robots.robot" : "RoboCup.robots.robot2";
  robot += std::to_string(Global::getSettings().playerNumber);

  int i = 0;
  for(const TestTarget& target : testParams.targets)
  {
    if(target.type == robotInSkill &&
       QString::fromStdString(target.robotName.value()).compare(QString::fromStdString(robot), Qt::CaseInsensitive) == 0 &&
       skill == target.skill)
    {
      testCycle.targetHit[i] = true;
      if(isTargetsExpressionFalse())
      {
        return true;
      }
    }
    ++i;
  }
  return false;
}

bool TestController::checkPositionInZone(const Vector2f& pos, const TargetZone& zone, const FieldDimensions& fieldDims) const
{
  auto isInCircle = [](Vector2f p, float cx, float cy, float r)
  {
    return (p.x() - cx) * (p.x() - cx) + (p.y() - cy) * (p.y() - cy) <= r * r;
  };
  auto isInRect = [](Vector2f p, float left, float top, float right, float bottom)
  {
    return p.x() >= left && p.x() <= right && p.y() >= top && p.y() <= bottom;
  };

  // Works iff Team A's field has x > 0 (globally), Team B's field has x < 0, and the field is symmetric.
  switch(zone.type)
  {
    case TargetZoneType::fieldHalf:
      if(zone.side == TargetZoneSide::sideA)
      {
        return pos.x() > 0;
      }
      if(zone.side == TargetZoneSide::sideB)
      {
        return pos.x() < 0;
      }
      return true;
    case TargetZoneType::centerCircle:
    {
      bool inCenterCircle = isInCircle(pos, 0, 0, fieldDims.centerCircleRadius);
      if(zone.side == TargetZoneSide::sideA)
      {
        return inCenterCircle && pos.x() > 0;
      }
      if(zone.side == TargetZoneSide::sideB)
      {
        return inCenterCircle && pos.x() < 0;
      }
      return inCenterCircle;
    }
    case TargetZoneType::penaltyArea:
    {
      bool inPenaltyAreaA = pos.x() >= std::abs(fieldDims.xPosOwnPenaltyArea) && std::abs(pos.y()) <= fieldDims.yPosLeftPenaltyArea;
      bool inPenaltyAreaB = pos.x() <= fieldDims.xPosOwnPenaltyArea && std::abs(pos.y()) <= fieldDims.yPosLeftPenaltyArea;

      if(zone.side == TargetZoneSide::sideA)
      {
        return inPenaltyAreaA;
      }
      if(zone.side == TargetZoneSide::sideB)
      {
        return inPenaltyAreaB;
      }
      return inPenaltyAreaA || inPenaltyAreaB;
    }
    case TargetZoneType::goalArea:
    {
      bool inGoalAreaA = pos.x() >= std::abs(fieldDims.xPosOwnGoalArea) && std::abs(pos.y()) <= fieldDims.yPosLeftGoalArea;
      bool inGoalAreaB = pos.x() <= fieldDims.xPosOwnGoalArea && std::abs(pos.y()) <= fieldDims.yPosLeftGoalArea;

      if(zone.side == TargetZoneSide::sideA)
      {
        return inGoalAreaA;
      }
      if(zone.side == TargetZoneSide::sideB)
      {
        return inGoalAreaB;
      }
      return inGoalAreaA || inGoalAreaB;
    }
    case TargetZoneType::customRect:
      return isInRect(pos, zone.topLeft.x(), zone.topLeft.y(), zone.bottomRight.x(), zone.bottomRight.y());
    default:
      return false;
  }
}

bool TestController::checkBudget(const int budgetTeamA, const int budgetTeamB)
{
  int i = 0;
  for(const TestTarget& target : testParams.targets)
  {
    if(target.type == TestController::budget)
    {
      int teamBudget = (target.teamID == TestController::teamA) ? budgetTeamA : budgetTeamB;
      bool result = false;
      switch(target.budgetRule->op)
      {
        case TestController::greaterThan:
          result = teamBudget > target.budgetRule->value;
          break;
        case TestController::greaterEqual:
          result = teamBudget >= target.budgetRule->value;
          break;
        case TestController::lessThan:
          result = teamBudget < target.budgetRule->value;
          break;
        case TestController::lessEqual:
          result = teamBudget <= target.budgetRule->value;
          break;
      }
      testCycle.targetHit[i] = result;
      if(isTargetsExpressionFalse())
      {
        return true;
      }
    }
    ++i;
  }
  return false;
}

bool TestController::isTargetsExpressionFalse() const
{
  return ExpressionEvaluator::evaluate(testParams.targetsExpr, testCycle.targetHit) == ExpressionEvaluator::falseResult;
}

bool TestController::isSituationTestRunning() const
{
  return isTestRunning() && testParams.testType == situation;
}

bool TestController::isGameTestRunning() const
{
  return isTestRunning() && testParams.testType == game;
}

bool TestController::isTestActive() const
{
  return (isTestRunning() || isTestResetting()) && !isTestCycleCompleted();
}

bool TestController::isSituationTestActive() const
{
  return isTestActive() && testParams.testType == situation;
}

bool TestController::isConfigCommandAllowed(const std::string& cmd) const
{
  if(isTestActive())
  {
    if(cmd.starts_with("test"))
    {
      return false;
    }
    if(isFuzzingApplicable() && cmd.starts_with("dr representation:SetupPoses:place"))
    {
      return false;
    }
  }
  return true;
}

TestController::TestAction TestController::finishTestCycle()
{
  SYNC;
  if(!isTestRunning())
    return TestController::nothing;

  if(testParams.testType == game)
  {
    testState.halftimes.push_back(testCycle.gameData[0]);
    testState.halftimes.push_back(testCycle.gameData[1]);
  }

  if(testParams.testType == situation)
  {
    std::string resultCode = TestUtils::boolArrayToString(testCycle.targetHit);

    // TODO: Invert result code for budget target back if needed

    if(isTestCycleSuccessful(false))
    {
      testState.numOfSuccess++;
      resultCode.push_back('1'); // Indicate that the test was successful
    }
    else
    {
      resultCode.push_back('0');
      if(isFuzzingApplicable())
      {
        std::filesystem::path failedConPath = testDirPath / std::format("faulty_config_{}th_run.con", testState.currTestRuns);
        std::filesystem::copy_file(getFuzzedConfigPath(), failedConPath);
      }
    }
    testState.cycleResultCodes.push_back(resultCode);
  }

  if(testState.currTestRuns < testParams.numOfTestRuns)
  {
    testState.currTestRuns++;
    testState.status = TestStatus::resetting;
    saveTestState();
    return TestController::reset;
  }
  else
  {
    testState.status = TestStatus::finished;
    saveTestState();
    logTestResults();
    if(testParams.quit)
    {
      const int result = testState.numOfSuccess == testParams.numOfTestRuns ? EXIT_SUCCESS : EXIT_FAILURE;
#ifndef MACOS
      exit(result);
#else
      MainWindow* mainWindow = static_cast<MainWindow*>(RoboCupCtrl::application);
      QTimer::singleShot(0, mainWindow, SLOT(closeFile()));
      QTimer::singleShot(0, mainWindow, [result] {QApplication::exit(result);});
#endif
    }
    return TestController::stop;
  }
  return TestController::nothing;
}

void TestController::stopTest()
{
  SYNC;
  if(!isTestRunning())
    return;
  testState.status = TestStatus::finished;
  saveTestState();
  logTestResults();
}

TestController::TestParameters& TestController::getTestParameters()
{
  return testParams;
}

std::string TestController::getConsoleMessage()
{
  if(message.empty())
  {
    return "";
  }
  std::string msg = std::move(message);
  message.clear();
  return msg;
}

void TestController::logTestResults()
{
  std::string csvResults;

  if(testParams.testType == game)
  {
    csvResults = "Game No,Half,Kickoff,Score A,Score B,Budget A,Budget B,Winner\n";

    int totalScoreA = 0;
    int totalScoreB = 0;
    int winsTeamA = 0;
    int winsTeamB = 0;
    int draws = 0;

    const int gameCount = static_cast<int>(testState.halftimes.size()) / 2; // 2 halves per game

    auto logHalf = [&](int gameNo, int halfNo, const HalftimeData& halftime, const std::string& kickoffTeam)
    {
      int scoreA = halftime.scores[0];
      int scoreB = halftime.scores[1];
      int budgetA = halftime.budgets[0];
      int budgetB = halftime.budgets[1];

      totalScoreA += scoreA;
      totalScoreB += scoreB;

      std::string winner;
      if(scoreA > scoreB)
      {
        winner = "Team A";
        ++winsTeamA;
      }
      else if(scoreA < scoreB)
      {
        winner = "Team B";
        ++winsTeamB;
      }
      else
      {
        winner = "Draw";
        ++draws;
      }

      csvResults += std::format("{},{},{},{},{},{},{},{}\n", gameNo, halfNo, kickoffTeam,
                                scoreA, scoreB, budgetA, budgetB, winner);
    };

    for(int game = 0; game < gameCount; ++game)
    {
      auto& firstHalf = testState.halftimes[game * 2];
      auto& secondHalf = testState.halftimes[game * 2 + 1];

      logHalf(game + 1, 1, firstHalf, "Team A");
      logHalf(game + 1, 2, secondHalf, "Team B");
    }

    float avgScoreA = testParams.numOfTestRuns > 0 ? static_cast<float>(totalScoreA) / testParams.numOfTestRuns : 0.0f;
    float avgScoreB = testParams.numOfTestRuns > 0 ? static_cast<float>(totalScoreB) / testParams.numOfTestRuns : 0.0f;

    message += std::format("Total score: {}:{}\n"
                           "Wins Team A: {}\n"
                           "Wins Team B: {}\n"
                           "Draws: {}\n"
                           "Average score of a single half-time: {}:{}\n",
                           totalScoreA, totalScoreB,
                           winsTeamA, winsTeamB, draws,
                           avgScoreA, avgScoreB);
  }
  else
  {
    csvResults = "Test Run,Target Hit Code,Success\n";
    for(size_t i = 0; i < testState.cycleResultCodes.size(); ++i)
    {
      std::string targetHits = testState.cycleResultCodes[i].substr(0, testState.cycleResultCodes[i].size() - 1);
      std::string success = testState.cycleResultCodes[i].back() == '1' ? "true" : "false";
      csvResults += std::format("{},{},{}\n", i + 1, targetHits, success);
    }
    message += std::format("Number of successful runs / number of total runs:\n{}/{}\n",
                           testState.numOfSuccess, testParams.numOfTestRuns);
  }

  OutTextRawFile((testDirPath / "results.csv").string()) << csvResults;
}

bool TestController::initTestDir() const
{
  std::filesystem::remove_all(testDirPath);
  return std::filesystem::create_directories(testDirPath);
}

bool TestController::saveTestState() const
{
  OutMapFile outFile(getStateFilePath(), true);
  if(outFile.exists())
  {
    outFile << testState;
    return true;
  }
  return false;
}

void TestController::setTestDirPath(const std::filesystem::path& path)
{
  testDirPath = path;
}

std::filesystem::path TestController::getTestDirPath() const
{
  return testDirPath;
}

std::string TestController::getPrimaryConfigPath() const
{
  return (testDirPath / "config.con").string();
}

std::string TestController::getSecondaryConfigPath() const
{
  if(isFuzzingApplicable())
  {
    return getFuzzedConfigPath();
  }
  return testParams.conFilePath;
}

std::string TestController::getStateFilePath() const
{
  return (testDirPath / "state.txt").string();
}

std::string TestController::getParametersPath() const
{
  return (testDirPath / "parameters.txt").string();
}

std::string TestController::getFuzzedConfigPath() const
{
  return (testDirPath / "fuzzed_config.con").string();
}

void TestController::recordHalftimeData(const int half, const int scoreTeamA, const int scoreTeamB,
                                        const int budgetTeamA, const int budgetTeamB)
{
  SYNC;
  testCycle.gameData[half].scores[0] = scoreTeamA;
  testCycle.gameData[half].budgets[0] = budgetTeamA;

  testCycle.gameData[half].scores[1] = scoreTeamB;
  testCycle.gameData[half].budgets[1] = budgetTeamB;
}

bool TestController::isTestCycleSuccessful(const bool ignoreFalse) const
{
  return ExpressionEvaluator::evaluate(testParams.targetsExpr, testCycle.targetHit, ignoreFalse) == ExpressionEvaluator::trueResult;
}

bool TestController::isTestCycleCompleted() const
{
  return testCycle.completed;
}

bool TestController::isTestRunning() const
{
  return testState.status == TestStatus::running;
}

bool TestController::isTestResetting() const
{
  return testState.status == TestStatus::resetting;
}

bool TestController::isTestFinished() const
{
  return testState.status == TestStatus::finished;
}

bool TestController::isFuzzingApplicable() const
{
  // If we also need to run a cycle with the original user con file,
  // we need to add a check like this: testState.numOfTestRuns > 1
  return testParams.fuzzing && testParams.testType == situation && !testParams.conFilePath.empty();
}
