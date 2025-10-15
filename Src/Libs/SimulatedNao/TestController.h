/**
 * @file SimulatedNao/TestController.h
 *
 * This file declares a class that is responsible for
 * controlling the automatic test execution.
 *
 * @author Nico Holsten
 * @author Adam Cihasev
 */

#pragma once

#include "Representations/BehaviorControl/SkillRequest.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Streaming/Enum.h"
#include "Streaming/AutoStreamable.h"
#include "Platform/Thread.h"
#include "TestUtils.h"
#include <filesystem>

class QString;

class TestController
{
public:
  ENUM(TestAction,
  {,
    reset,
    stop,
    nothing,
  });

  ENUM(TestType,
  {,
    notActive,
    game,
    situation,
  });

  ENUM(TeamID,
  {,
    noTeam,
    teamA,
    teamB,
  });

  ENUM(TargetType,
  {,
    noTarget,
    goalTeamA,
    goalTeamB,
    ballInZone,
    robotTouch,
    robotInZone,
    teamPossession,
    penalized,
    robotInSkill,
    budget,
  });

  ENUM(TargetZoneType,
  {,
    fieldHalf,
    centerCircle,
    penaltyArea,
    goalArea,
    customRect,
  });

  ENUM(TargetZoneSide,
  {,
    sideA,
    sideB,
    any,
  });

  ENUM(TestStatus,
  {,
    initial,   /**< The test has not started yet */
    running,   /**< The test is currently running */
    resetting, /**< The test is currently resetting */
    finished,  /**< The test is fully completed */
  });

  ENUM(ComparisonOperator,
  {,
    greaterThan, /**< The value is greater than the given value */
    lessThan,    /**< The value is less than the given value */
    greaterEqual,/**< The value is greater than or equal to the given value */
    lessEqual,   /**< The value is less than or equal to the given value */
  });

  STREAMABLE(FuzzingParams,
  {,
    (Vector3f)(0.f, 0.f, 0.f) pos, /**< The position of the robot */
    (Vector3f)(0.f, 0.f, 0.f) rot, /**< The rotation of the robot */
  });

  STREAMABLE(TargetZone,
  {,
    (TargetZoneType)(customRect) type,  /**< The type of the zone */
    (TargetZoneSide)(any) side,         /**< The side of the zone */
    (Vector2f)(0.f, 0.f) topLeft,       /**< The top left coordinates of a zone */
    (Vector2f)(0.f, 0.f) bottomRight,   /**< The bottom right coordinates of a zone */
  });

  STREAMABLE(HalftimeData,
  {,
    (std::array<int, 2>)({}) budgets, /**< The amount of packets left for each team in this half */
    (std::array<int, 2>)({}) scores, /**< The amount of goals scored by each team in this half */
  });

  STREAMABLE(ComparisonRule,
  {,
    (ComparisonOperator)(lessThan) op, /**< The operator to compare the value with */
    (int)(0) value, /**< The value to compare with */
  });

  STREAMABLE(TestTarget,
  {,
    (TargetType)(noTarget) type, /**< indicates the goal of the test */
    (std::optional<TeamID>) teamID, /**< indicates the label of a tested team */
    (std::optional<std::string>) robotName, /**< the name of a tested robot */
    (std::optional<SkillRequest::Type>) skill, /*the skill */
    (std::optional<TargetZone>) zone, /**< The zone to be checked */
    (std::optional<ComparisonRule>) budgetRule, /**< The rule to compare the budget with */
  });

  STREAMABLE(TestState,
  {,
    (TestStatus)(TestStatus::initial) status, /**< indicates the state of the test */
    (int)(0) currTestRuns, /**< indicates the number of test runs that have already been started */
    (int)(0) numOfSuccess, /**< indicates how many test runs have been successful */
    (std::vector<HalftimeData>) halftimes, /**< contains halftimes of the test, each half contains the scores and budgets of both teams */
    (std::vector<std::string>) cycleResultCodes, /**< contains target hit info for each test cycle and whether the cycle was successful */
  });

  STREAMABLE(TestParameters,
  {,
    (TestType)(TestType::notActive) testType, /**< indicates the type of the test */
    (int)(600) secsToPlay, /**< indicates the max. amount of seconds of each test */
    (std::string) conFilePath, /**< conFile which will be executed at the beginning of every test */
    (int)(0) numOfTestRuns, /**< indicates the max. number of test runs */
    (bool)(false) fuzzing, /**< Whether the test is fuzzing */
    (bool)(false) realTime, /**< Whether the test is running in real time */
    (bool)(false) quit, /**< Whether the test should quit the simulator after completion */
    (FuzzingParams) fuzzingParams, /*the parameters for the fuzzing */
    (std::vector<TestTarget>) targets, /**< the targets of the test */
    (TestUtils::ExpressionEvaluator::LogicalExpression) targetsExpr, /**< the logical expression of the targets */
    (std::array<int, 2>)({}) teamNumbers, /**< The team numbers of the teams in the test */
    (TeamID)(teamA) turn, /**< The team that receives the ball at the start of the test */
  });

  struct TestCycle
  {
    bool completed = false;
    std::vector<bool> targetHit;
    std::array<HalftimeData, 2> gameData; /**< The data of the game for each half */
  };

private:
  DECLARE_SYNC;
  const int secondsOfHalf = 600; /**< The amount of seconds of each half */
  std::filesystem::path testDirPath; /**< Process unique path where the test files are stored */
  TestParameters testParams; /**< The parameters of the test */
  TestState testState; /**< Test state containing all necessary test data */
  TestCycle testCycle; /**< Temporary test cycle data */
  std::string message; /**< Message that needs to be printed in console */

  /**
   * Creates/cleans the test directory.
   * @return Whether the test directory was created/cleaned successfully.
   */
  bool initTestDir() const;

public:

  /**
   * Initializes the test state object and prepares the scene.
   * @param filePath The path to the scene file.
   * @param inputParams The parameters of the test to be executed.
   * @return Whether the test state object was initialized successfully.
   */
  bool init(const QString& scenePath, TestController::TestParameters& inputParams);

  /**
   * Writes the test state date to the corresponding file.
   * @return Whether the test state data was written successfully.
   */
  bool saveTestState() const;

  /**
   * Loads the test state from the file into the test state object.
   * Also loads test parameters from the parameters file.
   * @param fieldDims The field dimensions (used for fuzzing).
   *
   * @return Whether the test state object was loaded successfully.
   */
  bool loadTest(const FieldDimensions& fieldDims);

  /**
   * Runs the selected check with the given arguments.
   * @param check The check to be executed.
   * @param args The arguments of the check.
   * @return Whether the check was successful/test cycle is completed.
   */
  template <typename Callable, typename... Args>
  bool runCheck(Callable&& check, Args&& ... args)
  {
    static_assert(std::is_invocable_v<Callable, TestController*, Args...>,
                  "Callable must be invocable with TestController* and the provided arguments.");
    SYNC;
    bool result = std::invoke(std::forward<Callable>(check), this, std::forward<Args>(args)...);
    result = result || isTestCycleSuccessful();
    testCycle.completed = testCycle.completed || result;  // Avoid overwriting completed state
    return result;
  }

  /** Checks if a team scored a goal */
  bool checkGoal(const int side);

  /** Checks if the seconds of the test case are expired */
  bool checkTime(const int secs);

  /** Checks if a specific position is in a specific zone */
  bool checkPositionInZone(const Vector2f& pos, const TargetZone& zone, const FieldDimensions& fieldDims) const;

  /** Checks if the ball is in a specific zone */
  bool checkBallInZone(const Vector2f& ball, const FieldDimensions& fieldDims);

  /** Checks if the specific robot is in a specific zone */
  bool checkRobotInZone(const std::vector<Vector2f>& positions, const FieldDimensions& fieldDims);

  /** Checks if a specific robot touched the ball */
  bool checkRobotTouch(const QString& name);

  /** Checks if a specific robot is penalized */
  bool checkPenalized(const QString& name);

  /** Checks if a specific team is in ball possession */
  bool checkTeamPossession(const QString& name);

  /** Checks if a specific robot is in a specific skill */
  bool checkRobotInSkill(const SkillRequest::Type skill);

  /** Checks if the budget of a specific team is within the specified limits */
  bool checkBudget(const int budgetTeamA, const int budgetTeamB);

  /**
   * Processes the end of a test cycle and returns the next action to be performed.
   * @return The next action to be performed.
   */
  TestAction finishTestCycle();

  /**
   * Stops the test execution and logs the results.
   */
  void stopTest();

  /**
   * Logs the test results to the console and corresponding files.
   */
  void logTestResults();

  /**
   * Sets the path where the test files are stored.
   * @param path The path where the test files are stored.
   */
  void setTestDirPath(const std::filesystem::path& path);

  /**
   * Records the scores of the teams in the current test cycle.
   * @param half The current half of the game (0 for first half, 1 for second half).
   * @param scoreTeamA The score of Team A.
   * @param scoreTeamB The score of Team B.
   * @param budgetTeamA The remaining packet budget of Team A.
   * @param budgetTeamB The remaining packet budget of Team B.
   */
  void recordHalftimeData(const int half, const int scoreTeamA, const int scoreTeamB,
                          const int budgetTeamA, const int budgetTeamB);

  /** Checks if the test iteration was positive */
  bool isTestCycleSuccessful(const bool ignoreFalse = true) const;

  /** Checks if the test iteration is done */
  bool isTestCycleCompleted() const;

  /** Checks if the test is running */
  bool isTestRunning() const;

  /** Checks if the test is resetting */
  bool isTestResetting() const;

  /** Checks if the test is finished */
  bool isTestFinished() const;

  /** Checks if the targets expression evaluates to false */
  bool isTargetsExpressionFalse() const;

  /** Checks if a situation test is running */
  bool isSituationTestRunning() const;

  /** Checks if a game test is running */
  bool isGameTestRunning() const;

  /** Checks if a test is active (running or resetting) */
  bool isTestActive() const;

  /** Checks if a situation test is active (running or resetting) */
  bool isSituationTestActive() const;

  /** Checks if it is allowed to execute a command from the config file */
  bool isConfigCommandAllowed(const std::string& cmd) const;

  /** Returns the path where the test files are stored */
  std::filesystem::path getTestDirPath() const;

  /** Returns the path to the base test config file */
  std::string getPrimaryConfigPath() const;

  /** Returns the path to the active user con file */
  std::string getSecondaryConfigPath() const;

  /** Returns the path to the test state file */
  std::string getStateFilePath() const;

  /** Returns the path to the file with the test options */
  std::string getParametersPath() const;

  /** Returns the path to the fuzzed con file */
  std::string getFuzzedConfigPath() const;

  /** Checks if fuzzing is applicable to the current test */
  bool isFuzzingApplicable() const;

  /** Returns parameters of the test */
  TestParameters& getTestParameters();

  /** Returns message that needs to be printed in console */
  std::string getConsoleMessage();

  /** Returns the robot names */
  std::vector<QString> getRobotNames() const;
};
