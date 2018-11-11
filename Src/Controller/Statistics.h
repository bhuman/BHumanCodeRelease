/**
 * @file Controller/Statistics.h
 *
 * Definition of class Statistics.
 *
 * @author <a href="mailto:jan_fie@uni-bremen.de">Jan Fiedler</a>
 * @author <a href="mailto:urbant@uni-bremen.de">Timo Urban</a>
 */

#pragma once

#include "Representations/Communication/TeamData.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/MotionControl/GetUpEngineOutputLog.h"
#include "RobotConsole.h"
#include "SimRobot.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"
#include "Tools/Streams/EnumIndexedArray.h"

#include <algorithm>
#include <list>
#include <map>
#include <QString>

class ConsoleRoboCupCtrl;
class Streamable;
struct BallModel;
struct BallPercept;
struct FallDownState;
struct FrameInfo;
struct GameInfo;
struct KeyStates;
struct MotionRequest;
struct RobotHealth;
struct RobotInfo;
struct RobotPose;
struct TeamData;

STREAMABLE(Statistic,
{
  static const unsigned xFieldParts = 20; /**< Number of field parts in x. */
  static const unsigned yFieldParts = 14; /**< Number of field parts in y. */
  static const unsigned heatStatus = 50; /**< Minimum relevant temperature for calibration. */

  struct Representations
  {
    Representations(const BallModel& ballModel, const BallPercept& ballPercept, const FallDownState& fallDownState,
                    const FrameInfo& frameInfo, const GameInfo& gameInfo, const GetUpEngineOutputLog& getUpEngineOutputLog,
                    const JointSensorData& jointSensorData, const KeyStates& keyStates, const MotionRequest& motionRequest,
                    const RobotHealth& robotHealth, const RobotInfo& robotInfo, const RobotPose& robotPose,
                    const TeamData& teamData) :
      ballModel(ballModel)COMMA ballPercept(ballPercept)COMMA fallDownState(fallDownState)COMMA frameInfo(frameInfo)
      COMMA gameInfo(gameInfo)COMMA getUpEngineOutputLog(getUpEngineOutputLog)COMMA jointSensorData(jointSensorData)
      COMMA keyStates(keyStates)COMMA motionRequest(motionRequest)COMMA robotHealth(robotHealth)COMMA robotInfo(robotInfo)
      COMMA robotPose(robotPose)COMMA teamData(teamData) {};
    const BallModel& ballModel;
    const BallPercept& ballPercept;
    const FallDownState& fallDownState;
    const FrameInfo& frameInfo;
    const GameInfo& gameInfo;
    const GetUpEngineOutputLog& getUpEngineOutputLog;
    const JointSensorData& jointSensorData;
    const KeyStates& keyStates;
    const MotionRequest& motionRequest;
    const RobotHealth& robotHealth;
    const RobotInfo& robotInfo;
    const RobotPose& robotPose;
    const TeamData& teamData;
  };

  STREAMABLE(Heatmap,
  {
    Heatmap() = delete;
    Heatmap(const unsigned width, const unsigned height) : width(width)COMMA height(height)COMMA data(width * height) {};
    unsigned& operator()(const unsigned x, const unsigned y) { return data[y * width + x]; };
    void clear() { std::fill(data.begin(), data.end(), 0); };
    unsigned maxValue() const
    {
      unsigned max = 0;
      for(size_t i = 0; i < data.size(); i++)
      {
        if(max < data[i])
          max = data[i];
      }
      return max;
    },

    (unsigned)(0) width,
    (unsigned)(0) height,
    (std::vector<unsigned>) data,
  });

  /* More things that might be useful, but first not so relevant for the calibration.
   * - Use MotionRequest (KickRequest) to analyze kicks
   * - jumps of the ballpercept
   * - successes of certain shots
   */
  STREAMABLE(StatisticRobot,
  {
    // Variables for calculations.
    bool isPickedUp = false;
    bool isPenalized = false;
    bool isBallKicked = false;
    bool isFallen = false;
    bool isUpright = false;
    bool isKicking = false;
    int fallenState = 0; /**< To determine failedGetUps. */
    unsigned kickDuration = 0;
    unsigned inWalkDuration = 0; /**< To determine inWalkKicks. */
    unsigned timeWhenLastAddTemperature = 0;
    unsigned tryCounterBuffer = 0;
    unsigned jointTemperatureTime = 5;
    bool isInWalkKicking = false;
    Vector2f ballPositionInWalk = Vector2f::Zero(); /**< To determine inWalkKicks. */
    Vector2f lastPosition = Vector2f::Zero(); /**< To determine inWalkKicks. */
    MotionRequest::Motion lastMotion, /**< To determine failedGetUps. */

    (std::string) name,
    (int)(0) number,
    (unsigned)(1) logCount, // Number of logs contained in this robot. If there is more than one file.
    (RobotHealth::Configuration) configuration,
    (std::string) location,
    (std::string) scenario,
    (unsigned)(0) frames,
    (unsigned)(0) pickedUps,
    (unsigned)(0) penaltys,
    (unsigned)(0) fallen, // Number of fallen
    (float)(0.f) walkedDistance,
    (unsigned)(0) badLocalisations,
    (std::vector<unsigned>) fallenFrames,
    (unsigned)(0) failedGetUps,
    (std::vector<unsigned>) failedGetUpFrames,
    //Number of
    (unsigned)(0) walkRequests,
    (unsigned)(0) standRequests,
    (unsigned)(0) specialActionRequests,
    (unsigned)(0) getUpRequests,
    (unsigned)(0) kickRequests,
    (unsigned)(0) ballKicked,
    (unsigned)(0) inWalkKicks,
    (unsigned)(0) seenBalls,
    (unsigned)(0) guessedBalls,
    (unsigned)(0) receivedMessages,
    (std::vector<unsigned>)({0, 0, 0, 0, 0, 0}) receivedMessagesRobots,
    (Heatmap)(Heatmap(xFieldParts, yFieldParts)) heatmap,
    (ENUM_INDEXED_ARRAY(unsigned, Joints::Joint)) maxJointTemperatures,
    (ENUM_INDEXED_ARRAY(std::vector<unsigned>, Joints::Joint)) jointTemperatures,
  });,

  (std::vector<StatisticRobot>) robots,
  (Heatmap)(Heatmap(xFieldParts, yFieldParts)) teamHeatmap,
});

/**
 * @class Statistics
 *
 * This class collects data for statistics and provides IO operations.
 */
class Statistics
{
public:
  RobotConsole::Plots plots; /**< Buffers for plots from the debug queue. */
  RobotConsole::PlotViews plotViews; /**< The map of all plot views. */

  RobotConsole::Views fieldViews; /**< The map of all field views. */
  RobotConsole::Drawings fieldDrawings; /**< Buffer for field drawings. */

private:
  DECLARE_SYNC;
  FieldDimensions fieldDimensions;
  Vector2f fieldPartSize; /**< Size of one Field in a Heatmap. */

  bool active = false; /**< Is the statistics feature active? */
  Statistic statistic;

  ConsoleRoboCupCtrl& ctrl; /**< A reference to the console object. */
  SimRobot::Object* category = nullptr; /**< The root scene graph object. Null until this is initialized. */
  const QString catName = "Statistics."; /**< The toplevel name of all widgets. */

public:
  Statistics(ConsoleRoboCupCtrl& ctrl);

  /**
   * Initialize Statistics.
   */
  void initialize();

  /**
   * Update the robot from log-frame.
   * @param frameType Type of the frame (m/c);
   * @param index The index of the robot.
   * @param repr struct with all needed Representations.
   */
  void updateLogFrame(const char frameType, const size_t index, const Statistic::Representations& repr);

  /**
   * Returns the index of the robot. If the robot is unknown, a new one will be created and the view will be triggered.
   * @param robotName Name of the robot.
   * @return Index of the robot.
   */
  size_t getRobotIndex(const std::string& robotName);

  /**
   * Add view for given robot.
   * @param index The robot index.
   */
  void viewRobot(const size_t index);

  /**
   * Export Statistic.
   */
  void exportSR() const;

  /**
   * Import Statistic.
   */
  void importSR();

  void exportHeatMaps();

  /**
   * Shows all not so important joint temperatures in graphs.
   */
  void plotJointTemperatures();

private:
  void updateCognitionFrame(Statistic::StatisticRobot& robot, const BallModel& ballModel, const BallPercept& ballPercept,
                            const FrameInfo& frameInfo, const GameInfo& gameInfo, const RobotHealth& robotHealth,
                            const RobotInfo& robotInfo, const RobotPose& robotPose, const TeamData& teamData);
  void updateMotionFrame(Statistic::StatisticRobot& robot, const FallDownState& fallDownState, const FrameInfo& frameInfo,
                         const GetUpEngineOutputLog& getUpEngineOutputLog, const JointSensorData& jointSensorData,
                         const KeyStates& keyStates, const MotionRequest& motionRequest);

  /**
   * Add All DebugDrawings for the given Heatmap.
   * Note: -1 for the teamHeatmap.
   * @param index The heatmap index.
   * @param namePrefix <prefix>heatmap
   */
  void fillHeatmap(const int index, const std::string& namePrefix);

  /**
   * Shows all important joint temperatures from given robot in graphs.
   * @param robot The given robot.
   * @param robotCategory The category for adding new Views in SimRobot.
   */
  void plotJointTemperatures(const Statistic::StatisticRobot& robot, SimRobot::Object* robotCategory);

  /**
   * The method writes a csv.
   * @param fileName The file name.
   * @param writeHeader Append the head to the file.
   * @param writeInFile Addend values to the file.
   * @return whether the action was successful or not.
   */
  bool exportCSV(const std::string& fileName, const std::function<void(Out& file, const std::string& sep)>& writeHeader,
                 const std::function<void(Out& file, const std::string& sep)>& writeInFile) const;
};
