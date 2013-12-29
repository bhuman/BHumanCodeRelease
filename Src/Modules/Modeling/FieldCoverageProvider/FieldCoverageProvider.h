#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/TeamMateData.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/RobotsModel.h"
#include "Representations/Modeling/FieldCoverage.h"
#include "Representations/Modeling/CombinedWorldModel.h"
#include "Tools/Math/Geometry.h"
#include <vector>
#include <algorithm>

MODULE(FieldCoverageProvider)
  REQUIRES(CameraInfo)
  REQUIRES(CameraMatrix)
  REQUIRES(ImageCoordinateSystem)
  REQUIRES(FieldDimensions)
  REQUIRES(RobotPose)
  REQUIRES(RobotsModel)
  REQUIRES(FrameInfo)
  REQUIRES(TeamMateData)
  REQUIRES(OwnTeamInfo)
  REQUIRES(GameInfo)
  REQUIRES(CombinedWorldModel)
  REQUIRES(JointData)
  PROVIDES_WITH_MODIFY(FieldCoverage)
END_MODULE

class FieldCoverageProvider : public FieldCoverageProviderBase
{
public:
  /**
   * Class to represent a cell of the field coverage grid.
   */
  class Cell
  {
  public:
    static const unsigned char maxCoverage = 255; /**< Coverage value the cell has when the robot looks at it. */
    static const unsigned int tick = 300; /**< Milliseconds one coverage tick is worth. */

    unsigned int lastseen; /**< Timestamp when this cell was last seen. */
    unsigned int lastseenScanPattern; /**< Timestamp used to generate scan-for-ball-pattern. */
    Vector2<> polygon[4]; /**< Edges of the cell in field coordinates. Used for drawing. */
    Vector2<> absoluteCameraTargetOnField; /**< Center of the cell in field coordinates. */
    Vector2<> relativeTargetOnField; /**< Center of the cell relative to the robot. */
    bool shadowed; /**< True if another robot shadows this cell. */
    float distance; /**< The distance between the robot and this cell. Negative if invalid. */

    /**
     * Constructor
     */
    Cell(float xMin, float xMax, float yMin, float yMax)
      : lastseen(0), lastseenScanPattern(0), absoluteCameraTargetOnField((xMin + xMax) / 2.0f, (yMin + yMax) / 2.0f),
        relativeTargetOnField(0.0f, 0.0f), shadowed(false), distance(-1.f)
    {
      polygon[0] = Vector2<>(xMin + 25.0f, yMin + 25.0f);
      polygon[1] = Vector2<>(xMin + 25.0f, yMax - 25.0f);
      polygon[2] = Vector2<>(xMax - 25.0f, yMax - 25.0f);
      polygon[3] = Vector2<>(xMax - 25.0f, yMin + 25.0f);
    }

    /**
     *  Returns the coverage of this cell given the current time [ms]
     */
    inline unsigned char coverage(unsigned time) const
    {
      return coverage(time, this->lastseen);
    }

    inline unsigned char coverage(unsigned time, unsigned lastseen) const
    {
      unsigned sub = (time - lastseen) / tick;
      return sub >= maxCoverage ? 0 : maxCoverage - static_cast<unsigned char>(sub);
    }

    /**
     * Sets the coverage value of this cell to 'coverage' based on the
     * timestamp 'time'.
     */
    inline void setCoverage(unsigned time, unsigned char coverage)
    {
      ASSERT(coverage <= maxCoverage);
      unsigned sub = tick * (maxCoverage - coverage);
      lastseen = time - std::min(time, sub);
    }

    /**
     * Sets the coverage value of this cell to the maximum coverage value
     * based on the timestamp 'time'.
     */
    inline void refresh(unsigned int time) { lastseen = time; }
  };

  /**
   * Class representing a shadow a robot can cast on the field.
   */
  class RobotShadow
  {
  public:
    /**
     * Copy constructor.
     */
    RobotShadow(const RobotShadow& other);

    /**
     * Constructor.
     */
    RobotShadow(const RobotPose& robotPose, const Vector2<>& otherRobotRelativePosition);

    /**
     * Assignment operator for the non-const attributes.
     * Not that this does not assign the robotPose attribute.
     */
    RobotShadow& operator=(const RobotShadow& other);

    /**
     * Tests whether this shadow extends over the center of the cell 'cell'.
     */
    bool isPointShadowed(const Cell& cell) const;

    /**
     * Draws this shadow.
     */
    void draw() const;

    /**
     * Draws this shadow on an image.
     */
    void drawOnImage(const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo,
                     const ImageCoordinateSystem& imageCoordinateSystem) const;
  private:
    const RobotPose& robotPose; /**< The pose of the robot that sees the robot causing the shadow. */
    float distance; /**< Distance to the robot causing the shadow. */
    Vector2<> vertices[4]; /**< Vertices of the shadow in field coordinates */

    /**
     * Projects a point in field coordinates 'absPosOnField' into a
     */
    Vector2<> projectOnImage(const Vector2<>& absPosOnField, const CameraMatrix& cameraMatrix,
                             const CameraInfo& cameraInfo, const ImageCoordinateSystem& imageCoordinateSystem) const;
  };

  class BallTime
  {
  public:
    Vector2<> position;
    unsigned time;
  };

  /**
   * Constructor
   */
  FieldCoverageProvider();

private:
  static const size_t xSteps = 12; /**< Number of cells the field is devided into in field-coordinate-x direction. */
  static const size_t ySteps = 8; /**< Number of cells the field is devided into in field-coordinate-y direction. */
  Vector2<> cellLength; /**< Length of the sides of a cell. */
  const float range; /**< Maximum distance between a robot and a cell center allowed when refreshing that cell. */
  Cell* fieldCoverageGrid[xSteps][ySteps]; /**< The coverage grid. Each cell is 0.5 m x 0.5 m. */
  FieldCoverage::GridInterval teamData; /**< The part of the field coverage grid which is to be sent to the teammates. */
  int lastWorstIdx; /**< Index of the worst covered cell from the last frame. */
  int lastWorstNoTurnIdx; /**< Index of the worst covered visible cell from the last frame. */
  int lastWorstNoTurnRangeIdx; /**< Index of the worst covered visible cell closer than range from the last frame. */
  int lastWorstNoTurnHalfRangeIdx; /**< Index of the worst covered visible cell closer than half the range from the last frame. */
  const float visibleAngle; /**< Angle up to which a cell is considered to be visible. */
  const float throwInLineDistanceSL; /**< Distance between the throw-in line and the closest field border sideline in mm. */
  const float throwInLineDistanceGL; /**< Distance between the throw-in line and the closest field border groundline in mm. */
  float xPosOpponentThrowInPoint; /**< x-field-coordinate of the end of the throw in line. */
  float xPosOwnThrowInPoint; /**< x-field-coordinate of the beginning of the throw in line. */
  float yPosLeftThrowInLine; /**< y-field-coordinate of the left throw in line. */
  float yPosRightThrowInLine; /**< y-field-coordinate of the right throw in line. */
  int throwInLineOffset; /**< Distance between the throw-in line and the closest field border in number-of-cells. */
  int lastThrowInTimestamp; /**< Time stamp of the last throw in *seconds* */
  const float throwInPenaltyDistance; /**< Distance the ball is moved backwards when the ball is thrown in. */
  const int throwInCellDiff; /**< Absolute coverage value difference between to cell on the throw-in line when the ball is thrown in. */
  const int minThrowInCellCoverage; /**< Minimum coverage of the cells which are not on a throw-in line when the ball is thrown in. */
  unsigned char lastGameState; /**< Game state from the last frame. */
  BallTime lastValidInsideLyingBall; /**< Position of the last valid global ball which is inside the field and does not move. */
  BallTime lastValidInsideBall; /**< Position of the last valid global ball which is inside the field. */
  BallTime lastValidOutsideBall; /**< Position of the last valid global ball */
  std::vector<Geometry::Line> fieldBorder; /**< The field border. */
  BallTime ballOut; /**< The position and timestamp of the last time the ball went out. */

  /**
   * Destructor
   */
  ~FieldCoverageProvider();

  /**
   * Updates the field coverage representation.
   */
  void update(FieldCoverage& fieldCoverage);

  /**
   * Computes the shadows of all robots in the robotsModel.
   */
  void computeShadows(std::vector<RobotShadow>& shadows, const RobotPose& robotPose, const RobotsModel& robotsModel);

  /**
   * Tests whether a cell 'cell' is shadowed by one of the 'shadows'.
   */
  bool isCellShadowed(const std::vector<RobotShadow>& shadows, const Cell& cell);

  /**
   * Projects a point from the image onto the field.
   */
  Vector2<> projectOnField(int x, int y);

  /**
   * Calculates the mean coverage value of all cells of the field coverage grid.
   */
  void calculateMean(FieldCoverage& fieldCoverage);

  /**
   * Calculates the standard deviation of the cells of the field coverage grid.
   */
  void calculateStdDev(FieldCoverage& fieldCoverage);

  /**
   * Returns when the ball is thrown in.
   */
  bool isBallThrownIn(); /** True if the ball is thrown in. */

  /**
   * Modifies the field coverage grid when the ball is thrown in.
   */
  void ballThrowIn(FieldCoverage& fieldCoverage);

  /**
   * Tries to determine where the ball crossed the field border
   * when it went out.
   */
  bool calculateBallOutPosition();

  void drawFieldView();
};
