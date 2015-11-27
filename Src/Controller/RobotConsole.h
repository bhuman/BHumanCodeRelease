/*
* @file Controller/RobotConsole.h
*
* Declaration of RobotConsole.
*
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

#ifdef WINDOWS
#define NOMINMAX
#include <WinSock2.h> // This must be included first to prevent errors, since windows.h is included in one of the following headers.
#endif

#include "Representations/BehaviorControl/ActivationGraph.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/Configuration/ColorCalibration.h"
#include "Representations/Configuration/ColorTable.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Infrastructure/USRequest.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Infrastructure/SensorData/SystemSensorData.h"
#include "Representations/Infrastructure/SensorData/UsSensorData.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Perception/LinePercept.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Debugging/DebugImages.h"
#include "Tools/ProcessFramework/Process.h"

#include "CameraCalibratorHandler.h"
#include "AutomaticCameraCalibratorHandlerInsertion.h"
#include "AutomaticCameraCalibratorHandlerDeletion.h"
#include "LogPlayer.h" // Must be included after Process.h
#include "Platform/Joystick.h"
#include "Representations/AnnotationInfo.h"
#include "Representations/ModuleInfo.h"
#include "Representations/TimeInfo.h"
#include "Views/DataView/DataView.h"
#include "Visualization/DebugDrawing.h"
#include "Visualization/DebugDrawing3D.h"

#include <QString>
#include <fstream>
#include <list>

class ConsoleRoboCupCtrl;
class ColorCalibrationView;
class ImageView;

/**
* @class RobotConsole
*
* A process that manages the text console for a single robot.
*/
class RobotConsole : public Process
{
private:
  /**
   * Writes a message to a list of strings. Used to save representations as files,
   * i.e. save calibration values set in the simulator.
   */
  class MapWriter : public MessageHandler
  {
  private:
    StreamHandler& streamHandler;
    Out& stream;

  public:
    MapWriter(StreamHandler& streamHandler, Out& stream) : streamHandler(streamHandler), stream(stream) {}

    bool handleMessage(InMessage& message);
  };

  /**
   * Writes a message to the console.
   */
  class Printer : public MessageHandler
  {
  private:
    ConsoleRoboCupCtrl* ctrl;

  public:
    Printer(ConsoleRoboCupCtrl* ctrl) : ctrl(ctrl) {}

    bool handleMessage(InMessage& message);
  };

public:
  ENUM(Color, /**< Colors for plot drawings. */
  {,
    black,
    red,
    green,
    blue,
    yellow,
    cyan,
    magenta,
    orange,
    violet,
    gray,
  });

  DECLARE_SYNC; /**< Make this object synchronizable. */
  ColorCalibration colorCalibration; /**< The color calibration */
  ColorCalibration prevColorCalibration; /**< The previous color calibration */
  bool colorCalibrationChanged = false; /**< Was the color calibration changed since the color table was updated? */
  ColorTable colorTable; /**< The color table */
  unsigned colorTableTimeStamp = 0; /**< The last time when the last color table was updated. */
  ColorCalibrationView* colorCalibrationView = nullptr;
  std::vector<ImageView*> segmentedImageViews;
  SystemCall::Mode mode; /**< Defines mode in which this process runs. */

protected:
  ConsoleRoboCupCtrl* ctrl; /** A pointer to the controller object. */
  QString robotFullName; /**< The full name of the robot. (e.g. "RoboCup.Robot1") */
  QString robotName; /**< The name of the robot. (e.g. "Robot1") */
  bool printMessages = true; /**< Decides whether to output text messages in the console window. */
  bool handleMessages = true; /**< Decides whether messages are handled or not. */
  bool logAcknowledged = true; /**< The flag is true whenever log data sent to the robot code was processed. */
  bool destructed = false; /**< A flag stating that this object has already been destructed. */
  std::string logFile; /**< The name of the log file replayed. */
  LogPlayer logPlayer; /**< The log player to record and replay log files. */
  MessageQueue& debugOut; /**< The outgoing debug queue. */
  StreamHandler streamHandler; /**< Local stream handler. Note: Process::streamHandler may be accessed unsynchronized in different thread, so don't use it here. */
  DrawingManager drawingManager;
  DrawingManager3D drawingManager3D;
  DebugRequestTable debugRequestTable;
  const char* pollingFor = nullptr; /**< The information the console is waiting for. */
  JointRequest jointRequest; /**< The joint angles request received from the robot code. */
  JointSensorData jointSensorData; /**< The most current set of joint angles received from the robot code. */
  FsrSensorData fsrSensorData; /**< The most current set of fsr sensor data received from the robot code. */
  InertialSensorData inertialSensorData; /**< The most current set of inertial sensor data received from the robot code. */
  KeyStates keyStates; /**< The most current set of key states received from the robot code. */
  SystemSensorData systemSensorData; /**< The most current set of system sensor data received from the robot code. */
  UsSensorData usSensorData; /**< The most current set of us sensor data received from the robot code. */
  enum MoveOp {noMove, movePosition, moveBoth, moveBallPosition} moveOp = noMove; /**< The move operation to perform. */
  Vector3f movePos = Vector3f::Zero(); /**< The position the robot is moved to. */
  Vector3f moveRot = Vector3f::Zero(); /**< The rotation the robot is moved to. */
  ObstacleModelCompressed obstacleModelCompressed; /**< Obstacle model from team communication. */
  RobotPose robotPose; /**< Robot pose from team communication. */
  BallModel ballModel; /**< Ball model from team communication. */
  TeamBallModel teamBallModel; /**< combined ball information from team communication */
  TeamPlayersModel teamPlayersModel; /**< combined player information from team communication */
  GoalPercept goalPercept; /**< Goal percept from team communication. */
  LinePercept linePercept; /**< Line percept from team communication. */
  BehaviorStatus behaviorStatus; /**< Behavior data from team communication. */
  RobotHealth robotHealth; /**< Robot Health from team communication. */
  MotionRequest motionRequest; /**< Motion Request from team communication. */
  bool isPenalized = false; /**< Penalized state from team communication. */
  bool hasGroundContact = true; /**< Ground contact state from team communication. */
  bool isUpright = true; /**<fall down state from team communication */
  unsigned obstacleModelCompressedReceived = 0; /**< When was the obstacle model received from team communication. */
  unsigned robotPoseReceived = 0; /**< When was the robot pose received from team communication. */
  unsigned ballModelReceived = 0; /**< When was the ball model received from team communication. */
  unsigned goalPerceptReceived = 0; /**< When was the goal percept received from team communication. */
  unsigned linePerceptReceived = 0; /**< When was the line percept received from team communication. */
  unsigned robotHealthReceived = 0; /**< When was the robot health received from team communication. */
  unsigned motionRequestReceived = 0; /**< When was the motion request received from team communication. */
  unsigned isPenalizedReceived = 0; /**< When was the penalized state received from team communication. */
  unsigned hasGroundContactReceived = 0; /**< When was the ground contact state received from team communication. */
  unsigned teamBallModelReceived = 0; /**< When was the team ball model received from team communication. */
  unsigned teamPlayersModelReceived = 0; /**< When was the team players model received from team communication. */
  unsigned isUprightReceived = 0; /**< When was the fall down state received from team communication. */
  int mrCounter = 0; /**< Counts the number of mr commands. */
  JointCalibration jointCalibration; /**< The joint calibration received from the robot code. */
  RobotDimensions robotDimensions; /**< The robotDimensions received from the robot code. */
  USRequest usRequest;  /**< The current us request received from the robot code (for simulation). */
  std::string printBuffer; /**< Buffer used for command get. */
  char drawingsViaProcess = 'b'; /** Which process is used to provide field and 3D drawings */
  std::unordered_map<char, AnnotationInfo> annotationInfos;

public:
  class ImagePtr
  {
  public:
    Image* image = nullptr;
    char processIdentifier = 0; /**< "c" denotes lower camera process, "d" denotes upper camera process */

    ~ImagePtr() {reset();}

    void reset() {if(image) delete image; image = nullptr;}
  };
  typedef std::unordered_map<std::string, ImagePtr> Images; /**< The type of the map of images. */

  Images upperCamImages;
  Images lowerCamImages;

  typedef std::unordered_map<std::string, DebugDrawing> Drawings;
  Drawings upperCamImageDrawings, lowerCamImageDrawings, motionImageDrawings; /**< Buffers for image drawings from the debug queue. */
  Drawings upperCamFieldDrawings, lowerCamFieldDrawings, motionFieldDrawings; /**< Buffers for field drawings from the debug queue. */
  typedef std::unordered_map<std::string, DebugDrawing3D> Drawings3D;
  Drawings3D upperCamDrawings3D, lowerCamDrawings3D, motionDrawings3D; /**< Buffers for 3d drawings from the debug queue. */

  Images* currentImages = nullptr;
  Drawings* currentImageDrawings = nullptr;
  Drawings* currentFieldDrawings = nullptr;
  Drawings3D* currentDrawings3D = nullptr;

  struct Plot
  {
    std::list<float> points;
    unsigned timeStamp = 0;
  };

  typedef std::unordered_map<std::string, Plot> Plots;
  Plots plots; /**< Buffers for plots from the debug queue. */

  struct Layer
  {
    std::string layer;
    std::string description;
    ColorRGBA color;

    bool operator==(const Layer& other) const {return layer == other.layer;}
  };

  typedef std::unordered_map<std::string, std::list<Layer> > PlotViews;
  typedef std::unordered_map<std::string, std::list<std::string> > Views;
  PlotViews plotViews; /**< The map of all plot views. */

  Views fieldViews; /**< The map of all field views. */
  Views imageViews; /**< The map of all image views. */
  Views imageViews3D; /**< The map of all 3-D image views. */
  ModuleInfo moduleInfo; /**< The current state of all solution requests. */

  /**List of currently active representation views. Key: representation name, value: pointer to the view */
  std::map<std::string, DataView*> representationViews;

  std::map<std::string, ImageView*> actualImageViews;

  /**
  * A MessageHandler that parses debug responses containing representation data and forwards them to the correct representation view
  */
  class RepViewWriter : public MessageHandler
  {
  private:
    std::map<std::string, DataView*>* pRepViews; /**< Pointer to representationViews of RobotConsole */

  public:
    RepViewWriter(std::map<std::string, DataView*>* pViews) : pRepViews(pViews) {}

    /**
     * Forwards the specified message to the representation view that  displays it.
     */
    bool handleMessage(InMessage& message, const std::string& type, const std::string& name);
    /**
     * Same as above but it extracts the type and name from the message
     */
    bool handleMessage(InMessage& message);
  };
  RepViewWriter repViewWriter; /**< The repViewWriter which is used to translate representations into a format that can be understood by the representation views */

private:
  typedef std::pair<std::string, MessageQueue*> DebugDataInfoPair; /**< The type of the information on a debug data entry. */
  typedef std::unordered_map<std::string, DebugDataInfoPair> DebugDataInfos; /**< The type of the map debug data. */
  DebugDataInfos debugDataInfos; /** All debug data information. */

  Images incompleteImages; /** Buffers images of this frame (created on demand). */
  Drawings incompleteImageDrawings; /**< Buffers incomplete image drawings from the debug queue. */
  Drawings incompleteFieldDrawings; /**< Buffers incomplete field drawings from the debug queue. */
  Drawings3D incompleteDrawings3D; /**< Buffers incomplete 3d drawings from the debug queue. */

  ActivationGraph activationGraph;/**< Graph of active options and states. */
  unsigned activationGraphReceived = 0; /**< When was the last activation graph received? */
  std::fstream* logMessages = nullptr; /** The file messages from the robot are written to. */
  typedef std::unordered_map<char, TimeInfo> TimeInfos;
  TimeInfos timeInfos; /**< Information about the timing of modules per process. */
  Vector3f background = Vector3f(0.5f, 0.5f, 0.5f); /**< The background color of all 3-D views. */
  std::list<std::string> lines; /**< Console lines buffered because the process is currently waiting. */
  int waitingFor[numOfMessageIDs]; /**< Each entry states for how many information packages the process waits. */
  bool polled[numOfMessageIDs]; /**< Each entry states whether certain information is up-to-date (if not waiting for). */
  std::string getOrSetWaitsFor; /**< The name of the representation get or set are waiting for. If empty, they are not waiting for any. */
  bool updateCompletion = false; /**< Determines whether the tab-completion table has to be updated. */
  bool directMode = false; /**< Console is in direct mode, not replaying a script. */
  bool logImagesAsJPEGs = false; /**< Compress images before they are stored in a log file. */
  std::list<std::string> commands; /**< Commands to execute in the next update step. */

  //Joystick
  Joystick joystick; /**< The joystick interface. */
  bool joystickTrace = false; /**< Display joystick commands when executed. */
  float joystickAxisMaxSpeeds[Joystick::numOfAxes]; /**< The maximum speeds in dimensions x, y, and rotation. */
  float joystickAxisThresholds[Joystick::numOfAxes]; /**< The thresholds below which joystick commands are suppressed. */
  float joystickAxisCenters[Joystick::numOfAxes]; /**< The zero offset for each axis. */
  unsigned int joystickAxisMappings[Joystick::numOfAxes];
  unsigned joystickLastTime = 0; /**< The last time when joystick commands were handled. */
  struct JoystickMotionCommand
  {
    int indices[Joystick::numOfAxes]; /**< Indices for the sequence of $x, $y, and $r in motionCommand. */
    std::string command; /**< The pattern for the motion command. */
    std::string lastCommand; /**< The last joystick commands calculated. */
  };
  enum
  {
    joystickNumOfMotionCommands = 2 /**< Max amount of motion commands. */
  };
  JoystickMotionCommand joystickMotionCommands[joystickNumOfMotionCommands]; /**< Some motion commands. */
  std::vector<char> joystickCommandBuffer; /**< Reusable buffer for calculating the joystick command. */
  std::string joystickButtonPressCommand[Joystick::numOfButtons]; /**< Command for each button press. */
  std::string joystickButtonReleaseCommand[Joystick::numOfButtons]; /**< Command for each button release. */
  bool joystickExecCommand(const std::string&); /**< Exec command and optionally output trace to console. */

  unsigned maxPlotSize = 0; /**< The maximum number of data points to remember for plots. */
  bool kickViewSet = false; /**Indicator if there is already a KikeView, we need it just once */
  int imageSaveNumber = 0; /**< A counter for generating image file names. */
  CameraCalibratorHandler cameraCalibratorHandler;
  AutomaticCameraCalibratorHandlerInsertion automaticCameraCalibratorHandlerInsertion;
  AutomaticCameraCalibratorHandlerDeletion automaticCameraCalibratorHandlerDeletion;

public:
  char processIdentifier = 0; /** The process from which messages are currently read. */

  /**
  * Constructor.
  */
  RobotConsole(MessageQueue& in, MessageQueue& out);

  /**
  * Destructor.
  */
  ~RobotConsole();

  /**
  * That function is called once before the first main(). It can be used
  * for things that can't be done in the constructor.
  */
  virtual void init();

  /**
  * The function adds all views.
  */
  void addViews();

  /**
  * The function is called for every incoming debug message.
  */
  virtual bool handleMessage(InMessage& message);

  /**
  * The function is called when a console command has been entered.
  * @param line A string containing the console command.
  */
  void handleConsole(const std::string& line);

  /**
  * The method is called when Shift+Ctrl+letter was pressed.
  * @param key A: 0 ... Z: 25.
  * @param pressed Whether the key was pressed or released.
  */
  void handleKeyEvent(int key, bool pressed);

  /**
  * The function must be called to exchange data with SimRobot.
  * It sends the motor commands to SimRobot and acquires new sensor data.
  */
  virtual void update();

  /**
   * Prints a line to the underlying ctrl.
   */
  void printLn(const std::string& line);

  /**
  * The method returns whether the console is polling for some data from the robot.
  * @return Currently waiting?
  */
  bool isPolling() const {return !lines.empty();}

  /**
   * Sends the specified message to debugOut
   */
  void sendDebugMessage(InMessage& msg);

  /**
   * Returns the corresponding debug request string for the specified name.
   * @return empty string in case of error.
   */
  std::string getDebugRequest(const std::string& name);

  /**
   * Save current color calibration and send it to robot.
   */
  void saveColorCalibration();

protected:
  /**
   * Called by the Process (the base class) to get the
   * "debug in" message queue handled.
   */
  virtual void handleAllMessages(MessageQueue& messageQueue);

private:
  /**
  * Poll information of a certain kind if it needs updated.
  * @param id The information required.
  * @return The information requested is already up-to-date.
  */
  bool poll(MessageID id);

  /**
  * The function polls all outdated information required for and only in direct mode.
  */
  void pollForDirectMode();

  /**
  * The method triggers the processes to keep them busy or to receive updated data.
  */
  void triggerProcesses();

  void addColorSpaceViews(const std::string& id, const std::string& name, bool user, bool upperCam);

  /**
  * The function is called when a console command has been entered.
  * @param line A string containing the console command.
  * @return Was the command processed? Otherwise, it has to be processed later.
  */
  bool handleConsoleLine(const std::string& line);

  /**
  * The function prints an unfolded type to the console.
  * @param type The type to print.
  * @param field The field with that type.
  */
  void printType(std::string type, const char* field = "");

  /**
  * The function handles the joystick.
  */
  void handleJoystick();

  /**
  * The function returns the path and filename for a given representation
  * @param representation A string naming a representation
  * @return A string to the filename to the requested file
  */
  std::string getPathForRepresentation(std::string representation);

  //!@name Handler for different console commands
  //!@{
  bool msg(In&);
  bool backgroundColor(In& stream);
  bool debugRequest(In&);
  bool gameControl(In& stream);
  bool joystickCommand(In& stream);
  bool joystickSpeeds(In& stream);
  bool joystickMaps(In& stream);
  bool log(In& stream, bool first = true);
  bool get(In& stream, bool first, bool print);
  bool set(In& stream);
  bool penalizeRobot(In& stream);
  bool saveImage(In& stream);
  bool saveRequest(In& stream, bool first);
  bool sendMof(In& stream);
  bool sendWek(In& stream);
  bool repoll(In& stream);
  bool queueFillRequest(In& stream);
  bool moduleRequest(In&);
  bool moveRobot(In&);
  bool moveBall(In&);
  bool view3D(In& stream);
  bool viewField(In& stream);
  bool viewData(In& stream); /**< Creates a new representation view. Stream should contain the name of the debug data to display. */
  bool kickView();
  bool viewDrawing(In& stream, RobotConsole::Views& views, const char* type);
  bool viewImage(In& stream);
  bool viewPlot(In& stream);
  bool viewPlotDrawing(In& stream);
  bool acceptCamera(In&);
  bool setDrawingsViaProcess(In&);
  //!@}

  friend class CameraCalibratorHandler;
  friend class AutomaticCameraCalibratorHandlerInsertion;
  friend class AutomaticCameraCalibratorHandlerDeletion;
};
