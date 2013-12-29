/*
* @file Controller/RobotConsole.h
*
* Declaration of RobotConsole.
*
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

#include <QString>
#include <fstream>
#include <list>

#include "Tools/ProcessFramework/Process.h"
#include "Platform/Joystick.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Infrastructure/LEDRequest.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Infrastructure/USRequest.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/FreePartOfOpponentGoalModel.h"
#include "Representations/Modeling/RobotsModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/CombinedWorldModel.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Perception/LinePercept.h"
#include "Representations/Perception/ColorReference.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/BehaviorControl/ActivationGraph.h"
#include "Representations/Sensing/RobotBalance.h"
#include "LogPlayer.h"
#include "Tools/Debugging/DebugImages.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Visualization/DebugDrawing.h"
#include "Visualization/DebugDrawing3D.h"
#include "Representations/ModuleInfo.h"
#include "Representations/TimeInfo.h"
#include "Controller/CameraCalibratorHandler.h"
#include "Views/DataView/DataView.h"
#include "Views/FootView.h"
#include "Views/ColorCalibrationView/ColorCalibrationView.h"

class ConsoleRoboCupCtrl;
class ColorCalibrationView;

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
  DECLARE_SYNC; /**< Make this object synchronizable. */
  ColorReference colorReference; /**< The color reference */
  unsigned colorReferenceTimeStamp; /**< The time when the last color reference was received. */
  unsigned colorReferenceChangedTimeStamp; /**< The time when the ColorReference was changed by the ColorCalibrator. */
  ColorCalibrationView* colorCalibrator; /**< The ColorCalibrator tool. */

protected:
  ConsoleRoboCupCtrl* ctrl; /** A pointer to the controller object. */
  QString robotFullName; /**< The full name of the robot. (e.g. "RoboCup.Robot1") */
  QString robotName; /**< The name of the robot. (e.g. "Robot1") */
  SystemCall::Mode mode; /**< Defines mode in which this process runs. */
  bool printMessages, /**< Decides whether to output text messages in the console window. */
       handleMessages, /**< Decides whether messages are handled or not. */
       logAcknowledged, /**< The flag is true whenever log data sent to the robot code was processed. */
       destructed; /**< A flag stating that this object has already been destructed. */
  std::string logFile; /**< The name of the log file replayed. */
  LogPlayer logPlayer; /**< The log player to record and replay log files. */
  MessageQueue& debugOut; /**< The outgoing debug queue. */
  StreamHandler streamHandler; /**< Local stream handler. Note: Process::streamHandler may be accessed unsynchronized in different thread, so don't use it here. */
  DrawingManager drawingManager;
  DrawingManager3D drawingManager3D;
  DebugRequestTable debugRequestTable;
  const char* pollingFor; /**< The information the console is waiting for. */
  JointData jointRequest; /**< The joint angles request received from the robot code. */
  JointData jointData; /**< The most current set of joint angles sent. */
  LEDRequest ledRequest; /**< The led state request received from the robot code. */
  enum MoveOp {noMove, movePosition, moveBoth, moveBallPosition} moveOp; /**< The move operation to perform. */
  Vector3<> movePos; /**< The position the robot is moved to. */
  Vector3<> moveRot; /**< The rotation the robot is moved to. */
  ObstacleModel obstacleModel; /**< Obstacle model from team communication. */
  FreePartOfOpponentGoalModel freePartOfOpponentGoalModel; /**< Free part of opponent goal model from team communication. */
  RobotsModel robotsModel; /**< Robots model from team communication. */
  RobotPose robotPose; /**< Robot pose from team communication. */
  BallModel ballModel; /**< Ball model from team communication. */
  CombinedWorldModel combinedWorldModel; /**< combined world model from team communication */
  GoalPercept goalPercept; /**< Goal percept from team communication. */
  LinePercept linePercept; /**< Line percept from team communication. */
  BehaviorStatus behaviorStatus; /**< Behavior data from team communication. */
  OdometryData odometryData; /**< Odometry data from logfile. */
  RobotHealth robotHealth; /**< Robot Health from team communication. */
  MotionRequest motionRequest; /**< Motion Request from team communication. */
  bool isPenalized, /**< Penalized state from team communication. */
       hasGroundContact, /**< Ground contact state from team communication. */
       isUpright; /**<fall down state from team communication */
  unsigned freePartOfOpponentGoalModelReceived, /**< When was the free part of opponent goal model received from team communication. */
           obstacleModelReceived, /**< When was the obstacle model received from team communication. */
           robotsModelReceived, /**< When was the robots model received from team communication. */
           robotPoseReceived, /**< When was the robot pose received from team communication. */
           ballModelReceived, /**< When was the ball model received from team communication. */
           goalPerceptReceived, /**< When was the goal percept received from team communication. */
           linePerceptReceived, /**< When was the line percept received from team communication. */
           robotHealthReceived, /**< When was the robot health received from team communication. */
           motionRequestReceived, /**< When was the motion request received from team communication. */
           isPenalizedReceived, /**< When was the penalized state received from team communication. */
           hasGroundContactReceived, /**< When was the ground contact state received from team communication. */
           combinedWorldModelReceived, /**< When was the combined world model received from team communication. */
           isUprightReceived; /**< When was the fall down state received from team communication. */
  SensorData sensorData; /**< The most current sensor data sent. */
  FilteredSensorData filteredSensorData; /**< The received filtered sensor data. */

  JointCalibration jointCalibration; /**< The joint calibration received from the robot code. */
  RobotDimensions robotDimensions; /**< The robotDimensions received from the robot code. */
  RobotBalance robotBalance; /**< Balance information received from the robot code, including the zero-moment point. */
  USRequest usRequest;  /**< The current us request received from the robot code (for simulation). */
  std::string printBuffer; /**< Buffer used for command get. */
  char drawingsViaProcess; /** Which process is used to provide field and 3D drawings */

public:
  class ImagePtr
  {
  public:
    Image* image;
    char processIdentifier;
    ImagePtr() : image(0), processIdentifier(0) {}
    ~ImagePtr() {reset();}
    void reset() {if(image) delete image; image = 0;}
  };
  typedef std::unordered_map<std::string, ImagePtr> Images; /**< The type of the map of images. */

  Images upperCamImages;
  Images lowerCamImages;

  typedef std::unordered_map<std::string, DebugDrawing> Drawings;
  Drawings upperCamImageDrawings, lowerCamImageDrawings, /**< Buffers for image drawings from the debug queue. */
           upperCamFieldDrawings, lowerCamFieldDrawings; /**< Buffers for field drawings from the debug queue. */
  typedef std::unordered_map<std::string, DebugDrawing3D> Drawings3D;
  Drawings3D upperCamDrawings3D, lowerCamDrawings3D; /**< Buffers for 3d drawings from the debug queue. */

  Images* currentImages;
  Drawings* currentImageDrawings;
  Drawings* currentFieldDrawings;
  Drawings3D* currentDrawings3D;

  class Plot
  {
  public:
    std::list<float> points;
    unsigned timeStamp;

    Plot() : timeStamp(0) {}
  };

  typedef std::unordered_map<std::string, Plot> Plots;
  Plots plots; /**< Buffers for plots from the debug queue. */

  class Layer
  {
  public:
    std::string layer;
    std::string description;
    ColorRGBA color;

    bool operator==(const Layer& other) const {return layer == other.layer;}
  };

  typedef std::unordered_map<std::string, std::list<Layer> > PlotViews;
  typedef std::unordered_map<std::string, std::list<std::string> > Views;
  PlotViews plotViews; /**< The map of all plot views. */

  Views fieldViews, /**< The map of all field views. */
        imageViews, /**< The map of all image views. */
        imageViews3D; /**< The map of all 3-D image views. */
  ModuleInfo moduleInfo; /**< The current state of all solution requests. */

  /**List of currently active representation views. Key: representation name, value: pointer to the view */
  std::map<std::string, DataView*> representationViews;

  std::map<std::string, void*> actualImageViews;

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
  Drawings incompleteImageDrawings, /**< Buffers incomplete image drawings from the debug queue. */
           incompleteFieldDrawings; /**< Buffers incomplete field drawings from the debug queue. */
  Drawings3D incompleteDrawings3D; /**< Buffers incomplete 3d drawings from the debug queue. */

  ActivationGraph activationGraph ;/**< Graph of active options and states. */
  unsigned activationGraphReceived; /**< When was the last activation graph received? */
  std::fstream* logMessages; /** The file messages from the robot are written to. */
  typedef std::unordered_map<char, TimeInfo> TimeInfos;
  TimeInfos timeInfos; /**< Information about the timing of modules per process. */
  Vector3<> background; /**< The background color of all 3-D views. */
  std::list<std::string> lines; /**< Console lines buffered because the process is currently waiting. */
  int waitingFor[numOfMessageIDs]; /**< Each entry states for how many information packages the process waits. */
  bool polled[numOfMessageIDs]; /**< Each entry states whether certain information is up-to-date (if not waiting for). */
  std::string getOrSetWaitsFor; /**< The name of the representation get or set are waiting for. If empty, they are not waiting for any. */
  bool updateCompletion, /**< Determines whether the tab-completion table has to be updated. */
       directMode, /**< Console is in direct mode, not replaying a script. */
       logImagesAsJPEGs; /**< Compress images before they are stored in a log file. */
  std::list<std::string> commands; /**< Commands to execute in the next update step. */

  //Joystick
  Joystick joystick; /**< The joystick interface. */
  bool joystickTrace; /**< Display joystick commands when executed. */
  float joystickAxisMaxSpeeds[Joystick::numOfAxes], /**< The maximum speeds in dimensions x, y, and rotation. */
        joystickAxisThresholds[Joystick::numOfAxes], /**< The thresholds below which joystick commands are suppressed. */
        joystickAxisCenters[Joystick::numOfAxes]; /**< The zero offset for each axis. */
  unsigned int joystickAxisMappings[Joystick::numOfAxes];
  unsigned joystickLastTime; /**< The last time when joystick commands were handled. */
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

  unsigned maxPlotSize; /**< The maximum number of data points to remember for plots. */
  bool bikeView; /**Indicator if there is already a BikeView, we need it just once */
  int imageSaveNumber; /**< A counter for generating image file names. */
  CameraCalibratorHandler cameraCalibratorHandler;

public:
  char processIdentifier; /** The process from which messages are currently read. */

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

  void addColorSpaceViews(const std::string& id, const std::string& name, bool user);

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
  void printType(const char* type, const char* field = "");

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
  bool log(In& stream);
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

  /**
   * Only call this method if you used SYNC before.
   * Otherwise use moduleRequest()!
   */
  bool moduleRequestWithoutSync(In&);
  bool moveRobot(In&);
  bool moveBall(In&);
  bool view3D(In& stream);
  bool viewField(In& stream);
  bool viewData(In& stream); /**< Creates a new representation view. Stream should contain the name of the debug data to display. */
  bool viewBike();
  bool viewDrawing(In& stream, RobotConsole::Views& views, const char* type);
  bool viewImage(In& stream);
  bool viewPlot(In& stream);
  bool viewPlotDrawing(In& stream);
  bool acceptCamera(In&);
  bool setDrawingsViaProcess(In&);
  //!@}

  friend class CameraCalibratorHandler;
};
