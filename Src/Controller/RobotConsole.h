/*
 * @file Controller/RobotConsole.h
 *
 * Declaration of RobotConsole.
 *
 * @author Thomas Röfer
 */

#pragma once

#ifdef MACOS
#include <QGLWidget>
#define WIDGET2D QGLWidget
#else
#define WIDGET2D QWidget
#endif

#ifdef WINDOWS
#include <WinSock2.h> // This must be included first to prevent errors, since Windows.h is included in one of the following headers.
#endif

#include "Representations/BehaviorControl/ActivationGraph.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Configuration/FieldColors.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Configuration/JointLimits.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Infrastructure/SensorData/SystemSensorData.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Debugging/DebugImages.h"
#include "Tools/Framework/ThreadFrame.h"

#include "LogPlayer.h" // Must be included after ThreadFrame.h
#include "LogExtractor.h"
#include "Platform/Joystick.h"
#include "Representations/AnnotationInfo.h"
#include "Representations/ModuleInfo.h"
#include "Representations/TimeInfo.h"
#include "Views/DataView/DataView.h"
#include "Visualization/DebugDrawing.h"
#include "Visualization/DebugDrawing3D.h"

#include <QString>
#include <list>

class ConsoleRoboCupCtrl;
class ColorCalibrationView;
class ImageView;

/**
 * @class RobotConsole
 *
 * A threads that manages the text console for a single robot.
 */
class RobotConsole : public ThreadFrame
{
public:
  ENUM(Color, /**< Colors for plot drawings. Same sequence as constants in ColorRGBA (without white)! */
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
    brown,
  });

  struct ImagePtr
  {
    DebugImage* image = nullptr;
    std::string threadIdentifier;

    ~ImagePtr() { reset(); }

    void reset() { if(image) delete image; image = nullptr; }
  };

  struct Plot
  {
    std::list<float> points;
    unsigned timestamp = 0;
  };

  using Images = std::unordered_map<std::string, ImagePtr>; /**< The type of the map of images. */
  using Drawings = std::unordered_map<std::string, DebugDrawing>;
  using Drawings3D = std::unordered_map<std::string, DebugDrawing3D>;
  using Plots = std::unordered_map<std::string, Plot>;

  struct ThreadData
  {
    TimeInfo timeInfo; /**< Information collected by stopwatches. */
    Images images; /**< Debug images. */
    DrawingManager drawingManager; /**< Mappings from drawing ids to drawing names. */
    DrawingManager3D drawingManager3D; /**< Mappings from 3D drawing ids to drawing names. */
    Drawings imageDrawings; /**< Drawings on camera images. */
    Drawings fieldDrawings; /**< Drawings on the field. */
    Drawings3D drawings3D; /**< Drawings in the scene view. */
    AnnotationInfo annotationInfo; /**< Annotations collected so far or all from a log file. */
    bool logAcknowledged = true; /**< The flag is true whenever log data sent to the robot code was processed. */
  };

  struct Layer
  {
    std::string layer;
    std::string description;
    ColorRGBA color;

    bool operator==(const Layer& other) const { return layer == other.layer; }
  };

  using Views = std::unordered_map<std::string, std::list<std::string>>;
  using PlotViews = std::unordered_map<std::string, std::list<Layer>>;

  /**
   * A parametrizable command to be executed after a click on an image view.
   */
  struct ImageViewCommand
  {
    struct Token
    {
      enum Type
      {
        literal,
        placeholder
      };

      Type type;
      std::string string;
      int id;

      Token(const std::string& string) : type(literal), string(string) {}

      Token(int id) : type(placeholder), id(id) {}
    };

    std::vector<Token> tokens; /**< The tokenized command. */
    Qt::KeyboardModifiers modifierMask = Qt::NoModifier; /**< The mask with which to bitwise-and the actually pressed modifier keys before comparing them. */
    Qt::KeyboardModifiers modifiers = Qt::NoModifier; /**< The modifier keys that have to be pressed while clicking. */
  };

  using DebugDataInfoPair = std::pair<std::string, MessageQueue*>; /**< The type of the information on a debug data entry. */
  using DebugDataInfos = std::unordered_map<std::string, DebugDataInfoPair>; /**< The type of the map debug data. */

private:
  /**
   * Writes a message to a list of strings. Used to save representations as files,
   * i.e. save calibration values set in the simulator.
   */
  class MapWriter : public MessageHandler
  {
  private:
    const TypeInfo& typeInfo;
    Out& stream;

  public:
    MapWriter(const TypeInfo& typeInfo, Out& stream) : typeInfo(typeInfo), stream(stream) {}

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

  /**
   * A MessageHandler that parses debug responses containing representation data and forwards them to the correct representation view
   */
  class DataViewWriter : public MessageHandler
  {
  private:
    std::unordered_map<std::string, DataView*>* pDataViews; /**< Pointer to dataViews of RobotConsole */

  public:
    DataViewWriter(std::unordered_map<std::string, DataView*>* pViews) : pDataViews(pViews) {}

    /**
     * Forwards the specified message to the representation view that  displays it.
     */
    bool handleMessage(InMessage& message, const std::string& type, const std::string& name);
    /**
     * Same as above but it extracts the type and name from the message
     */
    bool handleMessage(InMessage& message);
  };

  /** A console command based on the joystick's axes that is excuted when no button is pressed. */
  struct JoystickMotionCommand
  {
    int indices[Joystick::numOfAxes]; /**< Indices for the sequence of $x, $y, and $r in motionCommand. */
    std::string command; /**< The pattern for the motion command. */
    std::string lastCommand; /**< The last joystick commands calculated. */
  };

public:
  DECLARE_SYNC; /**< Make this object synchronizable. */
  FieldColors colorCalibration; /**< The color calibration */
  bool colorCalibrationChanged = false; /**< Was the color calibration changed since the color table was updated? */
  unsigned colorCalibrationTimestamp = 0; /**< The last time when the last color table was updated. */
  SystemCall::Mode mode; /**< Defines mode in which this thread runs. */
  std::string logFile; /**< The name of the log file replayed. */
  std::unordered_map<std::string, ThreadData> threadData; /** Data that is maintained per thread. */
  Views fieldViews; /**< The map from field view names to the list of field drawings shown in them. */
  Views imageViews; /**< The map from image view names to the list of image drawings shown in them. */
  Plots plots; /**< Buffers for plots. */
  PlotViews plotViews; /**< The map from plot view names to the list of plots shown in them. */
  std::unordered_map<std::string, std::list<ImageViewCommand>> imageViewCommands; /**< Commands executed after a click per image view. */
  ModuleInfo moduleInfo; /**< The current state of all solution requests. */
  DebugImageConverter debugImageConverter; /**< Helper for all image view to convert debug images. */

protected:
  ConsoleRoboCupCtrl* ctrl; /** A pointer to the controller object. */
  QString robotName; /**< The name of the robot. (e.g. "Robot1") */
  LogPlayer logPlayer; /**< The log player to record and replay log files. */
  const char* pollingFor = nullptr; /**< The information the console is waiting for. */
  enum MoveOp { noMove, movePosition, moveBoth, moveBallPosition } moveOp = noMove; /**< The move operation to perform. */
  Vector3f movePos = Vector3f::Zero(); /**< The position the robot is moved to. */
  Vector3f moveRot = Vector3f::Zero(); /**< The rotation the robot is moved to. */
  bool jointCalibrationChanged = false; /**< Was the joint calibration changed since setting it for the local robot? */

  // Representations received
  JointCalibration jointCalibration; /**< The new joint calibration angles received from the robot code. */
  JointSensorData jointSensorData; /**< The most current set of joint angles received from the robot code. */
  JointRequest jointRequest; /**< The joint angles request received from the robot code. */

private:
  QString robotFullName; /**< The full name of the robot. (e.g. "RoboCup.Robot1") */
  TypeInfo typeInfo; /**< Information about all data types used by the connected robot. */
  DebugRequestTable debugRequestTable;
  DebugDataInfos debugDataInfos; /** All debug data information. */
  std::unordered_map<std::string, std::string> threadsOfDebugData; /**< From which thread was certain debug data accepted? */
  Images incompleteImages; /** Buffers images of this frame (created on demand). */
  Drawings incompleteImageDrawings; /**< Buffers incomplete image drawings from the debug queue. */
  Drawings incompleteFieldDrawings; /**< Buffers incomplete field drawings from the debug queue. */
  Drawings3D incompleteDrawings3D; /**< Buffers incomplete 3d drawings from the debug queue. */
  ColorCalibrationView* colorCalibrationView = nullptr; /**< Points to the color calibration view. */
  std::unordered_map<std::string, DataView*> dataViews; /**< The map of all data views */
  Views imageViews3D; /**< The map of all 3-D image views. */
  Vector3f background = Vector3f(0.5f, 0.5f, 0.5f); /**< The background color of all 3-D views. */
  Out* logMessages = nullptr; /** The file messages from the robot are written to. */
  std::list<std::string> lines; /**< Console lines buffered because the thread is currently waiting. */
  std::list<std::string> commands; /**< (Global) commands to execute in the next update step. */
  int waitingFor[numOfMessageIDs]; /**< Each entry states for how many information packets the thread waits. */
  bool polled[numOfMessageIDs]; /**< Each entry states whether certain information is up-to-date (if not waiting for). */
  std::string getOrSetWaitsFor; /**< The name of the representation get or set are waiting for. If empty, they are not waiting for any. */
  std::string threadIdentifier; /** The thread from which messages are currently read. */

  // Flags
  bool printMessages = true; /**< Decides whether to output text messages in the console window. */
  bool handleMessages = true; /**< Decides whether messages are handled or not. */
  bool destructed = false; /**< A flag stating that this object has already been destructed. */
  bool directMode = false; /**< Console is in direct mode, not replaying a script. */
  bool updateCompletion = false; /**< Determines whether the tab-completion table has to be updated. */
  bool moduleRequestChanged = false; /**< Was the module request changed since it was sent? */
  bool logImagesAsJPEGs = false; /**< Compress images before they are stored in a log file. */
  bool perThreadViewsAdded = false; /**< Were the per-thread views already added? */
  bool kickViewSet = false; /**Indicator if there is already a KikeView, we need it just once */

  // Helpers
  LogExtractor logExtractor; /**< The log extractor to extract things from log files. */
  DataViewWriter dataViewWriter; /**< The writer which is used to translate data into a format that can be understood by the data views */
  std::string printBuffer; /**< Buffer used to pass output of command get to kick view. */
  unsigned maxPlotSize = 0; /**< The maximum number of data points to remember for plots. */
  int imageSaveNumber = 0; /**< A counter for generating image file names. */
  int mrCounter = 0; /**< Counts the number of mr commands. */
  unsigned currentFrame = 0; /**< Counts frames for assigning them to annotations. */

  // Representations received
  ActivationGraph activationGraph;/**< Graph of active options and states. */
  unsigned activationGraphReceived = 0; /**< When was the last activation graph received? */
  TeamActivationGraph teamActivationGraph;/**< Graph of active options and states in the team behavior. */
  unsigned teamActivationGraphReceived = 0; /**< When was the last team activation graph received? */
  FsrSensorData fsrSensorData; /**< The most current set of fsr sensor data received from the robot code. */
  InertialSensorData inertialSensorData; /**< The most current set of inertial sensor data received from the robot code. */
  SystemSensorData systemSensorData; /**< The most current set of system sensor data received from the robot code. */
  unsigned sensorDataTimestamp = 0; /**< Last time new sensor data was received. */
  JointLimits jointLimits; /**< The joint calibration received from the robot code. */
  KeyStates keyStates; /**< The most current set of key states received from the robot code. */
  MotionRequest motionRequest; /**< The motion request received from the robot code. */
  RobotDimensions robotDimensions; /**< The robotDimensions received from the robot code. */
  RobotInfo robotInfo; /**< The RobotInfo received from the robot code. */

  //Joystick
  Joystick joystick; /**< The joystick interface. */
  bool joystickTrace = false; /**< Display joystick commands when executed. */
  float joystickAxisMaxSpeeds[Joystick::numOfAxes]; /**< The maximum speeds in dimensions x, y, and rotation. */
  float joystickAxisThresholds[Joystick::numOfAxes]; /**< The thresholds below which joystick commands are suppressed. */
  float joystickAxisCenters[Joystick::numOfAxes]; /**< The zero offset for each axis. */
  unsigned int joystickAxisMappings[Joystick::numOfAxes];
  unsigned joystickLastTime = 0; /**< The last time when joystick commands were handled. */
  static const int joystickNumOfMotionCommands = 2; /**< Max amount of motion commands. */
  JoystickMotionCommand joystickMotionCommands[joystickNumOfMotionCommands]; /**< Some motion commands. */
  std::vector<char> joystickCommandBuffer; /**< Reusable buffer for calculating the joystick command. */
  std::string joystickButtonPressCommand[Joystick::numOfButtons]; /**< Command for each button press. */
  std::string joystickButtonReleaseCommand[Joystick::numOfButtons]; /**< Command for each button release. */
  bool joystickExecCommand(const std::string&); /**< Exec command and optionally output trace to console. */

public:
  /**
   * The constructor.
   * Note: ThreadFrame cares about the destruction of the pointers.
   * Note: If no messaging is required pass <code>nullptr</code>.
   * @param debugReceiver The receiver of this thread.
   * @param debugSender The sender of this thread.
   */
  RobotConsole(DebugReceiver<MessageQueue>* receiver, DebugSender<MessageQueue>* sender);

  /** Destructor. */
  ~RobotConsole();

  /**
   * The function adds per-robot views.
   */
  void addPerRobotViews();

  /**
   * The function is called when a console command has been entered.
   * @param line A string containing the console command.
   */
  void handleConsole(std::string line);

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
   * Sends the specified message to debugOut
   */
  void sendDebugMessage(InMessage& msg);

  /**
   * Request debug data.
   * @param name The name of the debug data. Does not contain "debug data:".
   * @param enable Is sending the debug data enabled?
   */
  void requestDebugData(const std::string& name, bool on);

  /**
   * Returns the corresponding debug request string for the specified name.
   * @return empty string in case of error.
   */
  std::string getDebugRequest(const std::string& name);

  /**
   * Save current color calibration.
   */
  void saveColorCalibration();

protected:
  /**
   * Prints a line to the underlying ctrl.
   */
  void printLn(const std::string& line);

  /**
   * The function determines the priority of the thread.
   * @return The priority of the thread.
   */
  int getPriority() const override { return 0; }

  /**
   * That function is called once before the first main(). It can be used
   * for things that can't be done in the constructor.
   */
  void init() override;

  /**
   * The function is called when the thread is terminated.
   */
  void terminate() override {}

  /**
   * The function is called for every incoming debug message.
   *
   * @param message An interface to read the message from the queue.
   * @return Has the message been handled?
   */
  bool handleMessage(InMessage& message) override;

  /**
   * Called by the thread (the base class) to get the
   * debugReceiver message queue handled.
   */
  void handleAllMessages(MessageQueue& messageQueue) override;

  /** Retrieves all annotations from log player. */
  void updateAnnotationsFromLog();

private:
  /**
   * The function adds per-thread views.
   */
  void addPerThreadViews();

  /**
   * The method returns whether the console is polling for some data from the robot.
   * @return Currently waiting?
   */
  bool isPolling() const { return !lines.empty(); }

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
   * The method triggers the threads to keep them busy or to receive updated data.
   */
  void triggerThreads();

  void addColorSpaceViews(const std::string& id, const std::string& name, bool user, const std::string& threadIdentifier);

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
  void printType(const std::string& type, const std::string& field = "");

  /**
   * The function handles the joystick.
   */
  void handleJoystick();

  /**
   * The function returns the path and filename for a given representation
   * @param representation A string naming a representation
   * @return A string to the filename to the requested file
   */
  std::string getPathForRepresentation(const std::string& representation);

  /** Send module request to robot if it was changed. */
  void sendModuleRequest();

  //!@name Handler for different console commands
  //!@{
  bool msg(In&);
  bool backgroundColor(In& stream);
  bool debugRequest(In&);
  bool joystickCommand(In& stream);
  bool joystickSpeeds(In& stream);
  bool joystickMaps(In& stream);
  bool log(In& stream);
  bool get(In& stream, bool first, bool print);
  bool set(In& stream);
  bool saveImage(In& stream);
  bool saveRequest(In& stream, bool first);
  bool sendMof(In& stream);
  bool repoll(In& stream);
  bool moduleRequest(In&);
  bool moveRobot(In&);
  bool moveBall(In&);
  bool view3D(In& stream);
  bool viewField(In& stream);
  bool viewData(In& stream); /**< Creates a new representation view. Stream should contain the name of the debug data to display. */
  bool kickView();
  bool viewDrawing(In& stream, RobotConsole::Views& views, const char* type);
  bool viewImage(In& stream);
  bool viewImageCommand(In& stream);
  bool viewPlot(In& stream);
  bool viewPlotDrawing(In& stream);
  //!@}
};
