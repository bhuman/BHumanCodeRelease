/*
 * @file Controller/RobotConsole.h
 *
 * Declaration of RobotConsole.
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
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
#include "Representations/Configuration/FieldColors.h"
#include "Representations/Configuration/JointLimits.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Infrastructure/SensorData/SystemSensorData.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Debugging/DebugImages.h"
#include "Tools/ProcessFramework/Process.h"

#include "LogPlayer.h" // Must be included after Process.h
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
  FieldColors colorCalibration; /**< The color calibration */
  bool colorCalibrationChanged = false; /**< Was the color calibration changed since the color table was updated? */
  unsigned colorCalibrationTimeStamp = 0; /**< The last time when the last color table was updated. */
  ColorCalibrationView* colorCalibrationView = nullptr;
  JointCalibration jointCalibration; /**< The new joint calibration angles received from the robot code. */
  bool jointCalibrationChanged = false; /**< Was the joint calibration changed since setting it for the local robot? */
  std::vector<ImageView*> segmentedImageViews;
  SystemCall::Mode mode; /**< Defines mode in which this process runs. */
  std::string logFile; /**< The name of the log file replayed. */

protected:
  ConsoleRoboCupCtrl* ctrl; /** A pointer to the controller object. */
  QString robotFullName; /**< The full name of the robot. (e.g. "RoboCup.Robot1") */
  QString robotName; /**< The name of the robot. (e.g. "Robot1") */
  bool printMessages = true; /**< Decides whether to output text messages in the console window. */
  bool handleMessages = true; /**< Decides whether messages are handled or not. */
  bool logAcknowledged = true; /**< The flag is true whenever log data sent to the robot code was processed. */
  bool destructed = false; /**< A flag stating that this object has already been destructed. */
  LogPlayer logPlayer; /**< The log player to record and replay log files. */
  LogExtractor logExtractor; /**< The log extractor to extract things from log files. */
  MessageQueue& debugOut; /**< The outgoing debug queue. */
  TypeInfo typeInfo; /**< Information about all data types used by the connected robot. */
  DrawingManager drawingManager;
  DrawingManager3D drawingManager3D;
  DebugRequestTable debugRequestTable;
  const char* pollingFor = nullptr; /**< The information the console is waiting for. */
  RobotInfo robotInfo; /**< The RobotInfo received from the robot code. */
  JointRequest jointRequest; /**< The joint angles request received from the robot code. */
  JointSensorData jointSensorData; /**< The most current set of joint angles received from the robot code. */
  FsrSensorData fsrSensorData; /**< The most current set of fsr sensor data received from the robot code. */
  InertialSensorData inertialSensorData; /**< The most current set of inertial sensor data received from the robot code. */
  KeyStates keyStates; /**< The most current set of key states received from the robot code. */
  SystemSensorData systemSensorData; /**< The most current set of system sensor data received from the robot code. */
  unsigned sensorDataTimeStamp = 0; /**< Last time new sensor data was received. */
  enum MoveOp { noMove, movePosition, moveBoth, moveBallPosition } moveOp = noMove; /**< The move operation to perform. */
  Vector3f movePos = Vector3f::Zero(); /**< The position the robot is moved to. */
  Vector3f moveRot = Vector3f::Zero(); /**< The rotation the robot is moved to. */
  MotionRequest motionRequest; /**< Motion request for the kick view. */
  int mrCounter = 0; /**< Counts the number of mr commands. */
  JointLimits jointLimits; /**< The joint calibration received from the robot code. */
  RobotDimensions robotDimensions; /**< The robotDimensions received from the robot code. */
  std::string printBuffer; /**< Buffer used for command get. */
  char drawingsViaProcess = 'b'; /** Which process is used to provide field and 3D drawings */
  AnnotationInfo annotationInfos;

public:
  class ImagePtr
  {
  public:
    DebugImage* image = nullptr;
    char processIdentifier = 0; /**< "c" denotes lower camera process, "d" denotes upper camera process */

    ~ImagePtr() { reset(); }

    void reset() { if(image) delete image; image = nullptr; }
  };
  using Images = std::unordered_map<std::string, ImagePtr>; /**< The type of the map of images. */

  Images upperCamImages;
  Images lowerCamImages;

  using Drawings = std::unordered_map<std::string, DebugDrawing>;
  Drawings upperCamImageDrawings, lowerCamImageDrawings, motionImageDrawings; /**< Buffers for image drawings from the debug queue. */
  Drawings upperCamFieldDrawings, lowerCamFieldDrawings, motionFieldDrawings; /**< Buffers for field drawings from the debug queue. */
  using Drawings3D = std::unordered_map<std::string, DebugDrawing3D>;
  Drawings3D upperCamDrawings3D, lowerCamDrawings3D, motionDrawings3D; /**< Buffers for 3d drawings from the debug queue. */
  DebugImageConverter debugImageConverter;

  Images* currentImages = nullptr;
  Drawings* currentImageDrawings = nullptr;
  Drawings* currentFieldDrawings = nullptr;
  Drawings3D* currentDrawings3D = nullptr;

  struct Plot
  {
    std::list<float> points;
    unsigned timeStamp = 0;
  };

  using Plots = std::unordered_map<std::string, Plot>;
  Plots plots; /**< Buffers for plots from the debug queue. */

  struct Layer
  {
    std::string layer;
    std::string description;
    ColorRGBA color;

    bool operator==(const Layer& other) const { return layer == other.layer; }
  };

  using PlotViews = std::unordered_map<std::string, std::list<Layer>>;
  using Views = std::unordered_map<std::string, std::list<std::string>>;
  PlotViews plotViews; /**< The map of all plot views. */

  Views fieldViews; /**< The map of all field views. */
  Views imageViews; /**< The map of all image views. */
  Views imageViews3D; /**< The map of all 3-D image views. */
  ModuleInfo moduleInfo; /**< The current state of all solution requests. */

  /**List of currently active representation views. Key: representation name, value: pointer to the view */
  std::map<std::string, DataView*> dataViews;

  std::map<std::string, ImageView*> actualImageViews;

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
      Token(const std::string& string) :
        type(literal), string(string)
      {}
      Token(int id) :
        type(placeholder), id(id)
      {}
      Type type;
      std::string string;
      int id;
    };
    std::vector<Token> tokens; /**< The tokenized command. */
    Qt::KeyboardModifiers modifierMask = Qt::NoModifier; /**< The mask with which to bitwise-and the actually pressed modifier keys before comparing them. */
    Qt::KeyboardModifiers modifiers = Qt::NoModifier; /**< The modifier keys that have to be pressed while clicking. */
  };
  /** The commands to be executed after a click per image view. */
  std::unordered_map<std::string, std::list<ImageViewCommand>> imageViewCommands;

  /**
   * A MessageHandler that parses debug responses containing representation data and forwards them to the correct representation view
   */
  class DataViewWriter : public MessageHandler
  {
  private:
    std::map<std::string, DataView*>* pDataViews; /**< Pointer to dataViews of RobotConsole */

  public:
    DataViewWriter(std::map<std::string, DataView*>* pViews) : pDataViews(pViews) {}

    /**
     * Forwards the specified message to the representation view that  displays it.
     */
    bool handleMessage(InMessage& message, const std::string& type, const std::string& name);
    /**
     * Same as above but it extracts the type and name from the message
     */
    bool handleMessage(InMessage& message);
  };
  DataViewWriter dataViewWriter; /**< The writer which is used to translate data into a format that can be understood by the data views */

private:
  using DebugDataInfoPair = std::pair<std::string, MessageQueue*>; /**< The type of the information on a debug data entry. */
  using DebugDataInfos = std::unordered_map<std::string, DebugDataInfoPair>; /**< The type of the map debug data. */
  DebugDataInfos debugDataInfos; /** All debug data information. */
  std::unordered_map<std::string, unsigned char> processesOfDebugData; /**< From which process was certain debug data accepted? */

  Images incompleteImages; /** Buffers images of this frame (created on demand). */
  Drawings incompleteImageDrawings; /**< Buffers incomplete image drawings from the debug queue. */
  Drawings incompleteFieldDrawings; /**< Buffers incomplete field drawings from the debug queue. */
  Drawings3D incompleteDrawings3D; /**< Buffers incomplete 3d drawings from the debug queue. */

  ActivationGraph activationGraph;/**< Graph of active options and states. */
  unsigned activationGraphReceived = 0; /**< When was the last activation graph received? */
  Out* logMessages = nullptr; /** The file messages from the robot are written to. */
  using TimeInfos = std::unordered_map<char, TimeInfo>;
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

public:
  char processIdentifier = 0; /** The process from which messages are currently read. */

  RobotConsole(MessageQueue& in, MessageQueue& out);
  ~RobotConsole();

  /**
   * That function is called once before the first main(). It can be used
   * for things that can't be done in the constructor.
   */
  void init() override;

  /**
   * The function adds all views.
   */
  void addViews();

  /**
   * The function is called for every incoming debug message.
   */
  bool handleMessage(InMessage& message) override;

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
   * Prints a line to the underlying ctrl.
   */
  void printLn(const std::string& line);

  /**
   * The method returns whether the console is polling for some data from the robot.
   * @return Currently waiting?
   */
  bool isPolling() const { return !lines.empty(); }

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
   * Called by the Process (the base class) to get the
   * "debug in" message queue handled.
   */
  void handleAllMessages(MessageQueue& messageQueue) override;

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
  bool viewImageCommand(In& stream);
  bool viewPlot(In& stream);
  bool viewPlotDrawing(In& stream);
  bool acceptCamera(In&);
  bool setDrawingsViaProcess(In&);
  //!@}
};
