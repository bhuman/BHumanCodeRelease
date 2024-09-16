/*
 * @file SimulatedNao/RobotConsole.h
 *
 * Declaration of RobotConsole.
 *
 * @author Thomas Röfer
 */

#pragma once

#ifdef WINDOWS
#include <WinSock2.h> // This must be included first to prevent errors, since Windows.h is included in one of the following headers.
#endif

#include "Debugging/DebugDrawings3D.h"
#include "Debugging/DebugImages.h"
#include "Framework/ThreadFrame.h"
#include "LogExtractor.h"
#include "LogPlayer.h"
#include "Platform/Joystick.h"
#include "Representations/AnnotationInfo.h"
#include "Representations/BehaviorControl/ActivationGraph.h"
#include "Representations/BehaviorControl/JoystickState.h"
#include "Representations/BehaviorControl/SharedAutonomyRequest.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/RawInertialSensorData.h"
#include "Representations/ModuleInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/TimeInfo.h"
#include "Streaming/OutStreams.h"
#include "Tools/Communication/SharedAutonomyChannel.h"
#include "Views/DataView/DataView.h"
#include "Visualization/DebugDrawing.h"
#include "Visualization/DebugDrawing3D.h"
#include "Visualization/DebugDrawing3DAdapter.h"
#include "Visualization/DebugImageConverter.h"

#include <QString>
#include <list>

class ConsoleRoboCupCtrl;
class ImageView;
class SimulatedRobot;
class SACControlWidget;

/**
 * @class RobotConsole
 *
 * A threads that manages the text console for a single robot.
 */
class RobotConsole : public ThreadFrame
{
  friend class SACControlWidget;
private:
  /** A console command based on the joystick's axes that is executed when no button is pressed. */
  struct JoystickMotionCommand
  {
    int indices[Joystick::numOfAxes]; /**< Indices for the sequence of $x, $y, and $r in motionCommand. */
    std::string command; /**< The pattern for the motion command. */
    std::string lastCommand; /**< The last joystick commands calculated. */
  };

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
    unsigned timestamp = 0;
    std::string threadName;

    ~ImagePtr() { delete image; }
  };

  struct Plot
  {
    std::list<float> points;
    unsigned timestamp = 0;
  };

  using Images = std::unordered_map<std::string, ImagePtr>; /**< The type of the map of images. */
  using Drawings = std::unordered_map<std::string, DebugDrawing>;
  using Drawings3D = std::unordered_map<std::string, DebugDrawing3D>;
  using DrawingAdapters3D = std::unordered_map<std::string, DebugDrawing3DAdapter>;
  using Plots = std::unordered_map<std::string, Plot>;
  using DebugDataInfoPair = std::pair<std::string, MessageQueue*>; /**< The type of the information on a debug data entry. */
  using DebugDataInfos = std::unordered_map<std::string, DebugDataInfoPair>; /**< The type of the map debug data. */

  struct ThreadData
  {
    TimeInfo timeInfo; /**< Information collected by stopwatches. */
    Images images; /**< Debug images. */
    DrawingManager drawingManager; /**< Mappings from drawing ids to drawing names. */
    DrawingManager3D drawingManager3D; /**< Mappings from 3D drawing ids to drawing names. */
    Drawings imageDrawings; /**< Drawings on camera images. */
    Drawings fieldDrawings; /**< Drawings on the field. */
    DrawingAdapters3D drawings3D; /**< Drawings in the scene view. */
    Plots plots; /**< Buffers for plots. */
    AnnotationInfo annotationInfo; /**< Annotations collected so far or all from a log file. */
    DebugRequestTable debugRequestTable; /**< Debug requests available in this thread. */
    DebugDataInfos debugDataInfos; /**< All debug data information in this thread. */
    std::unordered_map<std::string, DataView*> dataViews; /**< The map of all data views for this thread. */
    std::string getOrSetWaitsFor; /**< The name of the representation get or set are waiting for. If empty, they are not waiting for any. */
    bool logAcknowledged = true; /**< The flag is true whenever log data sent to the robot code was processed. */
    size_t currentFrame = 0; /**< The last frame that was played back for this thread (log replay only). */
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

public:
  DECLARE_SYNC; /**< Make this object synchronizable. */
  const SystemCall::Mode mode; /**< Defines the mode in which this thread runs. */
  std::string logFile; /**< The name of the log file replayed. */
  std::unordered_map<std::string, ThreadData> threadData; /**< Data that is maintained per thread. */
  Views fieldViews; /**< The map from field view names to the list of field drawings shown in them. */
  Views imageViews; /**< The map from image view names to the list of image drawings shown in them. */
  PlotViews plotViews; /**< The map from plot view names to the list of plots shown in them. */
  std::unordered_map<std::string, std::list<ImageViewCommand>> imageViewCommands; /**< Commands executed after a click per image view. */
  ModuleInfo moduleInfo; /**< The current state of all solution requests. */
  DebugImageConverter debugImageConverter; /**< Helper for all image view to convert debug images. */
  ConsoleRoboCupCtrl* ctrl; /**< A pointer to the controller object. */
  SharedAutonomyRequest sharedAutonomyRequest; /**< The command sent to the robot */
  unsigned timeSharedAutonomyRequestSent = 0; /**< When was the command sent? Set to 0 to force sending immediately. */

protected:
  LogPlayer logPlayer; /**< The log player to record and replay log files. */
  const char* pollingFor = nullptr; /**< The information the console is waiting for. */
  bool jointCalibrationChanged = false; /**< Was the joint calibration changed since setting it for the local robot? */
  std::unique_ptr<SimulatedRobot> simulatedRobot; /**< The interface to simulated objects. */

  // Representations received
  JointCalibration jointCalibration; /**< The new joint calibration angles received from the robot code. */
  JointSensorData jointSensorData; /**< The most current set of joint angles received from the robot code. */
  JointRequest jointRequest; /**< The joint angles request received from the robot code. */
  MotionRequest motionRequest; /**< The motion request received from the robot code. */
  RawInertialSensorData rawInertialSensorData; /**< The most current set of inertial sensor data received from the robot code. */

private:
  TypeInfo typeInfo; /**< Information about all data types used by the connected robot. */
  DebugRequestTable debugRequestTable;
  Images incompleteImages; /**< Buffers images of this frame (created on demand). */
  Drawings incompleteImageDrawings; /**< Buffers incomplete image drawings from the debug queue. */
  Drawings incompleteFieldDrawings; /**< Buffers incomplete field drawings from the debug queue. */
  Drawings3D incompleteDrawings3D; /**< Buffers incomplete 3d drawings from the debug queue. */
  Views imageViews3D; /**< The map of all 3-D image views. */
  Vector3f background = Vector3f(0.5f, 0.5f, 0.5f); /**< The background color of all 3-D views. */
  Out* logMessages = nullptr; /**< The file messages from the robot are written to. */
  std::list<std::string> lines; /**< Console lines buffered because the thread is currently waiting. */
  std::list<std::string> commands; /**< (Global) commands to execute in the next update step. */
  int waitingFor[numOfMessageIDs]; /**< Each entry states for how many information packets the thread waits. */
  bool polled[numOfMessageIDs]; /**< Each entry states whether certain information is up-to-date (if not waiting for). */
  std::string threadName; /**< The thread from which messages are currently read. */

  // Flags
  bool printMessages = true; /**< Decides whether to output text messages in the console window. */
  bool handleMessages = true; /**< Decides whether messages are handled or not. */
  bool destructed = false; /**< A flag stating that this object has already been destructed. */
  bool directMode = false; /**< Console is in direct mode, not replaying a script. */
  bool updateCompletion = false; /**< Determines whether the tab-completion table has to be updated. */
  bool updateDataViews = false; /**< Determines whether the set of data views has to be updated. */
  bool moduleRequestChanged = false; /**< Was the module request changed since it was sent? */
  bool perThreadViewsAdded = false; /**< Were the per-thread views already added? */
  bool newDebugDrawing3D = false; /**< Whether a new 3D debug drawing must be registered. */

  // Helpers
  LogExtractor logExtractor; /**< The log extractor to extract things from log files. */
  unsigned maxPlotSize = 0; /**< The maximum number of data points to remember for plots. */
  int imageSaveNumber = 0; /**< A counter for generating image file names. */
  int mrCounter = 0; /**< Counts the number of mr commands. */
  unsigned currentFrame = 1; /**< Counts frames for assigning them to annotations. */

  // Representations received
  ActivationGraph activationGraph;/**< Graph of active options and states. */
  unsigned activationGraphReceived = 0; /**< When was the last activation graph received? */

  //Joystick
  std::unique_ptr<Joystick> joystick; /**< The joystick interface. */
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
  JoystickState joystickState; /**< The last joystick state measured. */

  std::unique_ptr<SharedAutonomyChannel> sharedAutonomyChannel; /**< Channel to remote robot in shared autonomy challenge. */

public:
  /**
   * The constructor.
   * Note: ThreadFrame cares about the destruction of the pointers.
   * Note: If no messaging is required pass <code>nullptr</code>.
   * @param settings The settings for this thread.
   * @param robotName The name of the robot this thread belongs to.
   * @param ctrl A pointer to the controller object.
   * @param mode The mode of the robot.
   * @param debugReceiver The receiver of this thread.
   * @param debugSender The sender of this thread.
   */
  RobotConsole(const Settings& settings, const std::string& robotName, ConsoleRoboCupCtrl* ctrl, SystemCall::Mode mode, DebugReceiver<MessageQueue>* receiver, DebugSender<MessageQueue>* sender);

  /** Destructor. */
  ~RobotConsole();

  /**
   * The function must be called to exchange data with SimRobot.
   * It sends the motor commands to SimRobot and acquires new sensor data.
   */
  virtual void update();

  /** The function adds per-robot views. */
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
   * Request debug data.
   * @param threadName The name of the thread the request is sent to.
   * @param name The name of the debug data. Does not contain "debug data:".
   */
  void requestDebugData(const std::string& threadName, const std::string& name);

  /**
   * Send debug data to overwrite data in a specific thread.
   * @param threadName The thread the data is sent to.
   * @param name The name of the data to overwrite.
   * @param data The data to be sent. If nullptr, stop overwriting data.
   */
  void sendDebugData(const std::string& threadName, const std::string& name, OutBinaryMemory* data = nullptr);

protected:
  /**
   * The function determines the priority of the thread.
   * @return The priority of the thread.
   */
  int getPriority() const override { return 0; }

  /** The function is called once before the first call of the thread's main(). */
  void init() override;

  /** The function is called when the thread is terminated. */
  void terminate() override {}

  /**
   * The function is called for every incoming debug message.
   * @param message An interface to read the message from the queue.
   * @return Has the message been handled?
   */
  bool handleMessage(MessageQueue::Message message) override;

  /**
   * Called by the thread to handle all messages received.
   * @param The message queue of the thread.
   */
  void handleAllMessages(MessageQueue& messageQueue) override;

  /** Retrieves all annotations from the log player. */
  void updateAnnotationsFromLog();

private:
  /** The function adds all per-thread views, but not. */
  void addPerThreadViews();

  /**
   * The function is called by \c view3D to add a set of color space views.
   * @param path The path in the scene view.
   * @param name The name of the image shown by the views.
   * @param threadName The name of the thread the views are added for.
   */
  void addColorSpaceViews(const QString& path, const std::string& name, const std::string& threadName);

  /** Adds and removes data views for the current module configuration. */
  void updateDataViewTree();

  /**
   * The function is called when a console command has been entered.
   * @param line A string containing the console command.
   * @return Was the command processed? Otherwise, it has to be processed later.
   */
  bool handleConsoleLine(const std::string& line);

  /**
   * Handles all commands that can be prefixed by "for".
   * @param command The name of the command.
   * @param stream The stream containing the parameters of the command.
   * @param threadName The name of the thread the command runs for.
   * @param result The result of parsing the command.
   * @return Were all prerequisites satisfied for executing the command?
   */
  bool handleForCommands(const std::string& command, In& stream,
                         const std::string& threadName, bool& result);

  /** Initialize joystick if it has not been yet. */
  void initJoystick();

  /** The function handles the joystick. */
  void handleJoystick();

  /**
   * Executes a console command that was initiated by joystick input.
   * @param cmd The command.
   * @return Was the command executed (vs. was it empty)?
   */
  bool handleJoystickCommand(const std::string& cmd);

  /** Handle messages from remote robot in the shared autonomy challenge. */
  void handleSharedAutonomyMessages();

  /**
   * Poll information of a certain kind if it needs updated.
   * @param id The information required.
   * @return The information requested is already up-to-date.
   */
  bool poll(MessageID id);

  /** The function polls all outdated information required for and only in direct mode. */
  void pollForDirectMode();

  /** The method triggers the threads to keep them busy or to receive updated data. */
  void triggerThreads();

  /**
   * The function prints an unfolded type to the console.
   * @param type The type to print.
   * @param field The field with that type.
   */
  void printType(const std::string& type, const std::string& field = "");

  /**
   * Notifies all relevant views if a "set" or "set unchanged" command was executed.
   * @param data The name of the data for which it was executed.
   * @param set Was "set" executed (vs. "set unchanged")?
   * @param threadName The name of the thread for which the status changed.
   *                   Empty if it changes for all threads.
   */
  void notifyDataViewsAboutSetStatus(const std::string& data, bool set, const std::string& threadName);

  /**
   * The function returns the path and filename for a given representation
   * @param representation A string naming a representation
   * @return A string to the filename to the requested file
   */
  std::string getPathForRepresentation(const std::string& representation) const;

  //!@name Handler for different console commands
  //!@{
  bool backgroundColor(In&);
  bool debugRequest(In&, const std::string& threadName);
  bool for_(In&, bool& result);
  bool get(In&, const std::string& threadName, bool first);
  bool joystickCommand(In&);
  bool joystickMaps(In&);
  bool joystickSpeeds(In&);
  bool log(In&);
  bool moduleRequest(In&, std::string threadName);
  bool moveBall(In&);
  bool moveRobot(In&);
  bool msg(In&);
  bool penalizeRobot(In&);
  bool repoll(In&);
  bool saveImage(In&, std::string threadName);
  bool sensorNoise(In&);
  bool set(In&, const std::string& threadName);
  bool sharedAutonomyChallenge(In&);
  bool viewDrawing(In&, Views& views, const char* type);
  bool viewField(In&, const std::string& threadName);
  bool viewImage(In&, std::string threadName);
  bool viewImageCommand(In&);
  bool viewPlot(In&, const std::string& threadName);
  bool viewPlotDrawing(In&);
  //!@}
};
