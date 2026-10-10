/**
 * @file Main.cpp
 * Implementation of the main() function for starting and stopping the module framework.
 * @author Colin Graf
 * @author Thomas Röfer
 */

#include <booster/idl/b1/ButtonEvent.h>
#include <booster/idl/b1/LowCmd.h>
#include <booster/idl/b1/LowState.h>
#include <booster/idl/b1/RobotStatusDdsMsg.h>
#include <booster/robot/b1/b1_loco_client.hpp>
#include <booster/robot/channel/channel_publisher.hpp>
#include <booster/robot/channel/channel_subscriber.hpp>
#include <booster/robot/device/light/light_control_client.hpp>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <sys/file.h> // flock
#include <fcntl.h>
#include <pthread.h>
#include <sys/wait.h>
#include <unistd.h>

#include "Framework/Robot.h"
#include "Framework/Robots.h"
#include "Framework/Settings.h"
#include "Math/Constants.h"
#include "Platform/File.h"
#include "Platform/Semaphore.h"
#include "Platform/SystemCall.h"
#include "Platform/Thread.h"
#include "Platform/Time.h"
#include "Streaming/FunctionList.h"

static pid_t bhumanPid = 0;
static Robot* robot = nullptr;
static bool run = true;
static pthread_t mainThread;

// Used by BoosterProvider
namespace Main
{
  booster::robot::b1::B1LocoClient client;
  booster::robot::light::LightControlClient ledControl;
}
using namespace Main;

static bool ledControlAvailable = false;
static unsigned timeWhenAskedToTerminate = 0;

SystemCall::Mode SystemCall::getMode()
{
  return physicalRobot;
}

static void bhumanStart(const Settings& settings)
{
  fprintf(stderr, "B-Human: Start.\n");

  robot = new Robot(settings, std::string());
  robot->start();
}

static void bhumanStop()
{
  fprintf(stderr, "B-Human: Stop.\n");
  robot->announceStop();
  robot->stop();
  delete robot;
  robot = nullptr;
}

static void sigHandlerShutdown(int sig)
{
  if(run)
    fprintf(stderr, "Caught signal %i\nShutting down...\n", sig);
  run = false;
}

static void sigHandlerRedirect(int)
{
  timeWhenAskedToTerminate = Time::getCurrentSystemTime();
  run = false;
}

static void sigHandlerIgnore(int)
{
}

class EmergencyHandler : public Thread
{
  Semaphore stateChangedSignal; /**< Semaphore is triggered when the state has been changed. */
  Semaphore lowStateSignal; /**< Semaphore is triggered when a low state was received. */
  booster_interface::msg::LowCmd lowCmd; /**< Command to deactivate all joints. */
  booster::robot::ChannelPublisher<booster_interface::msg::LowCmd> lowCmdPublisher{booster::robot::b1::kTopicJointCtrl}; /**< Publisher for it. */
  Settings::RobotType robotType = Settings::k1; /**< Type of this robot. */

  /** The handler that switches between client modes for the K1. */
  void handler()
  {
    booster::robot::b1::GetModeResponse modeResponse;
    setPriority(21);
    while(isRunning())
    {
      // If state was changed or no low state was received recently.
      if(stateChangedSignal.wait(100) || !lowStateSignal.wait(10))
        while(isRunning())
        {
          switch(state)
          {
            case noEmergency:
            {
              if(lowStateSignal.wait(10))
              {
                if(!client.ChangeMode(booster::robot::RobotMode::kCustom))
                  goto success; // Leave switch and inner loop
                break;
              }
              else
                continue;
            }
            case emergencyPressed:
              // On the T1, wait for the Motion PC to switch to damping mode, then switch to emergencyReleased.
              if(robotType == Settings::t1 && !client.GetMode(modeResponse) && modeResponse.mode_ == booster::robot::RobotMode::kDamping)
              {
                resetMotorCommands();
                state = emergencyReleased;
                goto success;
              }
              [[fallthrough]];
            case emergencyReleased:
              // On the K1, switch to damping mode, both in emergencyPressed and emergencyReleased.
              // Nothing happens here for the T1, but the robotStatusHandler below waits for the joints
              // to reconnect.
              if(robotType == Settings::k1 && !client.ChangeMode(booster::robot::RobotMode::kDamping))
              {
                resetMotorCommands();
                goto success; // Leave switch and inner loop
              }
          }
          Thread::sleep(10);
        }
      success: ;
      }
  }

public:
  enum State {noEmergency, emergencyPressed, emergencyReleased} state = noEmergency;

  /**
   * Initialize the instance for the current robot type.
   * @param robotType The robot type.
   */
  void init(const Settings::RobotType robotType)
  {
    this->robotType = robotType;

    // Prepare motor commands with zero p and d values, i.e. motors off.
    lowCmd.cmd_type(booster_interface::msg::SERIAL);
    const size_t numOfJoints = booster::robot::b1::kJointCnt - (robotType == Settings::k1 ? 1 : 0);
    lowCmd.motor_cmd(std::vector<booster_interface::msg::MotorCmd>(numOfJoints));

    lowCmdPublisher.InitChannel();
  }

  /** Reset motor commands so that the robot does not move when activating the custom mode. */
  bool resetMotorCommands()
  {
    return lowCmdPublisher.Write(&lowCmd);
  }

  /**
   * Start handler.
   * @param robotType The type of robot the handler is required for.
   */
  void start()
  {
    Thread::start(this, &EmergencyHandler::handler);
  }

  /** Stop handler. */
  void stop()
  {
    announceStop();
    stateChangedSignal.post();
    Thread::stop();
    lowCmdPublisher.CloseChannel();
  }

  /**
   * Set state of handler.
   * @param state The new state.
   * @param trigger Will the handler be triggered to process the new state?
   */
  void setState(const State state, const bool trigger = true)
  {
    this->state = state;
    if(trigger)
      stateChangedSignal.post();
  }

  /** A low state was received. */
  void lowStateReceived()
  {
    while(lowStateSignal.tryWait());
    lowStateSignal.post();
  }
};

static EmergencyHandler emergencyHandler;

static void buttonEventHandler(const void* msg)
{
  const booster_interface::msg::ButtonEventMsg& buttonEvent = *reinterpret_cast<const booster_interface::msg::ButtonEventMsg*>(msg);
  if(buttonEvent.button() == 0) // F1
  {
    switch(emergencyHandler.state)
    {
      case EmergencyHandler::noEmergency:
        if(buttonEvent.event() == booster_interface::msg::PRESS_DOWN)
          emergencyHandler.setState(EmergencyHandler::emergencyPressed);
        break;
      case EmergencyHandler::emergencyPressed:
        if(buttonEvent.event() == booster_interface::msg::PRESS_UP)
          emergencyHandler.setState(EmergencyHandler::emergencyReleased, false);
        break;
      case EmergencyHandler::emergencyReleased:
        if(buttonEvent.event() == booster_interface::msg::LONG_PRESS_START)
          emergencyHandler.setState(EmergencyHandler::noEmergency);
    }
  }
}

static void lowStateHandler(const void*)
{
  emergencyHandler.lowStateReceived();
}

static void robotStatusHandler(const void* msg)
{
  const booster_interface::msg::RobotStatusDdsMsg& robotStatus = *reinterpret_cast<const booster_interface::msg::RobotStatusDdsMsg*>(msg);
  int connectedJoints = 0;
  for(const booster_interface::msg::RobotDdsJointStatus& joint : robotStatus.joint_vec())
    if(joint.is_connected())
      ++connectedJoints;

  switch(emergencyHandler.state)
  {
    case EmergencyHandler::noEmergency:
      if(!connectedJoints)
        emergencyHandler.setState(EmergencyHandler::emergencyPressed);
      break;
    case EmergencyHandler::emergencyReleased:
      if(connectedJoints == booster::robot::b1::kJointCnt)
        emergencyHandler.setState(EmergencyHandler::noEmergency);
  }
}

static Settings::RobotType getRobotType(const std::string& robotName)
{
  const std::string bhdir = File::getBHDir();
  InMapFile robotsStream(bhdir + "/Config/Robots/robots.cfg");
  if(!robotsStream.exists())
    fprintf(stderr, "Could not load robots.cfg\n");
  else
  {
    Robots robots;
    robotsStream >> robots;
    for(const Robots::RobotId& robot : robots.robotsIds)
      if(robot.name == robotName)
        return robot.robotType;
    fprintf(stderr, "Could not find robot name \"%s\" in robots.cfg!\n", robotName.c_str());
  }
  return Settings::k1;
}

static bool initInstance()
{
  // Initialize connection to robot at realtime priority.
  sched_param param;
  param.sched_priority = 21; // One more than Motion
  if(pthread_setschedparam(pthread_self(), SCHED_FIFO, &param))
  {
    fprintf(stderr, "Raising priority failed!\n");
    return false;
  }
  booster::robot::ChannelFactory::Instance()->Init(0);
  param.sched_priority = 0;
  if(pthread_setschedparam(pthread_self(), SCHED_OTHER, &param))
  {
    fprintf(stderr, "Lowering priority failed!\n");
    return false;
  }
  return true;
}

int main(int argc, char* argv[])
{
  booster::robot::ChannelSubscriber<booster_interface::msg::ButtonEventMsg> buttonEventSubscriber("rt/button_even2", &buttonEventHandler);
  booster::robot::ChannelSubscriber<booster_interface::msg::LowState> lowStateSubscriber(booster::robot::b1::kTopicLowState, &lowStateHandler);
  booster::robot::ChannelSubscriber<booster_interface::msg::RobotStatusDdsMsg> robotStatusSubscriber("rt/device_gateway", &robotStatusHandler);
  bool watchdog = false;
  {
    // Set stdout to be unbuffered.
    // This has previously been done using stdbuf, but this does not work for a 64-bit program on a 32-bit system.
    setvbuf(stdout, nullptr, _IONBF, 0);
    setvbuf(stderr, nullptr, _IONBF, 0);

    mainThread = pthread_self();

    // parse command-line arguments
    const char* bhDir = "/home/booster";

    for(int i = 1; i < argc; ++i)
      if(!strcmp(argv[i], "-c") && i + 1 < argc)
        bhDir = argv[++i];
      else if(!strcmp(argv[i], "-w"))
        watchdog = true;
      else
      {
        fprintf(stderr, "Usage: %s [-c <dir>] [-w]\n\
    -c <dir>      used bh directory (default is /home/booster)\n\
    -w            use a watchdog for crash recovery and creating trace dumps\n", argv[0]);
        exit(EXIT_FAILURE);
      }

    // avoid duplicated instances
    int fd = open("/tmp/bhuman", O_CREAT, 0600);
    if(fd == -1 || flock(fd, LOCK_EX | LOCK_NB) == -1)
    {
      fprintf(stderr, "There is already an instance of this process!\n");
      exit(EXIT_FAILURE);
    }

    // change working directory
    if(*bhDir && chdir(bhDir) != 0)
    {
      fprintf(stderr, "chdir to config directory failed!\n");
      exit(EXIT_FAILURE);
    }

    BH_TRACE_INIT("main");

    // Acquire static data, e.g. about types
    FunctionList::execute();

    const std::string name = SystemCall::getHostName();
    Settings settings(name, Constants::motionCycleTime, getRobotType(name));
    if(settings.playerNumber < 0 || settings.robotName.empty())
      return EXIT_FAILURE;

    // print status information
    printf("Hi, I am %s.\n", settings.robotName.c_str());
    printf("teamNumber %d\n", settings.teamNumber);
    printf("fieldPlayerColor %s\n", TypeRegistry::getEnumName(settings.fieldPlayerColor));
    printf("goalkeeperColor %s\n", TypeRegistry::getEnumName(settings.goalkeeperColor));
    printf("playerNumber %d\n", settings.playerNumber);
    printf("location %s\n", settings.location.c_str());
    printf("scenario %s\n", settings.scenario.c_str());
    printf("magicNumber %d\n", settings.magicNumber);

    struct Signal
    {
      const char* signalName = "/bhuman_body_connected";
      sem_t* signal;
      Signal() : signal(sem_open(signalName, O_CREAT, 0600, 0)) {}
      ~Signal() {if(signal) sem_unlink(signalName);}
    } connected;

    if(connected.signal == SEM_FAILED)
    {
      fprintf(stderr, "Creating semaphore failed!\n");
      return EXIT_FAILURE;
    }

    // the watchdog
    if(watchdog)
    {
      bhumanPid = fork();
      if(bhumanPid == -1)
        exit(EXIT_FAILURE);
    }

    if(bhumanPid)
    {
      signal(SIGTERM, sigHandlerRedirect);
      signal(SIGINT, sigHandlerRedirect);
      signal(SIGCHLD, sigHandlerIgnore);
    }

    if(!initInstance())
    {
      if(bhumanPid)
      {
        kill(bhumanPid, SIGTERM);
        sem_post(connected.signal);
      }
      return EXIT_FAILURE;
    }

    // Initialize the only instances in this process
    client.Init();
    Thread::sleep(100);
    if(settings.robotType == Settings::k1)
    {
      ledControl.Init();
      ledControlAvailable = true;
    }

    if(!watchdog || bhumanPid)
    {
      emergencyHandler.init(settings.robotType);

      // Deactivate all motors before switching to custom mode, because robot would otherwise apply previous commands.
      if(!emergencyHandler.resetMotorCommands())
      {
        fprintf(stderr, "Resetting motor commands failed!\n");
        if(bhumanPid)
        {
          kill(bhumanPid, SIGTERM);
          sem_post(connected.signal);
        }
        return EXIT_FAILURE;
      }

      // Listen to the F1 button on the K1 or to disconnected joints on the T1.
      if(settings.robotType == Settings::k1)
      {
        buttonEventSubscriber.InitChannel();
        emergencyHandler.setState(EmergencyHandler::noEmergency);
      }
      else
      {
        emergencyHandler.state = EmergencyHandler::emergencyReleased;
        robotStatusSubscriber.InitChannel();
      }
      lowStateSubscriber.InitChannel();

      // Start handler for handling emergency states.
      emergencyHandler.start();

      // Wait until the handler switched to custom mode.
      const unsigned startTime = Time::getCurrentSystemTime();
      booster::robot::b1::GetModeResponse modeResponse;
      bool waitingForBody = false;
      while(Time::getTimeSince(startTime) < 60000)
      {
        Thread::sleep(100);
        if(!client.GetMode(modeResponse) && modeResponse.mode_ == booster::robot::RobotMode::kCustom)
        {
          // When booting up, custom mode might be disabled again after a while. Wait a bit.
          if(waitingForBody)
            Thread::sleep(5000);
          goto success;
        }
        else if(!waitingForBody)
        {
          SystemCall::say("Waiting for robot body");
          waitingForBody = true;
        }
      }

      SystemCall::say("Failed to connect to robot body. Terminating.");
      if(bhumanPid)
      {
        kill(bhumanPid, SIGTERM);
        sem_post(connected.signal);
      }
      return EXIT_FAILURE;

    success:
      // Switch K1 back into damping mode
      if(settings.robotType == Settings::k1)
      {
        Thread::sleep(100);
        emergencyHandler.setState(EmergencyHandler::emergencyReleased);
      }
    }

    if(bhumanPid)
    {
      // Signal successful connection
      sem_post(connected.signal);

      sigset_t sigSet = {0};
      sigemptyset(&sigSet);
      sigaddset(&sigSet, SIGCHLD);
      siginfo_t sigInfo;
      const timespec timeout =
      {
        .tv_sec = 1,
        .tv_nsec = 0,
      };

      // Wait for main bhuman process to terminate
      while(!timeWhenAskedToTerminate || Time::getTimeSince(timeWhenAskedToTerminate) < 5000)
      {
        std::memset(&sigInfo, 0, sizeof(sigInfo));
        if(sigtimedwait(&sigSet, &sigInfo, &timeout) != -1 || errno == EINTR)
          break;
        else if(errno != EAGAIN)
        {
          kill(bhumanPid, SIGTERM);
          return EXIT_FAILURE;
        }
      }

      signal(SIGTERM, SIG_DFL);
      signal(SIGINT, SIG_DFL);
      signal(SIGCHLD, SIG_DFL);

      // detect requested or normal exit
      bool normalExit = !run || (WIFEXITED(sigInfo.si_status) && WEXITSTATUS(sigInfo.si_status) == EXIT_SUCCESS);

      // dump trace and assert trace
      if(!normalExit)
      {
        // Dump to file first, because writing to stderr may fail for various reasons.
        Assert::logDump(false, WIFSIGNALED(sigInfo.si_status) ? int(WTERMSIG(sigInfo.si_status)) : 0);
        Assert::logDump(true, WIFSIGNALED(sigInfo.si_status) ? int(WTERMSIG(sigInfo.si_status)) : 0);
      }

      // These calls are allowed even if the handlers were not started.
      buttonEventSubscriber.CloseChannel();
      lowStateSubscriber.CloseChannel();
      robotStatusSubscriber.CloseChannel();
      emergencyHandler.stop();

      // Switch Booster back to damping mode.
      const int result = client.ChangeMode(booster::robot::RobotMode::kDamping);
      if(result)
      {
        fprintf(stderr, "Switching robot back to damping mode failed with error %d!\n", result);
        return EXIT_FAILURE;
      }

      // Deactivate custom LED control.
      if(ledControlAvailable)
      {
        const int result = ledControl.StopLEDLightControl();
        if(result)
        {
          fprintf(stderr, "Deactivating custom LED control failed with error %d!\n", result);
          return EXIT_FAILURE;
        }
      }

      // quit
      exit(WIFEXITED(sigInfo.si_status) && normalExit ? WEXITSTATUS(sigInfo.si_status) : EXIT_FAILURE);
    }

    // This part is only reachable in the main bhuman process

    // register signal handler for ctrl+c and termination signal
    signal(SIGTERM, sigHandlerShutdown);
    signal(SIGINT, sigHandlerShutdown);

    // Wait for successful connection if established by the watchdog
    if(watchdog && sem_wait(connected.signal) < 0)
      return EXIT_FAILURE;

    if(run)
      bhumanStart(settings);

    // settings go out of scope here, but everything that needs it later makes a copy.
  }

  while(run)
    pause();

  if(robot)
    bhumanStop();

  if(!watchdog)
  {
    // These calls are allowed even if the handlers were not started.
    buttonEventSubscriber.CloseChannel();
    lowStateSubscriber.CloseChannel();
    robotStatusSubscriber.CloseChannel();
    emergencyHandler.stop();

    // Switch Booster back to damping mode.
    const int result = client.ChangeMode(booster::robot::RobotMode::kDamping);
    if(result)
    {
      fprintf(stderr, "Switching robot back to damping mode failed with error %d!\n", result);
      return EXIT_FAILURE;
    }

    // Deactivate custom LED control.
    if(ledControlAvailable)
    {
      const int result = ledControl.StopLEDLightControl();
      if(result)
      {
        fprintf(stderr, "Deactivating custom LED control failed with error %d!\n", result);
        return EXIT_FAILURE;
      }
    }
  }

  return EXIT_SUCCESS;
}
