/**
 * @file Main.cpp
 * Implementation of the main() function for starting and stopping the module framework.
 * @author Colin Graf
 */

#include <booster/idl/b1/LowCmd.h>
#include <booster/robot/b1/b1_loco_client.hpp>
#include <booster/robot/channel/channel_publisher.hpp>
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
#include "Platform/SystemCall.h"
#include "Streaming/FunctionList.h"

static pid_t bhumanPid = 0;
static Robot* robot = nullptr;
static bool run = true;
static pthread_t mainThread;
static bool shutdownBooster = false;

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

static void sighandlerShutdown(int sig)
{
  if(pthread_self() != mainThread)
  {
    shutdownBooster = true;
    pthread_kill(mainThread, sig);
  }
  else
  {
    if(run)
      fprintf(stderr, "Caught signal %i\nShutting down...\n", sig);
    run = false;
  }
}

static void sighandlerRedirect(int)
{
  run = false;
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
  return Settings::nao;
}

int main(int argc, char* argv[])
{
  booster::robot::b1::B1LocoClient client;
  {
    // Set stdout to be unbuffered.
    // This has previously been done using stdbuf, but this does not work for a 64-bit program on a 32-bit system.
    setvbuf(stdout, nullptr, _IONBF, 0);
    setvbuf(stderr, nullptr, _IONBF, 0);

    mainThread = pthread_self();

    // parse command-line arguments
    bool watchdog = false;
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

    // the watchdog
    if(watchdog)
    {
      for(;;)
      {
        bhumanPid = fork();
        if(bhumanPid == -1)
          exit(EXIT_FAILURE);
        else if(bhumanPid == 0)
          break;

        int status;
        signal(SIGTERM, sighandlerRedirect);
        signal(SIGINT, sighandlerRedirect);
        if(waitpid(bhumanPid, &status, 0) != bhumanPid)
        {
          exit(EXIT_FAILURE);
        }
        signal(SIGTERM, SIG_DFL);
        signal(SIGINT, SIG_DFL);

        // detect requested or normal exit
        bool normalExit = !run || (WIFEXITED(status) && WEXITSTATUS(status) == EXIT_SUCCESS);

        // dump trace and assert trace
        if(!normalExit)
        {
          // Dump to file first, because writing to stderr may fail for various reasons.
          Assert::logDump(false, WIFSIGNALED(status) ? int(WTERMSIG(status)) : 0);
          Assert::logDump(true, WIFSIGNALED(status) ? int(WTERMSIG(status)) : 0);

          // Switch Booster back to damping mode.
          booster::robot::ChannelFactory::Instance()->Init(0);
          booster::robot::b1::B1LocoClient client;
          client.Init();
          usleep(10000);
          const int result = client.ChangeMode(booster::robot::RobotMode::kDamping);
          if(result)
          {
            fprintf(stderr, "Switching robot back to damping mode failed with error %d!\n", result);
            return EXIT_FAILURE;
          }
        }

        // quit
        exit(WIFEXITED(status) && normalExit ? WEXITSTATUS(status) : EXIT_FAILURE);
      }
    }

    // Initialize connection to robot at realtime priority.
    sched_param param;
    param.sched_priority = 21; // One more than Motion
    if(pthread_setschedparam(pthread_self(), SCHED_FIFO, &param))
    {
      fprintf(stderr, "Raising priority failed!\n");
      return EXIT_FAILURE;
    }
    booster::robot::ChannelFactory::Instance()->Init(0);
    param.sched_priority = 0;
    if(pthread_setschedparam(pthread_self(), SCHED_OTHER, &param))
    {
      fprintf(stderr, "Lowering priority failed!\n");
      return EXIT_FAILURE;
    }

    client.Init();
    usleep(100000);

    // Prepare motor commands with zero p and d values, i.e. motors off.
    booster_interface::msg::LowCmd lowCmd;
    lowCmd.cmd_type(booster_interface::msg::SERIAL);
    lowCmd.motor_cmd(std::vector<booster_interface::msg::MotorCmd>(booster::robot::b1::kJointCnt));

    // Deactivate all motors before switching to custom mode, because robot would otherwise apply previous commands.
    booster::robot::ChannelPublisher<booster_interface::msg::LowCmd> lowCmdPublisher(booster::robot::b1::kTopicJointCtrl);
    lowCmdPublisher.InitChannel();
    if(!lowCmdPublisher.Write(&lowCmd))
    {
      fprintf(stderr, "Resetting motor commands failed!\n");
      return EXIT_FAILURE;
    }

    BH_TRACE_INIT("main");

    // Acquire static data, e.g. about types
    FunctionList::execute();

    const std::string name = SystemCall::getHostName();
    Settings settings(name, name, Constants::boosterMotionCycleTime, getRobotType(name));
    if(settings.playerNumber < 0 || settings.bodyName.empty())
      return EXIT_FAILURE;

    // print status information
    printf("Hi, I am %s.\n", settings.headName.c_str());
    printf("teamNumber %d\n", settings.teamNumber);
    printf("fieldPlayerColor %s\n", TypeRegistry::getEnumName(settings.fieldPlayerColor));
    printf("goalkeeperColor %s\n", TypeRegistry::getEnumName(settings.goalkeeperColor));
    printf("playerNumber %d\n", settings.playerNumber);
    printf("location %s\n", settings.location.c_str());
    printf("scenario %s\n", settings.scenario.c_str());
    printf("magicNumber %d\n", settings.magicNumber);

    // put the current body name in a file (at the moment only used in downloadCalibration)
    OutTextRawFile("/var/volatile/tmp/bodyName.txt") << settings.bodyName;

    // register signal handler for ctrl+c and termination signal
    signal(SIGTERM, sighandlerShutdown);
    signal(SIGINT, sighandlerShutdown);

    bhumanStart(settings);

    // settings go out of scope here, but everything that needs it later makes a copy.
  }

  while(run)
    pause();

  bhumanStop();

  const int result = client.ChangeMode(booster::robot::RobotMode::kDamping);
  if(result)
    fprintf(stderr, "Switching robot back to damping mode failed with error %d!\n", result);

  if(shutdownBooster)
    static_cast<void>(!system("sudo systemctl poweroff &"));

  return EXIT_SUCCESS;
}
