/**
 * @file Main.cpp
 * Implementation of the main() function for starting and stopping the module framework.
 * @author Colin Graf
 */

#include <algorithm>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <sys/file.h> // flock
#include <fcntl.h>
#include <sys/wait.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "Framework/Robot.h"
#include "Framework/Robots.h"
#include "Framework/Settings.h"
#include "Math/Angle.h"
#include "Math/Constants.h"
#include "Platform/File.h"
#include "Platform/SystemCall.h"
#include "RobotParts/Joints.h"
#include "Streaming/FunctionList.h"
#include "Streaming/InStreams.h"
#include "Streaming/OutStreams.h"
#include "Tools/Communication/MsgPack.h"

static pid_t bhumanPid = 0;
static Robot* robot = nullptr;
static bool run = true;
static pthread_t mainThread;
static bool shutdownNAO = false;

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
    shutdownNAO = true;
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

static std::string getBodyId()
{
  int socket = ::socket(AF_UNIX, SOCK_STREAM, 0);
  sockaddr_un address;
  address.sun_family = AF_UNIX;
  std::strcpy(address.sun_path, "/tmp/robocup");
  if(connect(socket, reinterpret_cast<sockaddr*>(&address), sizeof(address)))
  {
    fprintf(stderr, "Waiting for LoLA... ");
    while(connect(socket, reinterpret_cast<sockaddr*>(&address), sizeof(address)))
      usleep(100000);
    fprintf(stderr, "OK\n");
  }

  std::string bodyId;

  // Receive a single packet and extract serial number of body.
  unsigned char receivedPacket[896];
  long bytesReceived = recv(socket, reinterpret_cast<char*>(receivedPacket), sizeof(receivedPacket), 0);
  if(bytesReceived >= 0)
    MsgPack::parse(receivedPacket, bytesReceived,
                   [](const std::string&, const unsigned char*) {},
                   [](const std::string&, const unsigned char*) {},
                   [&bodyId](const std::string& key, const unsigned char* value, size_t valueSize)
                   {
                     if(key == "RobotConfig:0" && valueSize)
                     {
                       bodyId.resize(valueSize);
                       std::strncpy(&bodyId[0], reinterpret_cast<const char*>(value), valueSize);
                     }
                   });
  close(socket);

  return bodyId;
}

static void sitDown(bool ok)
{
  static constexpr Joints::Joint numOfJoints = static_cast<Joints::Joint>(Joints::numOfJoints - 2);
  static const Angle targetAngles[numOfJoints] =
  {
    0_deg, 0_deg, // Head

    51_deg, 3_deg, 15_deg, -36_deg, -90_deg,  // Left arm
    0_deg, 0_deg, -50_deg, 124_deg, -68_deg, 0_deg, // HipYawPitch and left leg

    0_deg, -50_deg, 124_deg, -68_deg, 0_deg, // Right leg
    51_deg, -3_deg, -15_deg, 36_deg, 90_deg, // Right arm

    0_deg, 0_deg // Hands
  };

  int socket = ::socket(AF_UNIX, SOCK_STREAM, 0);
  sockaddr_un address;
  address.sun_family = AF_UNIX;
  std::strcpy(address.sun_path, "/tmp/robocup");
  if(!connect(socket, reinterpret_cast<sockaddr*>(&address), sizeof(address)))
  {
    unsigned char packet[896];
    long bytesReceived = recv(socket, reinterpret_cast<char*>(packet), static_cast<int>(sizeof(packet)), 0);

    // Determine current angles and whether sitting down is required, i.e. has the hip stiffness?
    bool sitDownRequired = false;
    Angle startAngles[numOfJoints];
    std::array<float, 3> torsoAngles; /**< The addresses of torso angle data inside receivedPacket. */

    MsgPack::parse(packet, bytesReceived,
                   [&sitDownRequired, &startAngles](const std::string& key, const unsigned char* p)
                   {
                     const std::string::size_type pos = key.find(":");
                     ASSERT(pos != std::string::npos);
                     const std::string category = key.substr(0, pos);
                     const int index = std::stoi(key.substr(pos + 1));

                     if(category == "Position")
                       startAngles[index] = MsgPack::readFloat(p);
                     else if(category == "Stiffness" && (index == 8 || index == 13) && MsgPack::readFloat(p) > 0.f)
                       sitDownRequired = true;
                   },

                   // Ignore ints and strings
                   [](const std::string&, const unsigned char*) {},
                   [](const std::string&, const unsigned char*, size_t) {});

    // If sitting down is required, interpolate from start angles to target angles.
    if(sitDownRequired)
    {
      float ratio = 0.f;
      while(ratio < 1.f)
      {
        // Do sinus interpolation
        const float phase = 0.5f * std::sin(ratio * Constants::pi - Constants::pi / 2.f) + 0.5f;

        // Get torso angles from IMU
        MsgPack::parse(packet, bytesReceived,
                       [&torsoAngles](const std::string& key, const unsigned char* p)
                       {
                         const std::string::size_type pos = key.find(":");
                         ASSERT(pos != std::string::npos);
                         const std::string category = key.substr(0, pos);
                         const int index = std::stoi(key.substr(pos + 1));
                         if(category == "Angles")
                           torsoAngles[index] = MsgPack::readFloat(p);
                       },
                       // Ignore ints and strings
                       [](const std::string&, const unsigned char*) {},
                       [](const std::string&, const unsigned char*, size_t) {});

        unsigned char* p = packet;
        // Reduce stiffness if robot is falling
        if(std::abs(torsoAngles[0]) > 45_deg || std::abs(torsoAngles[1]) > 45_deg)
        {
          MsgPack::writeMapHeader(2, p);
          MsgPack::write("Stiffness", p);
          MsgPack::writeArrayHeader(numOfJoints, p);

          for(int i = 0; i < numOfJoints; ++i)
            MsgPack::write(0.2f, p);
        }
        else
          MsgPack::writeMapHeader(1, p);

        MsgPack::write("Position", p);
        MsgPack::writeArrayHeader(numOfJoints, p);
        // The shoulder pitch joints interpolate faster to avoid collisions of the arms with the legs.
        const float shoulderPitchPhase = std::sqrt(std::min(1.f, ratio / 0.6f));
        for(int i = 0; i < numOfJoints; ++i)
        {
          if(i == 2 || i == 18)
            MsgPack::write(targetAngles[i] * shoulderPitchPhase + startAngles[i] * (1.f - shoulderPitchPhase), p);
          else
            MsgPack::write(targetAngles[i] * phase + startAngles[i] * (1.f - phase), p);
        }

        // Send packet to LoLA
        send(socket, reinterpret_cast<char*>(packet), static_cast<int>(p - packet), 0);

        // Receive next packet (required for sending again)
        bytesReceived = recv(socket, reinterpret_cast<char*>(packet), static_cast<int>(sizeof(packet)), 0);

        ratio += Constants::naoMotionCycleTime / 2.f; // 2 seconds
      }
    }

    // Switch off stiffness of all joints
    unsigned char* p = packet;
    MsgPack::writeMapHeader(9, p);
    MsgPack::write("Stiffness", p);
    MsgPack::writeArrayHeader(numOfJoints, p);
    for(int i = 0; i < numOfJoints; ++i)
      MsgPack::write(0.f, p);

    // Switch off all leds, except for the eyes (ok: blue, crashed: red)
    static std::string ledCategories[8] = {"LEye", "REye", "LEar", "REar", "Skull", "Chest", "LFoot", "RFoot"};
    static size_t ledNumbers[8] = {24, 24, 10, 10, 12, 3, 3, 3};
    for(int i = 0; i < 8; ++i)
    {
      MsgPack::write(ledCategories[i], p);
      MsgPack::writeArrayHeader(ledNumbers[i], p);
      for(size_t j = 0; j < ledNumbers[i]; ++j)
      {
        bool isRed = i < 2 && j < 8;
        bool isBlue = i < 2 && j >= 16;
        MsgPack::write(ok && isBlue ? 0.1f : !ok && isRed ? 1.f : 0.f, p);
      }
    }

    // Send packet to LoLA
    send(socket, reinterpret_cast<char*>(packet), static_cast<int>(p - packet), 0);
  }
  close(socket);
}

static std::string getBodyName(const std::string& bodyId)
{
  std::string bodyName;
  std::string bhdir = File::getBHDir();
  InMapFile robotsStream(bhdir + "/Config/Robots/robots.cfg");
  if(!robotsStream.exists())
    fprintf(stderr, "Could not load robots.cfg\n");
  else
  {
    Robots robots;
    robotsStream >> robots;
    for(const Robots::RobotId& robot : robots.robotsIds)
      if(robot.robotType == Settings::nao && robot.bodyId == bodyId)
      {
        bodyName = robot.name;
        return bodyName;
      }
    fprintf(stderr, "Could not find bodyName in robots.cfg! BodyId: %s\nAssuming \"Default\".\n", bodyId.c_str());
    bodyName = "Default";
  }
  return bodyName;
}

int main(int argc, char* argv[])
{
  {
    // Set stdout to be unbuffered.
    // This has previously been done using stdbuf, but this does not work for a 64-bit program on a 32-bit system.
    setvbuf(stdout, nullptr, _IONBF, 0);
    setvbuf(stderr, nullptr, _IONBF, 0);

    mainThread = pthread_self();

    // parse command-line arguments
    bool watchdog = false;
    const char* bhDir = "/home/nao";

    for(int i = 1; i < argc; ++i)
      if(!strcmp(argv[i], "-c") && i + 1 < argc)
        bhDir = argv[++i];
      else if(!strcmp(argv[i], "-w"))
        watchdog = true;
      else
      {
        fprintf(stderr, "Usage: %s [-c <dir>] [-w]\n\
    -c <dir>      used bh directory (default is /home/nao)\n\
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
          // Wait 100 ms before attempting to sit down. Otherwise, LoLA might send an invalid packet.
          usleep(100000);

          sitDown(false);

          // Dump to file first, because writing to stderr may fail for various reasons.
          Assert::logDump(false, WIFSIGNALED(status) ? int(WTERMSIG(status)) : 0);
          Assert::logDump(true, WIFSIGNALED(status) ? int(WTERMSIG(status)) : 0);
        }

        // quit
        exit(WIFEXITED(status) && normalExit ? WEXITSTATUS(status) : EXIT_FAILURE);
      }
    }

    BH_TRACE_INIT("main");

    // Acquire static data, e.g. about types
    FunctionList::execute();

    Settings settings(SystemCall::getHostName(), getBodyName(getBodyId()), Constants::naoMotionCycleTime, Settings::nao);
    if(settings.playerNumber < 0 || settings.bodyName.empty())
      return EXIT_FAILURE;

    // print status information
    if(settings.headName == settings.bodyName)
      printf("Hi, I am %s.\n", settings.headName.c_str());
    else
      printf("Hi, I am %s (using %s's body).\n", settings.headName.c_str(), settings.bodyName.c_str());
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
  sitDown(true);
  if(shutdownNAO)
    static_cast<void>(!system("sudo systemctl poweroff &"));

  return EXIT_SUCCESS;
}
