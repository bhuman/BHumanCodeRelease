/**
 * @file Platform/Nao/Main.cpp
 * Implementation of the main() function for starting and stopping the module framework.
 * @author Colin Graf
 */

#include <csignal>
#include <cstdio>
#include <cstring>
#include <sys/file.h> // flock
#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

#include "Robot.h"
#include "NaoBody.h"
#include "Tools/FunctionList.h"
#include "Tools/Settings.h"
#include "libbhuman/bhuman.h"

static pid_t bhumanPid = 0;
static Robot* robot = nullptr;
static bool run = true;

static void bhumanStart()
{
  fprintf(stderr, "BHuman: Start.\n");

  robot = new Robot();
  robot->start();
}

static void bhumanStop()
{
  fprintf(stderr, "BHuman: Stop.\n");
  robot->announceStop();
  robot->stop();
  delete robot;
  robot = 0;
}

static void sighandlerShutdown(int sig)
{
  if(run)
    printf("Caught signal %i\nShutting down...\n", sig);
  run = false;
}

static void sighandlerRedirect(int sig)
{
  //if(bhumanPid != 0)
  //kill(bhumanPid, sig);
  run = false;
}

int main(int argc, char* argv[])
{
  {
    // parse command-line arguments
    bool background = false;
    bool watchdog = false;
    const char* bhDir = "/home/nao";

    for(int i = 1; i < argc; ++i)
      if(!strcmp(argv[i], "-b"))
        background = true;
      else if(!strcmp(argv[i], "-w"))
        watchdog = true;
      else if(!strcmp(argv[i], "-c") && i + 1 < argc)
        bhDir = argv[++i];
      else
      {
        fprintf(stderr, "Usage: %s [-b] [-c <dir>] [-w]\n\
    -b            run in background (as daemon)\n\
    -c <dir>      used gt directory (default is /home/nao)\n\
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

    // start as daemon
    if(background)
    {
      fprintf(stderr, "Starting as daemon...\n");
      pid_t childPid = fork();
      if(childPid == -1)
        exit(EXIT_FAILURE);
      if(childPid != 0)
        exit(EXIT_SUCCESS);
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
        // create pipe for logging
        int stdoutPipe[2];
        int stderrPipe[2];
        bool pipeReady = true;

        if(pipe(stdoutPipe) == -1 || pipe(stderrPipe) == -1)
        {
          fprintf(stderr, "B-Human: Error while creating pipes for logging. All logs will be printed on console only! \n");
          pipeReady = false;
        }

        bhumanPid = fork();
        if(bhumanPid == -1)
          exit(EXIT_FAILURE);
        if(bhumanPid != 0)
        {
          int status;
          signal(SIGTERM, sighandlerRedirect);
          signal(SIGINT, sighandlerRedirect);
          if(waitpid(bhumanPid, &status, 0) != bhumanPid)
          {
            exit(EXIT_FAILURE);
          }
          signal(SIGTERM, SIG_DFL);
          signal(SIGINT, SIG_DFL);

          if(pipeReady)
          {
            // close unused write end
            close(stdoutPipe[1]);
            close(stderrPipe[1]);

            dup2(STDOUT_FILENO, stdoutPipe[0]); // redirect out-pipe to stdout
            dup2(STDERR_FILENO, stderrPipe[0]); // redirect err-pipe to stderr
          }

          // detect requested or normal exit
          bool normalExit = !run || (WIFEXITED(status) && WEXITSTATUS(status) == EXIT_SUCCESS);

          // dump trace and assert trace
          if(!normalExit)
          {
            NaoBody naoBody;
            if(naoBody.init())
            {
              naoBody.setCrashed(WIFSIGNALED(status) ? int(WTERMSIG(status)) : int(abnormalTerminationState));
              naoBody.cleanup();
            }
            Assert::logDump(WIFSIGNALED(status) ? int(WTERMSIG(status)) : 0);
          }

          // quit
          exit(WIFEXITED(status) && normalExit ? WEXITSTATUS(status) : EXIT_FAILURE);
        }
        else
        {
          if(pipeReady)
          {
            // close unused read end
            close(stdoutPipe[0]);
            close(stderrPipe[0]);

            dup2(STDOUT_FILENO, stdoutPipe[1]); // redirect stdout to out-pipe
            dup2(STDERR_FILENO, stderrPipe[1]); // redirect stderr to err-pipe
          }
          break;
        }
      }
    }

    BH_TRACE_INIT("main");

    // Acquire static data, e.g. about types
    FunctionList::execute();

    // wait for NaoQi/libbhuman
    NaoBody naoBody;
    if(!naoBody.init())
    {
      fprintf(stderr, "B-Human: Waiting for NaoQi/libbhuman...\n");
      do
      {
        usleep(1000000);
      }
      while(!naoBody.init());
    }

    // load first settings instance
    Settings settings;

    if(!settings.loadingSucceeded())
      return EXIT_FAILURE;

    // register signal handler for strg+c and termination signal
    signal(SIGTERM, sighandlerShutdown);
    signal(SIGINT, sighandlerShutdown);

    //
    bhumanStart();
  }

  while(run)
    pause();

  bhumanStop();

  return EXIT_SUCCESS;
}
